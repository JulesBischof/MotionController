#include "MotionController.hpp"

#include <stdio.h>

#include "pico/stdlib.h"

#include "prain_uart/decoder.hpp"
#include "prain_uart/encoder.hpp"

#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

#include "RaspberryHatComTaskConfig.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

using namespace prain_uart;
using namespace encoder;

namespace MotionController
{

    void MotionController::_raspberryHatComTask()
    {
        // loop forever
        for (;;)
        {
            DispatcherMessage message(DispatcherTaskId::NoTask, DispatcherTaskId::RaspberryHatComTask, TaskCommand::NoCommand);

            // suspend Task until Message is recieved
            if (xQueueReceive(_raspberryHatComQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                if (message.receiverTaskId != DispatcherTaskId::RaspberryHatComTask)
                {
                    printf("RASPBERRYHATCOMTASK - Message contains wrong Task ID \n");
                    continue;
                }
                frame txMsg;
                // check incoming commands - messages supposed to go out
                switch (message.command)
                {
                case (TaskCommand::Info):
                    txMsg = encode_info(address::RASPBERRY_HAT, message.data);
                    break;
                case (TaskCommand::Error):
                    txMsg = encode_error(address::RASPBERRY_HAT, message.data);
                    break;
                case (TaskCommand::PollDistance):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DISTANCE, message.data);
                    break;
                case (TaskCommand::PollLineSensor):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::LINE_SENSOR, message.data);
                    break;
                case (TaskCommand::PollDegree):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DEGREE, message.data);
                    break;
                case (TaskCommand::PollStatusFlags):
                    // TODO: missing in prain_uart poll_id
                    break;
                case (TaskCommand::DecodeMessage):
                    DispatcherMessage msg = _getCommand(UART_INSTANCE_RASPBERRYHAT);
                    xQueueSend(_messageDispatcherQueue, &msg, 10);
                    break;
                default:
                    break;
                }
                sendUartMsg(&txMsg, UART_INSTANCE_RASPBERRYHAT);
            }
        }
    }

    DispatcherMessage MotionController::_getCommand(uart_inst_t *uartId)
    {
        if (!uart_is_readable(UART_INSTANCE_RASPBERRYHAT))
        {
            DispatcherMessage msg(
                DispatcherTaskId::RaspberryHatComTask,
                DispatcherTaskId::RaspberryHatComTask,
                TaskCommand::Error,
                0);
            printf("ERROR _getuartMsg");
            return msg;
        }

        uint64_t rawMessage = 0;

        // IMPLEMENTATION MSB FIRST
        // for (int i = 0; uart_is_readable(UART_INSTANCE_RASPBERRYHAT) && i != PROTOCOL_SIZE_IN_BYTES; i++)
        // {
        //     rawMessage |= (uart_getc(UART_INSTANCE_RASPBERRYHAT) << ((PROTOCOL_SIZE_IN_BYTES - 1 - i) * 8));
        // }

        // LSB FIRST
        for (int i = 0; uart_is_readable_within_us(UART_INSTANCE_RASPBERRYHAT, 1000) && i < PROTOCOL_SIZE_IN_BYTES; i++)
        {
            rawMessage |= ((uint64_t)uart_getc(UART_INSTANCE_RASPBERRYHAT) << (i * 8));
        }

        printf("raw Message read");

        decoder dec = decoder(rawMessage);

        DispatcherMessage retVal(DispatcherTaskId::NoTask, DispatcherTaskId::RaspberryHatComTask, TaskCommand::NoCommand);
        retVal.senderTaskId = DispatcherTaskId::RaspberryHatComTask;

        // Message is meant for Grip Controller
        if (dec.get_address() == address::GRIP_CTRL)
        {
            retVal.receiverTaskId = DispatcherTaskId::GripControllerComTask;
            retVal.command = TaskCommand::HandThroughMessage;
            retVal.data = rawMessage;
        } // end msg grpcntrl

        // message is meant for Motion Controller
        if (dec.get_address() == address::MOTION_CTRL)
        {
            if (!dec.verify_crc())
            {
                retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                retVal.command = TaskCommand::Error;
                retVal.data = static_cast<uint64_t>(error_code::INVALID_CRC);
                return retVal;
            }

            // if crc is correct, disassemble message
            switch (dec.get_command())
            {
            case (command::MOVE):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Move;
                retVal.data = dec.get_raw_parameters();
                break;

            case (command::REVERSE):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Reverse;
                retVal.data = dec.get_raw_parameters();
                break;

            case (command::STOP):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Stop;
                retVal.data = 0;
                break;

            case (command::TURN):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Turn;
                break;

            case (command::PING):
                retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                retVal.command = TaskCommand::Pong;
                break;

            case (command::POLL):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;

                // determine, what kind of data is requested
                switch (dec.get_params<poll_params>().poll_id)
                {
                case static_cast<uint8_t>(poll_id::DEGREE):
                    retVal.command = TaskCommand::PollDegree;
                    break;
                case static_cast<uint8_t>(poll_id::DISTANCE):
                    retVal.command = TaskCommand::PollDistance;
                    break;
                case static_cast<uint8_t>(poll_id::LINE_SENSOR):
                    retVal.command = TaskCommand::PollLineSensor;
                    break;
                default:
                    break;
                }
                break;
            default:
                // TODO: define Error code
                retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                retVal.command = TaskCommand::Error;
                retVal.data = 0;
                break;
            }
        } // end msg mtnctr

        // flush rx FIFO
        while (uart_is_readable(UART_INSTANCE_RASPBERRYHAT))
        {
            uart_getc(UART_INSTANCE_RASPBERRYHAT);
        }

        // enable uart interrupts again
        uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);

        return retVal;
    } // end _getCommand
}