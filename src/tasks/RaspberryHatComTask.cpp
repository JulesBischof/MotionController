#include "MotionController.hpp"

#include <stdio.h>
#include <cstdint>
#include <cstring>

#include "pico/stdlib.h"

#include "prain_uart/decoder.hpp"
#include "prain_uart/encoder.hpp"

#include "MotionControllerConfig.hpp"
#include "MotionControllerPinning.hpp"

#include "RaspberryHatComTaskConfig.hpp"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"


using namespace prain_uart;
using namespace encoder;

namespace MtnCtrl
{
    void MotionController::_raspberryHatComTask()
    {
        // init Rx interrupts
        _initUartRxIsr(UART_INSTANCE_RASPBERRYHAT, _uart0RxIrqHandler);

        // get QueueHandles
        QueueHandle_t raspberryHatComQueue = _raspberryHatComQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;

        // loop forever
        for (;;)
        {

            DispatcherMessage message;
            DispatcherMessage uartMsg;

            // suspend Task until Message is recieved
            if (xQueueReceive(raspberryHatComQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                if (message.receiverTaskId != DispatcherTaskId::RaspberryHatComTask)
                {
                    services::LoggerService::error("_raspberryHatComTask", "Message contains wrong Task ID");
                    continue;
                }

                frame txMsg;

                // check incoming commands - messages supposed to go out
                switch (message.command)
                {
                case (TaskCommand::Info):
                    txMsg = encode_info(address::RASPBERRY_HAT, message.getData());
                    break;
                case (TaskCommand::Error):
                    txMsg = encode_error(address::RASPBERRY_HAT, message.getData());
                    break;
                case (TaskCommand::PollDistance):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DISTANCE, message.getData());
                    break;
                case (TaskCommand::PollLineSensor):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::LINE_SENSOR, message.getData());
                    break;
                case (TaskCommand::PollDegree):
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DEGREE, message.getData());
                    break;
                case (TaskCommand::Pong):
                    txMsg = encode_pong(address::RASPBERRY_HAT, message.getData());
                    break;
                case (TaskCommand::Ping):
                    txMsg = encode_pong(address::RASPBERRY_HAT, message.getData());
                    break;
                case (TaskCommand::PollStatusFlags):
                    // TODO: prain_uart poll_id
                    break;
                case (TaskCommand::DecodeMessage):
                    uartMsg = _getCommand(UART_INSTANCE_RASPBERRYHAT);
                    
                    if (xQueueSend(messageDispatcherQueue, &uartMsg, portMAX_DELAY) != pdTRUE)
                    {
                        services::LoggerService::fatal("_raspberryHatComTask", "WRITE TO QUEUE cmd decode msg FAILED");
                        for(;;)
                            ;
                    }
                    break;
                default:
                    break;
                }
                // send msg to RaspberryPi
                if (txMsg.raw() != 0)
                {
                    sendUartMsg(&txMsg, UART_INSTANCE_RASPBERRYHAT);
                }
            }
        }
    }

    void MotionController::_uart0RxIrqHandler()
    {
        /* some message arrived!! send read command to RaspberryHatComQueue
        buffer-read and decoding shouldn't be handled inside a isr */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // access is atomic - no protection neccessary
        QueueHandle_t queueHandle = _raspberryHatComQueue;

        if (queueHandle != nullptr)
        {
            DispatcherMessage msg(
                DispatcherTaskId::RaspberryHatComTask,
                DispatcherTaskId::RaspberryHatComTask,
                TaskCommand::DecodeMessage,
                0);

            xQueueSendFromISR(queueHandle, &msg, &xHigherPriorityTaskWoken);
        }
        // disable uart interrupts
        uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, false, false);

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    DispatcherMessage MotionController::_getCommand(uart_inst_t *uartId)
    {
        if (!uart_is_readable(uartId))
        {
            DispatcherMessage msg(
                DispatcherTaskId::RaspberryHatComTask,
                DispatcherTaskId::RaspberryHatComTask,
                TaskCommand::Error,
                static_cast<uint64_t>(error_code::INTERNAL));
            services::LoggerService::fatal("MotionController::_getCommand", "Uart not readable");
            return msg;
        }

        char rawMessage[8] = {0};

        // LSB FIRST
        for (int i = 0; uart_is_readable_within_us(uartId, 1000) && i < 8; i++)
        {
            rawMessage[i] = uart_getc(uartId);
        }

        // // MSB FIRST
        // for (int i = 8; uart_is_readable_within_us(UART_INSTANCE_RASPBERRYHAT, 1000) && i >= 0; i--)
        // {
        //     rawMessage[i] = uart_getc(UART_INSTANCE_RASPBERRYHAT);
        // }

        uint64_t rawFrame = 0;
        std::memcpy(&rawFrame, rawMessage, sizeof(uint64_t));

        // get address
        decoder dec = decoder(rawFrame);
        address addr = dec.get_address();

        DispatcherMessage retVal;
        retVal.senderTaskId = (uartId == UART_INSTANCE_RASPBERRYHAT) ? DispatcherTaskId::RaspberryHatComTask : DispatcherTaskId::GripControllerComTask;

        bool crcCheck = dec.verify_crc();
        command cmd = dec.get_command();

        // Message is meant for Grip Controller
        if (addr == address::GRIP_CTRL)
        {
            retVal.receiverTaskId = DispatcherTaskId::GripControllerComTask;
            retVal.command = TaskCommand::HandThroughMessage;
            retVal.setData(rawFrame);
        } // end msg grpcntrl

        // message is meant for Motion Controller
        if (addr == address::MOTION_CTRL)
        {
            if (!crcCheck)
            {
                retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                retVal.command = TaskCommand::Error;
                retVal.setData(static_cast<uint64_t>(error_code::INVALID_CRC));
                return retVal;
            }

            // if crc is correct, disassemble message
            switch (cmd)
            {
            case (command::MOVE):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Move;
                retVal.setData(dec.get_raw_parameters());
                break;

            case (command::REVERSE):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Reverse;
                retVal.setData(dec.get_raw_parameters());
                break;

            case (command::STOP):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Stop;
                retVal.setData(0);
                break;

            case (command::TURN):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::Turn;
                retVal.setData(dec.get_params<turn_params>().angle);
                break;

            case (command::PING):
                retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                retVal.command = TaskCommand::Pong;
                retVal.setData(0);
                break;

            case (command::RESPONSE):
                retVal.receiverTaskId = DispatcherTaskId::LineFollowerTask;
                retVal.command = TaskCommand::GcAck;
                retVal.setData(0);
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
                case static_cast<uint8_t>(poll_id::ULTRASONIC):
                    retVal.command = TaskCommand::PollUltrasonic;
                    break;
                default:
                    break;
                }
                break;
            default:
                // TODO: define Error code
                // retVal.receiverTaskId = DispatcherTaskId::RaspberryHatComTask;
                // retVal.command = TaskCommand::Error;
                // retVal.setData(0);
                break;
            }
        } // end msg mtnctr

        // flush rx FIFO
        while (uart_is_readable(uartId))
        {
            uart_getc(uartId);
        }

        // enable uart interrupts again
        uart_set_irq_enables(uartId, true, false);

        return retVal;
    } // end _getCommand
}