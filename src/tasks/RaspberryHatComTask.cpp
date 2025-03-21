#include "MotionController.hpp"

#include "queues.h"
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

void MotionController::_raspberryHatComTask()
{
    // loop forever
    for (;;)
    {
        dispatcherMessage_t message;

        // suspend Task until Message is recieved
        if (xQueueReceive(_raspberryHatComQueue, &message, portMAX_DELAY) == pdTRUE)
        {
            if (message.recieverTaskId != TASKID_RASPBERRY_HAT_COM_TASK)
            {
                printf("RASPBERRYHATCOMTASK - Message contains wrong Task ID \n");
                continue;
            }
            frame txMsg;
            // check incoming commands - messages supposed to go out
            switch (message.command)
            {
            case (COMMAND_INFO):
                txMsg = encode_info(address::RASPBERRY_HAT, message.data);
                break;
            case (COMMAND_ERROR):
                txMsg = encode_error(address::RASPBERRY_HAT, message.data);
                break;
            case (COMMAND_POLL_DISTANCE):
                txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DISTANCE, message.data);
                break;
            case (COMMAND_POLL_LINE_SENSOR):
                txMsg = encode_response(address::RASPBERRY_HAT, poll_id::LINE_SENSOR, message.data);
                break;
            case (COMMAND_POLL_DEGREE):
                txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DEGREE, message.data);
                break;
            case (COMMAND_POLL_STATUSFLAGS):
                // TODO: missing in prain_uart poll_id
                break;
            case (COMMAND_DECODE_MESSAGE):
                dispatcherMessage_t msg;
                msg = _getCommand(UART_INSTANCE_RASPBERRYHAT);
                xQueueSend(_messageDispatcherQueue, &msg, 10);
                break;
            default:
                break;
            }
            sendUartMsg(&txMsg, UART_INSTANCE_RASPBERRYHAT);
        }
    }
}

dispatcherMessage_t MotionController::_getCommand(uart_inst_t *uartId)
{
    if (!uart_is_readable(UART_INSTANCE_RASPBERRYHAT))
    {
        dispatcherMessage_t msg;
        msg.recieverTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
        msg.senderTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
        msg.data = 0;
        msg.command = COMMAND_ERROR;
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

    dispatcherMessage_t retVal;
    retVal.senderTaskId = TASKID_RASPBERRY_HAT_COM_TASK;

    // Message is meant for Grip Controller
    if (dec.get_address() == address::GRIP_CTRL)
    {
        retVal.recieverTaskId = TASKID_GRIPCONTROLLER_COM_TASK;
        retVal.command = COMMAND_HAND_THROUGH_MESSAGE;
        retVal.data = rawMessage;
    } // end msg grpcntrl

    // message is meant for Motion Controller
    if (dec.get_address() == address::MOTION_CTRL)
    {
        if (!dec.verify_crc())
        {
            retVal.recieverTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
            retVal.command = COMMAND_ERROR;
            retVal.data = static_cast<uint64_t>(error_code::INVALID_CRC);
            return retVal;
        }

        // if crc is correct, disassemble message
        switch (dec.get_command())
        {
        case (command::MOVE):
            retVal.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;
            retVal.command = COMMAND_MOVE;
            retVal.data = dec.get_raw_parameters();
            break;

        case (command::REVERSE):
            retVal.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;
            retVal.command = COMMAND_REVERSE;
            retVal.data = dec.get_raw_parameters();
            break;

        case (command::STOP):
            retVal.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;
            retVal.command = COMMAND_STOP;
            retVal.data = 0;
            break;

        case (command::TURN):
            retVal.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;
            retVal.command = COMMAND_TURN;
            break;

        case (command::PING):
            retVal.recieverTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
            retVal.command = COMMAND_PONG;
            break;

        case (command::POLL):
            retVal.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;

            // determine, what kind of data is requested
            switch (dec.get_params<poll_params>().poll_id)
            {
            case static_cast<uint8_t>(poll_id::DEGREE):
                retVal.command = COMMAND_POLL_DEGREE;
                break;
            case static_cast<uint8_t>(poll_id::DISTANCE):
                retVal.command = COMMAND_POLL_DISTANCE;
                break;
            case static_cast<uint8_t>(poll_id::LINE_SENSOR):
                retVal.command = COMMAND_POLL_LINE_SENSOR;
                break;
            default:
                break;
            }
            break;
        default:
            // TODO: define Error code
            retVal.recieverTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
            retVal.command = COMMAND_ERROR;
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