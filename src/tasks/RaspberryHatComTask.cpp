#include "RaspberryHatComTask.hpp"

#include <stdio.h>

#include <pico/stdlib.h>
#include <hardware/uart.h>

#include "MotionControllerConfig.h"

#include "RaspberryHatComTaskConfig.h"

#include "prain_uart/encoder.hpp"
#include "prain_uart/decoder.hpp"
#include "prain_uart/helper.hpp"
#include "prain_uart/protocol.hpp"

using namespace prain_uart;
using namespace encoder;

/* ================================= */
/*         static Members            */
/* ================================= */
RaspberryHatComTask RaspberryHatComTask::*_instance = nullptr;

TaskHandle_t RaspberryHatComTask::_taskHandle = nullptr;
QueueHandle_t RaspberryHatComTask::_dispatcherQueue = nullptr;
QueueHandle_t RaspberryHatComTask::_raspberryHatComQueue = nullptr;
xSemaphoreHandle RaspberryHatComTask::_uartRxSemaphore = nullptr;

/* ================================= */
/*         implementation            */
/* ================================= */

RaspberryHatComTask::RaspberryHatComTask(QueueHandle_t *dispatcherQueue)
{
    // set rx irq
    uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);
    irq_set_exclusive_handler(UART0_IRQ, _uartRxIrqHandler);
    irq_set_enabled(UART0_IRQ, true);

    // init static members
    _dispatcherQueue = *dispatcherQueue;
    _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));
    _uartRxSemaphore = xSemaphoreCreateCounting(100, 0);
    xTaskCreate(_run, RASPBERRYHATCOMTASK_NAME, RASPBERRYHATCOMTASK_STACKSIZE, this, RASPBERRYHATCOMTASK_PRIORITY, &_taskHandle);
}

void RaspberryHatComTask::_run(void *pvParameters)
{
    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(_dispatcherQueue) > 0 || uxSemaphoreGetCount(_uartRxSemaphore) >= PROTOCOL_SIZE_IN_BYTES)
        {
            // check if there is some data to send
            dispatcherMessage_t message;
            if (xQueueReceive(_raspberryHatComQueue, &message, pdMS_TO_TICKS(10)) == pdTRUE)
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
                    txMsg = encode_response(address::RASPBERRY_HAT, poll_id::DISTANCE,message.data);
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
                default:
                    break;
                }
                sendUartMsg(&txMsg);
            } // end queuemessage recieved

            // check if one package is stored inside uart rx buffer
            if (uxSemaphoreGetCount(_uartRxSemaphore) >= PROTOCOL_SIZE_IN_BYTES)
            {
                xSemaphoreTake(_uartRxSemaphore, 0);
                dispatcherMessage_t msg;
                msg = _getUartMsg();
            }
        }
    }
}

dispatcherMessage_t RaspberryHatComTask::_getUartMsg()
{
    // TODO: does the messega get into read buffer MSB or LSB first?
    uint64_t rawMessage = 0;
    for (size_t i = 0; i < PROTOCOL_SIZE_IN_BYTES; i++)
    {
        rawMessage |= (uart_getc(UART_INSTANCE_RASPBERRYHAT) << i);
    }

    decoder dec = decoder(rawMessage);

    dispatcherMessage_t retVal;
    retVal.senderTaskId = TASKID_RASPBERRY_HAT_COM_TASK;

    // Message is meant for Grip Controller
    if (dec.get_address() == address::GRIP_CTRL)
    {
        retVal.recieverTaskId = TASKID_GRIPCONTROLLER_COM_TASK;
        retVal.command = COMMAND_HAND_THROUGH_MESSAGE;
        retVal.data = rawMessage;
    }

    // message is meant for Motion Controller
    if (dec.get_address() == address::MOTION_CTRL)
    {
        if(!dec.verify_crc())
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
            poll_id id = dec.get_params<poll_id>();
            switch (id)
            {
            case (poll_id::DEGREE):
                retVal.command = COMMAND_POLL_DEGREE;
                break;
            case (poll_id::DISTANCE):
                retVal.command = COMMAND_POLL_DISTANCE;
                break;
            case (poll_id::LINE_SENSOR):
                retVal.command = COMMAND_POLL_LINE_SENSOR;
                break;
            default:
                break;
            }

            break;

        default:
            break;
        } // end get_command

        return retVal;
    }
}

/// @brief sends a message frame on the uart bus
/// @param data pointer to encoded frame to send
void RaspberryHatComTask::sendUartMsg(frame *data)
{
    uint64_t rawValue = reinterpret_cast<uint64_t>(data);
    for (int i = 0; i < 8; i++)
    { // 64 Bit = 8 Bytes
        uint8_t byte = (rawValue >> (i * 8)) & 0xFF;
        uart_putc_raw(UART_INSTANCE_RASPBERRYHAT, byte);
    }

    _uartFlushTxWithTimeout(UART_INSTANCE_RASPBERRYHAT, 10);
    return;
}

void RaspberryHatComTask::_uartFlushTxWithTimeout(uart_inst_t *uart, uint32_t timeout_ms)
{
    uint8_t msTicks = 0;
    while (!uart_is_writable(UART_INSTANCE_RASPBERRYHAT))
    {
        msTicks++;
        vTaskDelay(pdMS_TO_TICKS(1));
        if (msTicks > timeout_ms)
        {
            printf("UART TX ERROR - TIMEOUT");
            return;
        }
    }
}

void RaspberryHatComTask::_uartRxIrqHandler()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(_uartRxSemaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

RaspberryHatComTask::~RaspberryHatComTask()
{
}

/* ================================= */
/*              getters              */
/* ================================= */

RaspberryHatComTask RaspberryHatComTask::getInstance(QueueHandle_t *raspberryHatComQueue)
{
    if (_instance == nullptr)
    {
        _instance = new RaspberryHatComTask(raspberryHatComQueue);
    }
    return *_instance;
}

QueueHandle_t RaspberryHatComTask::getQueue()
{
    return QueueHandle_t();
}

TaskHandle_t RaspberryHatComTask::getTaskHandle()
{
    return TaskHandle_t();
}
