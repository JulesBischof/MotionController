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

#include "queues.h"

using namespace prain_uart;
using namespace encoder;

/* ================================= */
/*         static Members            */
/* ================================= */
RaspberryHatComTask *RaspberryHatComTask::_instance = nullptr;

TaskHandle_t RaspberryHatComTask::_taskHandle = nullptr;
QueueHandle_t RaspberryHatComTask::_messageDispatcherQueue = nullptr;
QueueHandle_t RaspberryHatComTask::_raspberryHatComQueue = nullptr;

SemaphoreHandle_t RaspberryHatComTask::_uartRxSemaphore = nullptr;
/* ================================= */
/*         implementation            */
/* ================================= */

RaspberryHatComTask::RaspberryHatComTask(QueueHandle_t messageDispatcherQueue, QueueHandle_t raspberryHatComQueue)
{
    int uartIRQId = UART_INSTANCE_RASPBERRYHAT == (uart0) ? UART0_IRQ : UART1_IRQ;

    /*  ---- enable uart interrupt on Rx events ---- */

    // Set UART flow control CTS/RTS, we don't want these, so turn them off ### sure???
    // uart_set_hw_flow(UART_INSTANCE_RASPBERRYHAT, false, false);
    
    // Set our data format
    // uart_set_format(UART_INSTANCE_RASPBERRYHAT, 8, 1, UART_PARITY_NONE);

    // Turn on FIFO's 
    uart_set_fifo_enabled(UART_INSTANCE_RASPBERRYHAT, true);
    // set up and enable isr
    irq_set_exclusive_handler(uartIRQId, _uartRxIrqHandler);
    irq_set_enabled(uartIRQId, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);

    // init static members
    _messageDispatcherQueue = messageDispatcherQueue;
    _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));
    _uartRxSemaphore = xSemaphoreCreateBinary();

    if (xTaskCreate(_run, RASPBERRYHATCOMTASK_NAME, RASPBERRYHATCOMTASK_STACKSIZE / sizeof(StackType_t), NULL, RASPBERRYHATCOMTASK_PRIORITY, &_taskHandle) != pdTRUE)
    {
        for (;;)
            ;
        /* ERROR Todo: error handling */
    }
}

RaspberryHatComTask::~RaspberryHatComTask()
{
    if (_instance != nullptr)
    {
        delete _instance;
        _instance = nullptr;
    }
}

void RaspberryHatComTask::_run(void *pvParameters)
{
    // loop forever
    for (;;)
    {
        dispatcherMessage_t message;
        if (xQueueReceive(_raspberryHatComQueue, &message, pdMS_TO_TICKS(100)) == pdTRUE)
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
                msg = _getCommand();

                xSemaphoreTake(_uartRxSemaphore, 0);
                xQueueSend(_messageDispatcherQueue, &msg, 10);
                break;
            default:
                break;
            }
            sendUartMsg(&txMsg);
        }
    }
} // end _run

dispatcherMessage_t RaspberryHatComTask::_getCommand()
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

    // TODO: does the messega get into read buffer MSB or LSB first?

    volatile uint64_t rawMessage = 0;

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
    return;
}

void RaspberryHatComTask::_uartRxIrqHandler()
{
    /* some message arrived!! send read command to RaspberryHatComQueue
    buffer-read and decoding shouldn't be handled inside a isr */

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (uart_is_readable(UART_INSTANCE_RASPBERRYHAT) && !uxSemaphoreGetCountFromISR(_uartRxSemaphore))
    {
        // semaphore makes sure, irq sends decode command only once per message
        xSemaphoreGiveFromISR(_uartRxSemaphore, &xHigherPriorityTaskWoken);

        dispatcherMessage_t msg;
        msg.command = COMMAND_DECODE_MESSAGE;
        msg.recieverTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
        msg.senderTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
        msg.data = 0;

        xQueueSendFromISR(_raspberryHatComQueue, &msg, &xHigherPriorityTaskWoken);
    }

    int uartIRQId = UART_INSTANCE_RASPBERRYHAT == (uart0) ? UART0_IRQ : UART1_IRQ;

    // disable uart interrupts
    uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, false, false);

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ================================= */
/*              getters              */
/* ================================= */

RaspberryHatComTask RaspberryHatComTask::getInstance(QueueHandle_t messageDispatcherQueue, QueueHandle_t raspberryHatComQueue)
{
    if (_instance == nullptr)
    {
        _instance = new RaspberryHatComTask(messageDispatcherQueue, raspberryHatComQueue);
    }
    return *_instance;
}

QueueHandle_t RaspberryHatComTask::getQueue()
{
    return _raspberryHatComQueue;
}

TaskHandle_t RaspberryHatComTask::getTaskHandle()
{
    return _taskHandle;
}
