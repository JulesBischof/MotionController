#include "RaspberryHatComTask.hpp"

#include <stdio.h>

#include <pico/stdlib.h>
#include <hardware/uart.h>

#include "MotionControllerConfig.h"

#include "prain_uart/encoder.hpp"
#include "prain_uart/decoder.hpp"
#include "prain_uart/helper.hpp"
#include "prain_uart/protocol.hpp"

using namespace prain_uart;
using namespace encoder;

RaspberryHatComTask::RaspberryHatComTask(QueueHandle_t *dispatcherQueue)
{
}

void RaspberryHatComTask::_run(void *pvParameters)
{
    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(_raspberryHatComQueue) > 0)
        {
            dispatcherMessage_t message;
            xQueueReceive(_raspberryHatComQueue, &message, pdMS_TO_TICKS(10));

            if (message.recieverTaskId != TASKID_RASPBERRY_HAT_COM_TASK)
            {
                printf("RASPBERRYHATCOMTASK - Message contains wrong Task ID \n");
                continue;
            }

            // check incoming commands - messages supposed to go out
            switch (message.command)
            {
            case (COMMAND_INFO):
                frame txMsg = encode_info(address::RASPBERRY_HAT, message.data);
                sendUartMsg(&txMsg);
                break;
            case (COMMAND_ERROR):
                break;
            case (COMMAND_POLL_DISTANCE):
                break;
            case (COMMAND_POLL_LINE_POSITION):
                break;
            case (COMMAND_POLL_ANGLE):
                break;
            case (COMMAND_POLL_STATUSFLAGS):
                break;

            default:
                break;
            }
        }
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

    uart_flush_with_timeout(UART_INSTANCE_RASPBERRYHAT, 10);
    return;
}

void uart_flush_with_timeout(uart_inst_t *uart, uint32_t timeout_ms)
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
