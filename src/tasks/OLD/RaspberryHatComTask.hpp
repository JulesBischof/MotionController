#ifndef RASPBERRYHATCOM_TASK_H
#define RASPBERRYHATCOM_TASK_H

#include "queues.h"

#include "prain_uart/protocol.hpp"

#include <hardware/uart.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

using namespace prain_uart;

class RaspberryHatComTask
{
private:
    // singleton
    RaspberryHatComTask(QueueHandle_t messageDispatcherQueue, QueueHandle_t raspberryHatComQueue);
    static RaspberryHatComTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _messageDispatcherQueue, _raspberryHatComQueue;
    static xSemaphoreHandle _uartRxSemaphore;

    static void _run(void* pvParameters);

    static dispatcherMessage_t _getCommand();

    static void sendUartMsg(frame *data);
    static void _uartFlushTxWithTimeout(uart_inst_t *uart, uint32_t timeout_ms);

    static void _uartRxIrqHandler();

public:
    ~RaspberryHatComTask();

    // singleton
    static RaspberryHatComTask getInstance(QueueHandle_t messageDispatcherQueue, QueueHandle_t raspberryHatComQueue);
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // RASPBERRYHATCOM_TASK_H

static TaskHandle_t _taskHandle;
static QueueHandle_t _dispatcherQueue, _lineFollowerQueue;
static uint32_t _statusFlags;