#ifndef MESSAGE_DISPATCHER_TASK_H
#define MESSAGE_DISPATCHER_TASK_H

#include "queues.hpp"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

class RaspberryHatComTask
{
private:
    // singleton
    RaspberryHatComTask(QueueHandle_t *dispatcherQueue);
    static RaspberryHatComTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue, _raspberryHatComQueue;
    static xSemaphoreHandle _uartRxSemaphore;

    static void _run(void* pvParameters);

    static dispatcherMessage_t _getUartMsg();

    static void sendUartMsg(frame *data);
    static void _uartFlushTxWithTimeout(uart_inst_t *uart, uint32_t timeout_ms);

    static void _uartRxIrqHandler();

public:
    ~RaspberryHatComTask();

    // singleton
    static RaspberryHatComTask getInstance(QueueHandle_t *raspberryHatComQueue);
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // MESSAGE_DISPATCHER_TASK_H

static TaskHandle_t _taskHandle;
static QueueHandle_t _dispatcherQueue, _lineFollowerQueue;
static uint32_t _statusFlags;