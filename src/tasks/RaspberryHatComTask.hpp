#ifndef MESSAGE_DISPATCHER_TASK_H
#define MESSAGE_DISPATCHER_TASK_H

#include "queues.hpp"

#include "FreeRTOS.h"
#include "queue.h"

class RaspberryHatComTask
{
private:
    // singleton
    RaspberryHatComTask(QueueHandle_t *dispatcherQueue);
    static RaspberryHatComTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue, _raspberryHatComQueue;

    static void _run(void* pvParameters);

    static void sendUartMsg(frame *data);

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