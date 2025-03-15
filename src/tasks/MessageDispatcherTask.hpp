#ifndef MESSAGE_DISPATCHER_TASK_H
#define MESSAGE_DISPATCHER_TASK_H

#include "queues.hpp"

#include "FreeRTOS.h"
#include "queue.h"

class MessageDispatcherTask
{
private:
    // singleton
    MessageDispatcherTask(QueueHandle_t *lineFollowerQueue, QueueHandle_t *raspberryComQueue, QueueHandle_t *gripControllerComQueue);
    static MessageDispatcherTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue, _lineFollowerQueue, _raspberryHatComQueue, _gripControllerComQueue;

    static void _run(void* pvParameters);

public:
    ~MessageDispatcherTask();

    // singleton
    static MessageDispatcherTask getInstance(QueueHandle_t *lineFollowerQueue, QueueHandle_t *raspberryComQueue, QueueHandle_t *gripControllerComQueue);
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // MESSAGE_DISPATCHER_TASK_H

static TaskHandle_t _taskHandle;
static QueueHandle_t _dispatcherQueue, _lineFollowerQueue;
static uint32_t _statusFlags;