#ifndef MESSAGE_DISPATCHER_TASK_H
#define MESSAGE_DISPATCHER_TASK_H

#include "queues.hpp"

#include "FreeRTOS.h"
#include "queue.h"

class MessageDispatcherTask
{
private:
    // singleton
    MessageDispatcherTask(QueueHandle_t *lineFollowerQueue);
    static MessageDispatcherTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue, _lineFollowerQueue, _raspberryHatComQueue;
    static uint16_t _statusFlags;

    static void _run(void *pvParameters);

public:
    ~MessageDispatcherTask();

    // singleton
    static MessageDispatcherTask getInstance(QueueHandle_t *lineFollowerQueue);
    static QueueHandle_t initQueue();
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // MESSAGE_DISPATCHER_TASK_H