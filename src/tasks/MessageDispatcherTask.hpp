#ifndef MESSAGE_DISPATCHER_TASK_H
#define MESSAGE_DISPATCHER_TASK_H

#include "queues.h"

#include "FreeRTOS.h"
#include "queue.h"

class MessageDispatcherTask
{
private:
    // singleton
    MessageDispatcherTask(QueueHandle_t messageDispatcherQueue, QueueHandle_t lineFollowerQueue, QueueHandle_t raspberryComQueueHandle);
    static MessageDispatcherTask *_instance;
    static TaskHandle_t _taskHandle;
    static QueueHandle_t _messageDispatcherQueue, _lineFollowerQueue, _raspberryHatComQueue;
    static uint16_t _statusFlags;

    static void _run(void *pvParameters);

public:
    ~MessageDispatcherTask();

    // singleton
    static MessageDispatcherTask getInstance(QueueHandle_t messageDispatcherQueue, QueueHandle_t lineFollowerQueue, QueueHandle_t raspberryComQueueHandle);
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // MESSAGE_DISPATCHER_TASK_H