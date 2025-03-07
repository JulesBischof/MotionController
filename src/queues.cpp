#include "queues.hpp"

// definition of global queues
QueueHandle_t xDispatcherQueue = NULL;
QueueHandle_t xLineFollowerTaskQueue = NULL;
QueueHandle_t xCom0TaskQueue = NULL;

// initialisation of global queues
void vInitQueues(void)
{
    xDispatcherQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xLineFollowerTaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xCom0TaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
}

QueueHandle_t getDispatcherTaskQueue(void)
{
    return xDispatcherQueue;
}

QueueHandle_t getLineFollowerTaskQueue(void)
{
    return xLineFollowerTaskQueue;
}

QueueHandle_t getCom0TaskQueue(void)
{
    return xCom0TaskQueue;
}
