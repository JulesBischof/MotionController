#include "queues.hpp"

// initialisation of global queues
void vInitQueues(void)
{
    xDispatcherQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xMotorTaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
    xCom0TaskQueue = xQueueCreate(10, sizeof(dispatcherMessage_t));
}

QueueHandle_t getDispatcherQueue(void)
{
    return xDispatcherQueue;
}

QueueHandle_t getMotorTaskQueue(void)
{
    return xMotorTaskQueue;
}

QueueHandle_t getCom0TaskQueue(void)
{
    return xCom0TaskQueue;
}
