#include "queues.h"

QueueHandle_t getRaspberryComQueue()
{
    return raspberryComQueueHandle;
}

QueueHandle_t getLineFollowerQueue()
{
    return lineFollowerTaskQueueHandle;
}

QueueHandle_t getMessageDispatcherQueue()
{
    return messageDispatcherTaskQueueHandle;
}

void initGlobalQueues()
{
    raspberryComQueueHandle = xQueueCreate(1000, sizeof(dispatcherMessage_t));
    lineFollowerTaskQueueHandle = xQueueCreate(1000, sizeof(dispatcherMessage_t));
    messageDispatcherTaskQueueHandle = xQueueCreate(1000, sizeof(dispatcherMessage_t));
}

/// @brief creates a dispatcherMessage_t struct as message for inter task communication
/// @param senderTaskId Sender task
/// @param recieverTaskId RecieverTask
/// @param command command wich is meant to send
/// @param data parameter for the command
/// @return
dispatcherMessage_t generateResponse(dispatcherTaskId_t senderTaskId, dispatcherTaskId_t recieverTaskId, taskCommand_t command, uint32_t data)
{
    dispatcherMessage_t response;
    response.senderTaskId = senderTaskId;
    response.recieverTaskId = recieverTaskId;
    response.command = command;
    response.data = data;

    return response;
}
