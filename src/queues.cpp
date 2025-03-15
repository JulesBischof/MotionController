#include "queues.hpp"

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
