#ifndef QUEUES_H
#define QUEUES_H

#include "FreeRTOS.h"
#include "queue.h"

/* -------------------------------------------------- */
/*               typedefs global queues               */
/* -------------------------------------------------- */

typedef enum dispatcherTaskId_t
{
    TASKID_DISPATCHER_TASK,
    TASKID_LINE_FOLLOWER_TASK,
    TASKID_RASPBERRY_HAT_COM_TASK,
    TASKID_GRIPCONTROLLER_COM_TASK
}
dispatcherTaskId_t;

typedef enum taskCommand_t
{
    COMMAND_MOVE,
    COMMAND_TURN,
    COMMAND_STOP,
    COMMAND_INFO,
    COMMAND_PING,
    COMMAND_PONG,
    COMMAND_ERROR,
    COMMAND_POLL_DISTANCE,
    COMMAND_POLL_LINE_POSITION,
    COMMAND_POLL_ANGLE,
    COMMAND_POLL_STATUSFLAGS,
} taskCommand_t;

typedef struct dispatcherMessage_t
{
    dispatcherTaskId_t senderTaskId;
    dispatcherTaskId_t recieverTaskId;
    taskCommand_t command;
    uint32_t data;
} dispatcherMessage_t;

/* -------------------------------------------------- */
/*             declaration global queue Fn            */
/* -------------------------------------------------- */

dispatcherMessage_t generateResponse(dispatcherTaskId_t senderTaskId, dispatcherTaskId_t recieverTaskId, taskCommand_t command, uint32_t data);

#endif