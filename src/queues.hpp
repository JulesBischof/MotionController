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
    TASKID_GETSENSORDATA_TASK,
    TASKID_GRIPCONTROLLER_COM_TASK
}
dispatcherTaskId_t;

typedef enum taskCommand_t
{
    COMMAND_FOLLOW_LINE,
    COMMAND_STOP,
    COMMAND_TURN,
    COMMAND_GET_DISTANCE,
    COMMAND_GET_LINE_POSITION,
    COMMAND_GET_ANGLE,
    COMMAND_GET_STATUSFLAGS,
    COMMAND_SEND_WARNING,
    COMMAND_SEND_ERROR,
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