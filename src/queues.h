#ifndef QUEUES_H
#define QUEUES_H

#include "FreeRTOS.h"
#include "queue.h"

/* -------------------------------------------------- */
/*               typedefs global queues               */
/* -------------------------------------------------- */

typedef enum dispatcherTaskId_t
{
    TASKID_DEFAULT_NOTASK,
    TASKID_DISPATCHER_TASK,
    TASKID_LINE_FOLLOWER_TASK,
    TASKID_RASPBERRY_HAT_COM_TASK,
    TASKID_GRIPCONTROLLER_COM_TASK
}
dispatcherTaskId_t;

typedef enum taskCommand_t
{
    COMMAND_MOVE,
    COMMAND_REVERSE,
    COMMAND_TURN,
    COMMAND_STOP,
    COMMAND_INFO,
    COMMAND_PING,
    COMMAND_PONG,
    COMMAND_ERROR,
    COMMAND_POLL_DISTANCE,
    COMMAND_POLL_LINE_SENSOR,
    COMMAND_POLL_DEGREE,
    COMMAND_POLL_STATUSFLAGS,
    COMMAND_HAND_THROUGH_MESSAGE,
    COMMAND_DECODE_MESSAGE
} taskCommand_t;

typedef struct dispatcherMessage_t
{
    dispatcherTaskId_t senderTaskId;
    dispatcherTaskId_t recieverTaskId;
    taskCommand_t command;
    uint64_t data;
} dispatcherMessage_t;

/* -------------------------------------------------- */
/*                 define global queues               */
/* -------------------------------------------------- */
static QueueHandle_t raspberryComQueueHandle = NULL;
static QueueHandle_t lineFollowerTaskQueueHandle = NULL;
static QueueHandle_t messageDispatcherTaskQueueHandle = NULL;

/* -------------------------------------------------- */
/*                  queue getters                     */
/* -------------------------------------------------- */
QueueHandle_t getRaspberryComQueue();
QueueHandle_t getLineFollowerQueue();
QueueHandle_t getMessageDispatcherQueue();

/* -------------------------------------------------- */
/*             declaration global queue Fn            */
/* -------------------------------------------------- */

void initGlobalQueues();
dispatcherMessage_t generateResponse(dispatcherTaskId_t senderTaskId, dispatcherTaskId_t recieverTaskId, taskCommand_t command, uint32_t data);

#endif