#ifndef QUEUES_H
#define QUEUES_H

#include "FreeRTOS.h"
#include "queue.h"

/* -------------------------------------------------- */
/*               typedefs global queues               */
/* -------------------------------------------------- */

typedef enum dispatcherTaskId_t
{
    DISPATCHER_TASK,
    LINE_FOLLOWER_TASK,
} dispatcherTaskId_t;

typedef enum taskCommand_t
{
    COMMAND_FOLLOW_LINE,
    COMMAND_STOP,
    COMMAND_TURN,
    COMMAND_GET_DISTANCE,
    COMMAND_GET_LINE_POSITION,
    GET_ANGLE,
    COMMAND_GET_STATUSFLAGS,
    SEND_ERROR,
} taskCommand_t;

typedef struct dispatcherMessage_t
{
    dispatcherTaskId_t senderTaskId;
    dispatcherTaskId_t recieverTaskId;
    taskCommand_t command;
    uint32_t data;
} dispatcherMessage_t;

/* -------------------------------------------------- */
/*             declaration global queues              */
/* -------------------------------------------------- */

extern QueueHandle_t xDispatcherQueue;
extern QueueHandle_t xLineFollowerTaskQueue;
extern QueueHandle_t xCom0TaskQueue;

// init global queues
void vInitQueues(void);

// getters
QueueHandle_t getDispatcherTaskQueue(void);
QueueHandle_t getLineFollowerTaskQueue(void);
QueueHandle_t getCom0TaskQueue(void);

#endif