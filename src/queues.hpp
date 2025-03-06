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
    MOTOR_TASK,
    COMM0_TASK,
} dispatcherTaskId_t;

typedef struct dispatcherMessage_t
{
    dispatcherTaskId_t taskId;
    char command [2];
    uint32_t data;
} dispatcherMessage_t;

/* -------------------------------------------------- */
/*             declaration global queues              */
/* -------------------------------------------------- */

extern QueueHandle_t xDispatcherQueue;
extern QueueHandle_t xMotorTaskQueue;
extern QueueHandle_t xCom0TaskQueue;

// init global queues
void vInitQueues(void);

// getters
QueueHandle_t getDispatcherQueue(void);
QueueHandle_t getMotorTaskQueue(void);
QueueHandle_t getCom0TaskQueue(void);

#endif