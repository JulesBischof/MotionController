#include "LineFollowerTaskTest.h"

#include "LineFollowerTask.hpp"
#include <stdio.h>

#include "queues.hpp"

void vLineFollowerTaskTest()
{
    // create queues
    QueueHandle_t dispatcherQueue = xQueueCreate(100, sizeof(dispatcherMessage_t));

    // create LineFollowerTask instance
    LineFollowerTask lineFollower = LineFollowerTask::getInstance(&dispatcherQueue);

    while (1)
    {
        // define example Command
        dispatcherMessage_t message;
        message.senderTaskId = TASKID_DISPATCHER_TASK;
        message.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;
        message.command = COMMAND_MOVE;
        message.data = 0;

        // send msg to queue
        if (xQueueSend(lineFollower.getQueue(), &message, portMAX_DELAY) != pdPASS)
        {
            printf("Failed to send message to Line Follower Task\n");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}