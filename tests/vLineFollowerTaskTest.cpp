#include "vLineFollowerTaskTest.h"

#include "vLineFollowerTask.hpp"
#include <stdio.h>
#include "queues.hpp"

void vLineFollowerTaskTest()
{
    QueueHandle_t Queue = getLineFollowerTaskQueue();

    xTaskCreate(vLineFollowerTask, "LineFollowerTask", 1000, NULL, 1, NULL);

    while (1)
    {
        // define example Command
        dispatcherMessage_t message;
        message.senderTaskId = DISPATCHER_TASK;
        message.recieverTaskId = LINE_FOLLOWER_TASK;
        message.command = COMMAND_FOLLOW_LINE;
        message.data = 0;

        // send msg to queue
        if (xQueueSend(getLineFollowerTaskQueue(), &message, portMAX_DELAY) != pdPASS)
        {
            printf("Failed to send message to Line Follower Task\n");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));

        // define example Command
        message.senderTaskId = DISPATCHER_TASK;
        message.recieverTaskId = LINE_FOLLOWER_TASK;
        message.command = COMMAND_STOP;
        message.data = 0;

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}