#include "LineFollowerTaskTest.hpp"

#include "LineFollowerTask.hpp"
#include "MessageDispatcherTask.hpp"
#include "TestConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

#include "queues.hpp"

void LineFollowerTaskTest()
{
    // create queues
    QueueHandle_t *dispatcherQueue = MessageDispatcherTask::initQueue();

    // create LineFollowerTask instance
    LineFollowerTask lineFollower = LineFollowerTask::getInstance(dispatcherQueue);
    QueueHandle_t lineFollowerQueue = lineFollower.getQueue();
    
    // create DipatcherTask instance
    MessageDispatcherTask messageDispatcherTask = MessageDispatcherTask::getInstance(&lineFollowerQueue);

    dispatcherMessage_t message;
    QueueHandle_t testRecieverQueue = nullptr;

#if TEST_SENDMESSAGE_VIA_DISPATCHER == 1
    testRecieverQueue = *dispatcherQueue;
#else
    testRecieverQueue = lineFollowerQueue;
#endif

    while (1)
    {
        // define example Command
        message.senderTaskId = TASKID_RASPBERRY_HAT_COM_TASK;
        message.recieverTaskId = TASKID_LINE_FOLLOWER_TASK;

#if TEST_FOLLOW_LINE == 1
        message.command = COMMAND_MOVE;
        message.data = 0;
#endif

#if TEST_TURN == 1
        message.command = COMMAND_TURN;
        message.data = 1800; // 180° * 10
#endif
        // send msg to queue
        if (xQueueSend(testRecieverQueue, &message, portMAX_DELAY) != pdPASS)
        {
            printf("Failed to send message to Line Follower Task\n");
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}