#include "LineFollowerTaskTest.hpp"

#include "LineFollowerTask.hpp"
#include "MessageDispatcherTask.hpp"
#include "RaspberryHatComTask.hpp"
#include "TestConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

#include "queues.h"

void LineFollowerTaskTest()
{
        // create queues
        initGlobalQueues();
        QueueHandle_t messageDispatcherQueue = getMessageDispatcherQueue();
        QueueHandle_t lineFollowerQueue = getLineFollowerQueue();
        QueueHandle_t raspberryHatComQueue = getRaspberryComQueue();

        // create LineFollowerTask instance
        LineFollowerTask lineFollower = LineFollowerTask::getInstance(messageDispatcherQueue, lineFollowerQueue);

        // create RaspberryCom instance
        RaspberryHatComTask raspberryHatComTask = RaspberryHatComTask::getInstance(messageDispatcherQueue, raspberryHatComQueue);

        // create DipatcherTask instance
        MessageDispatcherTask messageDispatcherTask = MessageDispatcherTask::getInstance(messageDispatcherQueue, lineFollowerQueue, raspberryHatComQueue);

        dispatcherMessage_t message;

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
                xQueueSend(lineFollowerQueue, &message, portMAX_DELAY);

                vTaskDelay(pdMS_TO_TICKS(5000));
        }
}