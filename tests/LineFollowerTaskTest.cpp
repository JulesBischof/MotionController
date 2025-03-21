#include "LineFollowerTaskTest.hpp"

#include "MotionController.hpp"
#include "TestConfig.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

#include "queues.h"

void LineFollowerTaskTest()
{
        MotionController motionController = MotionController();
        motionController.startScheduler();

        dispatcherMessage_t message = {};
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
                // xQueueSend(lineFollowerQueue, &message, portMAX_DELAY);

                vTaskDelay(pdMS_TO_TICKS(5000));
        }
}