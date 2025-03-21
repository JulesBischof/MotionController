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

        DispatcherMessage message = {};
        while (1)
        {
                // define example Command
                message.senderTaskId = DispatcherTaskId::RaspberryHatComTask;
                message.receiverTaskId = DispatcherTaskId::LineFollowerTask;

#if TEST_FOLLOW_LINE == 1
                message.command = TaskCommand::Move;
                message.data = 0;
#endif

#if TEST_TURN == 1
                message.command = TaskCommand::Turn;
                message.data = 1800; // 180° * 10
#endif
                // send msg to queue
                // xQueueSend(lineFollowerQueue, &message, portMAX_DELAY);

                vTaskDelay(pdMS_TO_TICKS(5000));
        }
}