#include "LineFollowerTaskTest.hpp"

#include "MotionController.hpp"
#include "TestConfig.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

void LineFollowerTaskTest()
{

#if TEST_FOLLOW_LINE == 1

#endif

#if TEST_TURN == 1

#endif
        // send msg to queue
        // xQueueSend(lineFollowerQueue, &message, portMAX_DELAY);

        vTaskDelay(pdMS_TO_TICKS(5000));
}