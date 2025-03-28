#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "AppConfig.h"
#include "tests.h"

#include "MotionController.hpp"

int main()
{
    stdio_init_all();

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 1

    MotionController::MotionController motionController = MotionController::MotionController();
    motionController.startScheduler();

    // never reached
    while (1)
    {
    };
#endif
}