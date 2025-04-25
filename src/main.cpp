#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "AppConfig.hpp"
#include "tests.hpp"

#include "MotionController.hpp"

int main()
{
    stdio_init_all();
    services::LoggerService::setLogLevel(APPCONFIG_LOGLEVEL);

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 1

    MtnCtrl::MotionController motionController = MtnCtrl::MotionController();
    motionController.startTasks();

    vTaskStartScheduler();
    for (;;)
    {
        /* never reached */
    }

#endif
}

// -------- Hooks
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    services::LoggerService::fatal("vApplicationStackOverflowHook","stack overflow in Task: %s\n", pcTaskName);
    for (;;)
        ;
}
