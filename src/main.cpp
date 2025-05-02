#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "AppConfig.hpp"
#include "tests.hpp"

#include "MotionController.hpp"
#include "GripController.hpp"

int main()
{

    stdio_init_all();
    services::LoggerService::setLogLevel(APPCONFIG_LOGLEVEL);

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 1

    GripControllerBoard::GripController gripController = GripControllerBoard::GripController();
    QueueHandle_t comQueue = gripController.getMotionControllerComQueue();
    gripController.startTasks();

    MtnCtrl::MotionController motionController = MtnCtrl::MotionController();
    motionController.startTasks();
    //motionController.registerComQueue(comQueue);

    services::LoggerService::info("main", "init done - gonna start scheduler now! ");
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
