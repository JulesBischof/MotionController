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
    MtnCtrl::MotionController motionController = MtnCtrl::MotionController();

    QueueHandle_t comQueue = gripController.getGripControllerRxQueue();
    QueueHandle_t dispatcher = motionController.getMessageDispatcherQueue();

    motionController.registerGripControllerInstance(comQueue);
    gripController.registerTxQueue(dispatcher);

    gripController.startTasks();
    motionController.startTasks();

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
