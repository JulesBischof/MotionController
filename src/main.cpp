#include "FreeRTOS.h"
#include "task.h"
#include "queues.hpp"

#include "pico/stdlib.h"
#include "AppConfig.h"
#include "tests.h"

#include "MotionControllerInit.hpp"

#include "RaspberryHatComTask.hpp"
#include "MessageDispatcherTask.hpp"
#include "LineFollowerTask.hpp"

int main()
{
    stdio_init_all();
    MotionControllerInit();

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 0
    // -------------- init Tasks ---------------

    // first set dispatcher queue
    QueueHandle_t *messageDispatcherQueue = MessageDispatcherTask::initQueue();

    // create task istances
    LineFollowerTask lineFollowerTask = LineFollowerTask::getInstance(messageDispatcherQueue);
    QueueHandle_t lineFollowerQueue = lineFollowerTask.getQueue();

    // last of all -create dispatcher instance
    MessageDispatcherTask messageDispatcherTask = MessageDispatcherTask::getInstance(&lineFollowerQueue);

    // ------------ start scheduler --------------
    vTaskStartScheduler();

    // never reached
    while (1)
    {
    };
#endif
}