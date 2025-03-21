#include "FreeRTOS.h"
#include "task.h"
#include "queues.h"

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

#if APP_MODE == 1
    // -------------- init Tasks ---------------

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

    // ------------ start scheduler --------------
    vTaskStartScheduler();

    // never reached
    while (1)
    {
    };
#endif
}