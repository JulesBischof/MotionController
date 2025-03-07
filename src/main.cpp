#include "FreeRTOS.h"
#include "task.h"
#include "queues.hpp"

#include "pico/stdlib.h"

#include "MotionControllerInit.hpp"

#include "vMessageDispatcherTask.hpp"
#include "vLineFollowerTask.hpp"

int main()
{
    stdio_init_all();
    MotionControllerInit();
    vInitQueues();

    // -------------- init Tasks ---------------
    // xTaskCreate(vMessageDispatcherTask, "QueueDispatcherTask", 1000, NULL, 1, NULL);
    xTaskCreate(vLineFollowerTask, "LineFollowerTask", 1000, NULL, 1, NULL);

    // ------------ start scheduler --------------
    vTaskStartScheduler();

    // never reached
    while (1)
    {
    };
}