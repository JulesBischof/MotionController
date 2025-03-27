#include "HcSr04Test.h"

#include "HcSr04.hpp"
#include "MotionControllerPinning.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

void HcSr04Test(void)
{
    HcSr04 hcsr04 = HcSr04(HCSR04_TRIGGER, HCSR04_ECHO);

    TaskHandle_t sensorTask = hcsr04.getTaskHandle();
    QueueHandle_t queueHandle = hcsr04.getQueueHandle();
    hcsr04.setCurrentVelocity(0);

    while (1)
    {
        double buffer;
        xQueuePeek(queueHandle, &buffer, portMAX_DELAY);

        volatile float value = static_cast<float>(buffer);
        printf("Sensorvalue: %f ", value);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}