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

    hcsr04.setCurrentVelocity(0);

    while (1)
    {
        double buffer = hcsr04.getSensorData();

        volatile float value = static_cast<float>(buffer);
        printf("Sensorvalue: %f ", value);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}