#include "HcSr04Test.hpp"

#include "HcSr04.hpp"
#include "MotionControllerPinning.hpp"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include <stdio.h>

void HcSr04Test(void)
{
    miscDevices::HcSr04 hcSr04 = miscDevices::HcSr04(HCSR04_TRIGGER, HCSR04_ECHO);
    hcSr04.initMeasurmentTask();

    while (1)
    {
        float buffer = hcSr04.getSensorData();
        hcSr04.triggerNewMeasurment();

        printf("%f\n", buffer);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}