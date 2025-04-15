#include "Tla2528Test.h"

#include <stdio.h>
#include "pico/stdlib.h"

#include "MotionControllerConfig.h"

#include "Tla2528.hpp"



void Tla2528Test(void)
{
    i2cDevices::Tla2528 adc = i2cDevices::Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);

    for (;;)
    {
        uint16_t analogValues[8] = {0};

        adc.readAdc(analogValues);

        for (int i = 0; i < (sizeof(analogValues) / sizeof(uint16_t)); i++)
        {
            printf("A%d = [ %d ]", i, analogValues[i]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}