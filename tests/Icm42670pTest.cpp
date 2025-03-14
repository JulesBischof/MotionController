#include "Icm42670pTest.h"

#include "MotionControllerConfig.h"
#include "Icm42670pConfig.h"

void Icm42670Test(void)
{
    // init device
    Icm42670p gyro = Icm42670p(I2C_INSTANCE_DEVICES, I2C_DEVICE_ICM42670P_ADDRESS);

    float z_angle = 0.0f;
    uint16_t ticks = 0;
    absolute_time_t oldTime = get_absolute_time();

    for (;;)
    {
        absolute_time_t newTime = get_absolute_time();
        int64_t diff_us = absolute_time_diff_us(oldTime, newTime);
        oldTime = newTime;

        volatile int16_t value_raw = gyro.getRawDataZaxis() - STATIC_OFFSET;
        float value_dps = 0;

        if ( (value_raw >= SENSITIVITY_TRESHOLD || value_raw <= -SENSITIVITY_TRESHOLD) )
            value_dps = gyro.ConvertLsbToDps(value_raw);

        z_angle += value_dps * (diff_us / 1e6f); // 1e6 due to unit microseconds!

        if (ticks++ == 10)
        {
            ticks = 0;
            printf("current angle: %f° \n", z_angle);
            // printf("status: %d \n", gyro.getStatus());
            // printf("Temperature: %f °C \n", gyro.getTemperature());
        }

        sleep_ms(POLLING_RATE_MS);
    }
}