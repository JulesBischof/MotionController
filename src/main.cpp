#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"

#include "MotionControllerPinning.h"

// void led_task()
// {
//     const uint LED_PIN = PICO_DEFAULT_LED_PIN;
//     gpio_init(LED_PIN);
//     gpio_set_dir(LED_PIN, GPIO_OUT);
//     while (true)
//     {
//         gpio_put(LED_PIN, 1);
//         vTaskDelay(100);
//         gpio_put(LED_PIN, 0);
//         vTaskDelay(100);
//     }
// }

int main()
{
    stdio_init_all();

    // -------------   initialize UART  -------------

    // ------------- initialize SPI Bus -------------

    // ------------- initialize I2C Bus -------------
    i2c_init(i2c0, 400e3); // set Baudrate 400kHz
    gpio_set_function(I2C0_SCK, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);




    
    // xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1)
    {
    };
}