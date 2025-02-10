#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/i2c.h"
#include "hardware/spi.h"

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
    spi_init(spi0, SPI_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(SPI_CS_DRIVER_0);
    gpio_set_dir(SPI_CS_DRIVER_0, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_0, true); // pull up CS
    gpio_init(SPI_CS_DRIVER_1);

    gpio_set_dir(SPI_CS_DRIVER_1, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_1, true); // pull up CS

    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    // ------------- initialize I2C Bus -------------
    i2c_init(i2c0, I2C_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(I2C0_SCK, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);




    
    // xTaskCreate(led_task, "LED_Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();

    while (1)
    {
    };
}