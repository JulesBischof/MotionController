#include "MotionControllerInit.hpp"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"

void MotionControllerInit(void)
{
    initUart();
    initSpi();
    initI2c();

    return;
}

void initUart(void)
{
    gpio_set_function(Tx_UART0, UART_FUNCSEL_NUM(UART_INSTANCE_RASPBERRYHAT, Tx_UART0));
    gpio_set_function(Rx_UART0, UART_FUNCSEL_NUM(UART_INSTANCE_RASPBERRYHAT, Rx_UART0));
    uart_init(uart0, UART0_BAUDRATE);

    gpio_set_function(Tx_UART1, UART_FUNCSEL_NUM(UART_INSTANCE_GRIPCONTROLLER, Tx_UART1));
    gpio_set_function(Rx_UART1, UART_FUNCSEL_NUM(UART_INSTANCE_GRIPCONTROLLER, Rx_UART1));
    uart_init(UART_INSTANCE_GRIPCONTROLLER, UART1_BAUDRATE);

    return;
}

void initSpi(void)
{
    spi_init(TMC5240_SPI_INSTANCE, SPI_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(SPI_CS_DRIVER_0);
    gpio_set_dir(SPI_CS_DRIVER_0, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_0, 1); // pull up CS

    gpio_init(SPI_CS_DRIVER_1);
    gpio_set_dir(SPI_CS_DRIVER_1, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_1, 1); // pull up CS

    spi_set_format(TMC5240_SPI_INSTANCE, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    return;
}

void initI2c(void)
{
    i2c_init(I2C_INSTANCE_DEVICES, I2C_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(I2C0_SCK, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);

    #if ACTIVATE_INTERNAL_PULLUP == 1
        gpio_pull_up(I2C0_SCK);
        gpio_pull_up(I2C0_SDA);
    #endif

    return;
}
