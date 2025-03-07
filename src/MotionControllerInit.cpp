#include "MotionControllerInit.hpp"

void MotionControllerInit(void)
{
    initUart();
    initSpi();
    initI2c();

    return;
}

void initUart(void)
{
    gpio_set_function(Tx_UART0, GPIO_FUNC_UART);
    gpio_set_function(Rx_UART0, GPIO_FUNC_UART);
    uart_init(UART_INSTANCE_RASPBERRYHAT, UART0_BAUDRATE);

    gpio_set_function(Tx_UART1, GPIO_FUNC_UART);
    gpio_set_function(Rx_UART1, GPIO_FUNC_UART);
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
    gpio_put(SPI_CS_DRIVER_0, true); // pull up CS
    gpio_init(SPI_CS_DRIVER_1);

    gpio_set_dir(SPI_CS_DRIVER_1, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_1, true); // pull up CS

    spi_set_format(TMC5240_SPI_INSTANCE, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    return;
}

void initI2c(void)
{
    i2c_init(I2C_INSTANCE_DEVICES, I2C_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(I2C0_SCK, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);

    return;
}
