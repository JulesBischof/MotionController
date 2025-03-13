#ifndef MOTIONCONTROLLERCONFIG_H
#define MOTIONCONTROLLERCONFIG_H

/* ==============================================================

                Basic Settings Com - Channels

============================================================== */

/* -------------------- UART --------------------- */
#define UART0_BAUDRATE (115200)
#define UART1_BAUDRATE (115200)

/* -------------------- SPI ---------------------- */
#define SPI_BAUDRATE_KHZ (1000)

/* -------------------- I2C ----------------------- */
#define I2C_BAUDRATE_KHZ (100)

#define I2C_DEVICE_TLA2528_ADDRESS (0x14)
#define I2C_DEVICE_ICM42670P_ADDRESS (0x68)

#define ACTIVATE_INTERNAL_PULLUP (1)

/* ------------------ Channels -------------------- */
#define UART_INSTANCE_RASPBERRYHAT (uart0)
#define UART_INSTANCE_GRIPCONTROLLER (uart1)
#define I2C_INSTANCE_DEVICES (i2c0)
#define TMC5240_SPI_INSTANCE (spi0)

#endif