#ifndef MOTIONCONTROLLERPINNIG
#define MOTIONCONTROLLERPINNIG

/* ==============================================================

    Basic Pinning MotionController - refer schematics

============================================================== */

#define Tx_UART0 0
#define Rx_UART0 1

#define Tx_UART1 4
#define Rx_UART1 5

#define DIN_1 2
#define DIN_2 3
#define DIN_3 6
#define DIN_4 7

#define DOUT_1 8
#define DOUT_2 9
#define DOUT_3 10
#define DOUT_4 11

#define GYRO_INT1 12

#define SPI_CS_DRIVER_0 13
#define SPI_CS_DRIVER_1 17
#define SPI_MOSI 19
#define SPI_MISO 16
#define SPI_SCK 18

#define I2C0_SCK 21
#define I2C0_SDA 20

#define HCSR04_TRIGGER 22
#define HCSR04_ECHO 26

#define UV_LED_GPIO 27

#define IREF_R2_DRIVER (14)
#define IREF_R3_DRIVER (15)

#endif
