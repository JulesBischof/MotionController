#include "Icm42670p.hpp"

/// @brief creates instance of ICM42670p Gyroscope
/// @param i2cInstance i2cInstance ref c/c++ sdk raspberry pico
/// @param i2cAddress i2c address 
Icm42670p::Icm42670p(i2c_inst_t i2cInstance, uint8_t i2cAddress) : I2cBase(i2cInstance, i2cAddress)
{
}

/// @brief deconstructor not implemented yet
Icm42670p::~Icm42670p()
{
}

/// @brief basic settings for Device
void Icm42670p::_initDevice()
{

    /* PWR_MGMT
        - Accelerometer LP mode uses Wake Up oscillator clock
        - the RC oscillator is powered on even if Accel and Gyro are powered off
        - Places gyroscope in Low Noise (LN) Mode
        - Turns accelerometer off
    */
    i2cWriteReg(PWR_MGMT0 | 0xFF, 0x00 | 0b0 << PWR_MGMT0_ACCEL_LP_CLK_SEL_POS | 0b1 << PWR_MGMT0_IDLE_POS | 0b11 << PWR_MGMT0_GYRO_MODE_POS | 0b00 << PWR_MGMT0_ACCEL_MODE_POS);

    /* GYRO_CONFIG0
        +-2000 °/s
        ODR 1.6kHz
    */
    i2cWriteReg(GYRO_CONFIG0 | 0xFF, 0x00 | 0b00 << GYRO_CONFIG0_GYRO_UI_FS_SEL_POS | 0b0101 << GYRO_CONFIG0_GYRO_ODR_POS);

    /* GYRO_CONFIG1
        Gyro low pass Filter bandwith 16Hz
        disable average filter accelerometer
        disable accel low pass filter
    */
    i2cWriteReg(GYRO_CONFIG1 | 0xFF, 0x00 | 0b111 << GYRO_CONFIG1_GYRO_UI_FILT_BW_POS);
}

/// @brief sends example message to device
void Icm42670p::_checkDevice()
{
    uint8_t buffer = 0;
    i2cReadReg(MCLK_RDY | 0xFF, &buffer, 1);
    // TODO validation
}

/// @brief reads raw value of z-Axis Gyro
/// @return Z-Gyro-Data 16bit unsigned! 
uint16_t Icm42670p::_getRawDataZaxis()
{
    uint8_t rawVal[2] = {0};

    i2cReadReg(GYRO_DATA_Z0 | 0xFF, &rawVal[0], 1);
    i2cReadReg(GYRO_DATA_Z1 | 0xFF, &rawVal[1], 1);

    uint16_t retVal = rawVal[1] << 8 | rawVal[0];
}
