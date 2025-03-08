#include "Icm42670p.hpp"

/// @brief creates instance of ICM42670p Gyroscope
/// @param i2cInstance i2cInstance ref c/c++ sdk raspberry pico
/// @param i2cAddress i2c address
Icm42670p::Icm42670p(i2c_inst_t *i2cInstance, uint8_t i2cAddress) : I2cBase(i2cInstance, i2cAddress)
{
    _initDevice();
    _checkDevice();
}

/// @brief deconstructor not implemented yet
Icm42670p::~Icm42670p()
{
}

/// @brief returns current angular change in °/s
/// @return signed int - angular change in °/s
float Icm42670p::ConvertLsbToDps(int16_t rawVal)
{
    float val = (float)rawVal / LSB_TO_DPS_SCALE_FACTOR;
    return val;
}



/// @brief calculates absolut angle in degree, given by how many steps both drivers did 
/// @param xActualDriver0 xActual Value driver 0
/// @param xActualDriver1 xActual Value driver 1
/// @return angle in °
float Icm42670p::getAngle(int32_t xActualDriver0, int32_t xActualDriver1)
{
    float retval = (xActualDriver0 / xActualDriver1) * (180 / 3.14);
    return 0.0f;
}

/// @brief basic settings for Device
void Icm42670p::_initDevice()
{
    uint8_t address = 0;

    address = (uint8_t)(PWR_MGMT0 & 0xFF);
    printf("init PWR_MGMT0 register (addr %Xh) value: (%Xh)\n", address, PWR_MGMT0_INITVALUE);
    if (!i2cWriteReg(address, PWR_MGMT0_INITVALUE))
        printf("Failed to write PWR_MGMT0 register\n");

    address = (uint8_t)(GYRO_CONFIG0 & 0xFF);
    printf("init GYRO_CONFIG0 register (addr %Xh) value: (%Xh)\n", address, CONFIG0_INITVALUE);
    if (!i2cWriteReg(address, CONFIG0_INITVALUE))
        printf("Failed to write GYRO_CONFIG0 register\n");

    address = (uint8_t)(GYRO_CONFIG1 & 0xFF);
    printf("init GYRO_CONFIG1 register (addr %Xh) value: (%Xh)\n", address, CONFIG1_INITVALUE);
    if (!i2cWriteReg(address, CONFIG1_INITVALUE))
        printf("Failed to write GYRO_CONFIG1 register\n");

    printf("Icm4670p init done! \n");
}

/// @brief sends example message to device
void Icm42670p::_checkDevice()
{
    printf("check Device ... Read out initialization values ... \n");
    uint8_t address = 0;
    uint8_t buffer[3] = {0};

    address = (uint8_t)(PWR_MGMT0);
    i2cReadReg(address, &buffer[0], 1);

    address = (uint8_t)(GYRO_CONFIG0);
    i2cReadReg(address, &buffer[1], 1);

    address = (uint8_t)(GYRO_CONFIG1);
    i2cReadReg(address, &buffer[2], 1);

    printf("Initialization Register values: \n - PWR_MGMT0: %Xh \n - GYRO_CONFIG0: %Xh \n - GYRO_CONFIG1: %Xh \n", buffer[0], buffer[1], buffer[2]);
    printf("Icm4670p check done! \n");

    return;
}

/// @brief reads raw value of z-Axis Gyro
/// @return Z-Gyro-Data 16bit unsigned!
int16_t Icm42670p::getRawDataZaxis()
{
    uint8_t rawVal[2] = {0};
    uint8_t address_0 = (uint8_t)(GYRO_DATA_Z0 & 0xFF);
    uint8_t address_1 = (uint8_t)(GYRO_DATA_Z1 & 0xFF);

    i2cReadReg(address_0, &rawVal[0], 1);
    i2cReadReg(address_1, &rawVal[1], 1);

    int16_t retVal = 0;

    if (this->_i2cStatus == I2C_STATUSOK)
        retVal = (int16_t)(rawVal[1] << 8 | rawVal[0]);

    return retVal;
}

float Icm42670p::getTemperature()
{
    uint8_t temp_data[2] = {0};
    uint8_t address_0 = (uint8_t)(TEMP_DATA0 & 0xFF);
    uint8_t address_1 = (uint8_t)(TEMP_DATA1 & 0xFF);

    i2cReadReg(address_1, &temp_data[1], 1);
    i2cReadReg(address_0, &temp_data[0], 1);

    int16_t rawVal = (int16_t)(temp_data[1] << 8 | temp_data[0]);
    float temp_celsius = 25;

    if (this->_i2cStatus == I2C_STATUSOK)
        temp_celsius = (rawVal / 132.48f) + 25.0f; // unit conversion ref datasheet

    return temp_celsius;
}