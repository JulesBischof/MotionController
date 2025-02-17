#include "I2cBase.hpp"

// init static class members
SemaphoreHandle_t I2cBase::_i2cMutex = NULL;
bool I2cBase::_i2cMutexInititalized = false;

/// @brief initialises basic i2c Device
/// @param i2cInstance i2c0 / i2c1 refer pico datasheet
/// @param i2cAddress Bus-Adress i2c device
I2cBase::I2cBase(i2c_inst_t i2cInstance, uint8_t i2cAddress)
    : _i2cInstance(&i2cInstance), _i2cAddress(i2cAddress)
{
    _i2cStatus = STATUSOK;

    if (!this->_i2cMutexInititalized)
    {
        _i2cMutex = xSemaphoreCreateMutex();
        _i2cMutexInititalized = true;
    }

    this->_initDevice();
    this->_checkDevice();
}

/// @brief Deconstructor i2c baseclass - not implemented yet
I2cBase::~I2cBase()
{
    // no deconstructor
}

/// @brief provides a readframe for i2c bus. No Register Adress neccessary - just read slave
/// @param buffer pointer to a buffer
/// @param num  numbers of bytes to read
/// @return true if no error occurred
bool I2cBase::i2cReadFrame(uint8_t *buffer, uint8_t num)
{
    int err = 0;

    xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(100));

    err = i2c_read_timeout_us(_i2cInstance, _i2cAddress, buffer, num, false, 10000);

    xSemaphoreGive(_i2cMutex);

    if (err != 1)
    {
        _i2cStatus = err; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}

/// @brief reads registervalue into a buffer
/// @param reg register address
/// @param buffer pointer to a buffer
/// @param num numbers of bytes to read
/// @return true if no error occurred
bool I2cBase::i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num)
{
    int err = 0;

    xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(100));

    // write with repeated start condition (read after adress has been set)
    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, &reg, 1, true, 10000);
    err = i2c_read_timeout_us(_i2cInstance, _i2cAddress, buffer, num, false, 10000);

    xSemaphoreGive(_i2cMutex);

    if (err != 1)
    {
        _i2cStatus = err; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}

/// @brief writes data to a specific register
/// @param reg register address
/// @param data 8-bit data to send
/// @return true if no error occurred
bool I2cBase::i2cWriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};

    int err = 0;

    xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(100));

    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, buffer, 2, true, 10000);

    xSemaphoreGive(_i2cMutex);

    if (err != 2) // 2 due to 2Bytes data were send
    {
        _i2cStatus = err; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}
