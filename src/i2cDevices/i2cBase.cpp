#include "i2cBase.hpp"

/// @brief initialises basic i2c Device
/// @param i2cInstance i2c0 / i2c1 refer pico datasheet
/// @param i2cAddress Bus-Adress i2c device
i2cBase::i2cBase(i2c_inst_t i2cInstance, uint8_t i2cAddress)
{
    _i2cAddress = i2cAddress;
    _i2cStatus = STATUSOK;
    _i2cInstance = &i2cInstance;

    this->_initDevice();
    this->_checkDevice();
}

/// @brief Deconstructor i2c baseclass - not implemented yet
i2cBase::~i2cBase()
{
    // no deconstructor
}

/// @brief reads registervalue into a buffer
/// @param reg register address
/// @param buffer pointer to a buffer
/// @return true if no error occurred
bool i2cBase::i2cReadReg(uint8_t reg, uint8_t *buffer)
{
    int err = 0;

    // disables all Interrupts and Scheduler activity from FreeRTOS
    taskENTER_CRITICAL();

    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, &reg, 1, true, 10000);
    err = i2c_read_timeout_us(_i2cInstance, _i2cAddress, buffer, 1, false, 10000);

    // returns to TimeSlicing Behaviour
    taskEXIT_CRITICAL();

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
bool i2cBase::i2cWriteReg(uint8_t reg, uint8_t data)
{
    uint8_t buffer[2] = {reg, data};

    int err = 0;

    // disables all Interrupts and Scheduler activity from FreeRTOS
    taskENTER_CRITICAL();

    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, buffer, 2, true, 10000);

    // returns to TimeSlicing Behaviour
    taskEXIT_CRITICAL();

    if (err != 2) // 2 due to 2Bytes data were send
    {
        _i2cStatus = err; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}
