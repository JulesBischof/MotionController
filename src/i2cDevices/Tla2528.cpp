#include "Tla2528.hpp"

Tla2528::Tla2528(i2c_inst_t *i2cInstance, uint8_t i2cAddress) : I2cBase(i2cInstance, i2cAddress)
{
    _initDevice();
    _checkDevice();
}

Tla2528::~Tla2528()
{
}

/// @brief global confgiuration ADC-Converter
void Tla2528::_initDevice()
{
    // perform reset all registers to default values
    i2cWriteReg(GENERAL_CFG_ADDRESS, RST_COMPLETE);

    // set Sampling Rate 
    i2cWriteReg(OPMODE_CFG_ADDRESS, OSC_SEL_LOW_POWER | 0x00); // 32us - sampling ref p16 Datasheet

    // config all pins to default-value (0x00 = analog in)
    i2cWriteReg(PIN_CFG_ADDRESS, PIN_CFG_DEFAULT);

    // select manual mode
    i2cWriteReg(SEQUENCE_CFG_ADDRESS, SEQ_MODE_MANUAL);

    // set don't append channel-id to adc Value
    i2cWriteReg(DATA_CFG_ADDRESS, APPEND_STATUS_DISABLE);

    // set oversampling - process IC-internal average
    i2cWriteReg(OSR_CFG_ADDRESS, OSR_8);
}

/// @brief sends test-message to adc
void Tla2528::_checkDevice()
{
    uint8_t buffer = 0;
    i2cReadReg(SYSTEM_STATUS_ADDRESS, &buffer, 1);
    bool retVal = buffer >> 7;

    if (retVal)
        printf("ADC initialized");
    else
        printf("ADC NO ANSWER");
    return;

}

/// @brief reads ADC-Value
/// @return vector containing all raw ADC-Values
bool Tla2528::readAdc(uint16_t *buffer)
{

    for(size_t i = 0; i < 8; i++)
    {
        i2cWriteReg(MANUAL_CH_SEL_ADDRESS, i);

        uint8_t rawValue[2] = {0};
        i2cReadFrame(rawValue, 2);

        buffer[i] = (rawValue[0] << 8) | rawValue[1];
    }

    return true;
}

/// @brief performs a single register read. Overrides base-class due to individual Operation-Codes
/// @param reg register address
/// @param buffer pointer to a buffer
/// @param num numbers of bytes to read
/// @return true if no error occurred
bool Tla2528::i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num)
{
    int err = 0;

    // refer datasheet p20 - device needs operation code
    uint8_t readOpCodePackage[2] = {OPCODE_SINGLE_REGISTER_READ, reg};

    xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(100));

    // write with repeated start condition (read after adress has been set)
    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, readOpCodePackage, 2, true, 10000);
    err = i2c_read_timeout_us(_i2cInstance, _i2cAddress, buffer, num, false, 10000);

    xSemaphoreGive(_i2cMutex);

    if (err != 1)
    {
        _i2cStatus = I2C_TIMEOUT; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}

/// @brief performs a single register write. Overrides base-class due to individual Operation-Codes
/// @param reg register address
/// @param data register value
/// @return true if no error occurred
bool Tla2528::i2cWriteReg(uint8_t reg, uint8_t data)
{

    // refer datasheet p21 - device needs operation code
    uint8_t buffer[3] = {OPCODE_SINGLE_REGISTER_WRITE, reg, data};

    int err = 0;

    xSemaphoreTake(_i2cMutex, pdMS_TO_TICKS(100));

    err = i2c_write_timeout_us(_i2cInstance, _i2cAddress, buffer, 3, true, 10000);

    xSemaphoreGive(_i2cMutex);

    if (err != 2) // 2 due to 2Bytes data were send
    {
        _i2cStatus = I2C_TIMEOUT; // errors = PICO_ERROR_GENERIC or PICO_ERROR_TIMEOUT
        return false;
    }
    return true;
}
