#ifndef I2CBASE_H
#define I2CBASE_H

#include "FreeRTOS.h"
#include "semphr.h"

#include "hardware/i2c.h"
#include "pico/stdlib.h"

// I2C ERRORCODES
typedef enum I2cStatus_t
{
    I2C_STATUSOK = 0,
    I2C_TIMEOUT = 1,
    I2C_NO_ACK = 2
} I2cStatus_t;

class I2cBase
{
private:
    static bool _i2cMutexInititalized;

protected:
    static SemaphoreHandle_t _i2cMutex;
    i2c_inst_t *_i2cInstance;
    uint8_t _i2cAddress;
    I2cStatus_t _i2cStatus;

    virtual void _initDevice() = 0;
    virtual void _checkDevice() = 0;

    void _setError(uint8_t errorcode);

public:
    I2cBase(i2c_inst_t *i2cInstance, uint8_t i2cAddress);
    ~I2cBase();

    static void i2cInit(uint8_t sdaPin, uint8_t sckPin, i2c_inst_t *i2cInstance, uint16_t baudrate);

    I2cStatus_t getStatus() { return this->_i2cStatus; };

    virtual bool i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num);
    virtual bool i2cReadFrame(uint8_t *buffer, uint8_t num);

    virtual bool i2cWriteReg(uint8_t reg, uint8_t data);
};

#endif
