#ifndef I2CBASE_H
#define I2CBASE_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"

// I2C ERRORCODES
#define STATUSOK 0;
#define TIMEOUT 1;

class I2cBase
{
private:
    static bool _i2cMutexInititalized;

protected:
    static SemaphoreHandle_t _i2cMutex;
    i2c_inst_t *_i2cInstance;
    uint8_t _i2cAddress;
    uint8_t _i2cStatus;

    virtual void _initDevice() = 0;
    virtual void _checkDevice() = 0;

public:
    I2cBase(i2c_inst_t i2cInstance, uint8_t i2cAddress);
    ~I2cBase();

    static void i2cInit(uint8_t sdaPin, uint8_t sckPin, i2c_inst_t *i2cInstance, uint16_t baudrate);

    uint8_t getStatus() { return this->_i2cStatus; };

    virtual bool i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num);
    virtual bool i2cReadFrame(uint8_t *buffer, uint8_t num);

    virtual bool i2cWriteReg(uint8_t reg, uint8_t data);
};

#endif
