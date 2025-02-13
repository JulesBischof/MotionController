#ifndef I2CBASE
#define I2CBASE

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "semphr.h"

// I2C ERRORCODES
#define STATUSOK 0;
#define TIMEOUT 1;

class i2cBase
{

protected:
    i2c_inst_t *_i2cInstance;
    uint8_t _i2cAddress;
    uint8_t _i2cStatus;

    virtual void _initDevice();
    virtual void _checkDevice();

public:
    i2cBase(i2c_inst_t i2cInstance, uint8_t i2cAddress);
    ~i2cBase();

    uint8_t getStatus() { return this->_i2cStatus; };

    bool i2cReadReg(uint8_t reg, uint8_t *buffer);

    bool i2cWriteReg(uint8_t reg, uint8_t data);
};

#endif
