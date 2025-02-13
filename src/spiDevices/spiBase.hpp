#ifndef SPIBASE
#define SPIBASE

#include "hardware/spi.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"

// I2C ERRORCODES
#define STATUSOK 0;
#define TIMEOUT 1;

class spiBase
{

protected:
    spi_inst_t *_spiInstance;
    uint8_t _csPin;
    uint8_t _spiStatus;

    virtual void _initDevice();
    virtual void _checkDevice();

public:
    spiBase(spi_inst_t *spiInstance, uint8_t csPin);
    ~spiBase();

    uint8_t getStatus() { return this->_spiStatus; };

    uint32_t spiReadReg(uint8_t reg);
    uint32_t spiReadBitField(uint8_t reg, uint32_t mask, uint8_t shift);

    bool spiWriteReg(uint8_t reg, uint32_t data);
};

#endif
