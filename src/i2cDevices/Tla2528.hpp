#ifndef TLA2528_H
#define TLA2528_H

#include "I2cBase.hpp"
#include "Tla2528_HW_Abstraction.h"
#include <vector>
#include <stdio.h>

#define OPCODE_SINGLE_REGISTER_READ 0x10
#define OPCODE_SINGLE_REGISTER_WRITE 0x08

class Tla2528 : public I2cBase
{
private:
    void _initDevice() override;
    void _checkDevice() override;

    uint8_t _uvGpio;
    bool _uvLedState;
    void _toggleUvLight(bool state);

public:
    Tla2528(i2c_inst_t i2cInstance, uint8_t i2cAddress, uint8_t uvGpio);
    ~Tla2528();

    std::vector<uint16_t> readAdc();

    bool getUvLedState(){return _uvLedState;};

    bool i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num) override;

    bool i2cWriteReg(uint8_t reg, uint8_t data) override;
};

#endif
