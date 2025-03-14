#ifndef TLA2528_H
#define TLA2528_H

#include "I2cBase.hpp"

#define OPCODE_SINGLE_REGISTER_READ 0x10
#define OPCODE_SINGLE_REGISTER_WRITE 0x08

class Tla2528 : public I2cBase
{
protected:
    void _initDevice() override;
    void _checkDevice() override;

public:
    Tla2528(i2c_inst_t *i2cInstance, uint8_t i2cAddress);
    ~Tla2528();

    bool readAdc(uint16_t *buffer);

    bool i2cReadReg(uint8_t reg, uint8_t *buffer, uint8_t num) override;

    bool i2cWriteReg(uint8_t reg, uint8_t data) override;
};

#endif
