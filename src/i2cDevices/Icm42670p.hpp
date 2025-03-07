#ifndef ICM42670_H
#define ICM42670_H

#include "I2cBase.hpp"
#include "Icm42670p_HW_Abstraction.h"

class Icm42670p : public I2cBase
{
private:
    uint16_t _getRawDataZaxis();

protected:
    void _initDevice() override;
    void _checkDevice() override;

public:
    Icm42670p(i2c_inst_t i2cInstance, uint8_t i2cAddress);
    ~Icm42670p();
};

#endif