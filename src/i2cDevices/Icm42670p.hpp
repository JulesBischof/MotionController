#ifndef ICM42670_H
#define ICM42670_H

#include "I2cBase.hpp"
#include "Icm42670p_HW_Abstraction.h"
#include "tests.h"

#include "Icm42670pConfig.h"

class Icm42670p : public I2cBase
{
private:
   

protected:
    void _initDevice() override;
    void _checkDevice() override;

public:
    Icm42670p(i2c_inst_t *i2cInstance, uint8_t i2cAddress);
    ~Icm42670p();

    int16_t getRawDataZaxis();

    float getTemperature();
    
    float ConvertLsbToDps(int16_t rawVal);
};

#endif