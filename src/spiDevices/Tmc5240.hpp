#ifndef TMC5240_H
#define TMC5240_H

#include <stdio.h>

#include "TMC5240_HW_Abstraction.h"
#include "Tmc5240Config.h"
#include "SpiBase.hpp"

#include <cmath>

class Tmc5240 : public SpiBase
{
private:
    uint8_t _spi_status_flags;

    void _initCurrentSetting();
    void _initSpreadCycle();

protected:
    void _initDevice() override;
    void _checkDevice() override;

public:
    Tmc5240(spi_inst_t *spiInstance, uint8_t csPin);
    ~Tmc5240();

    uint8_t getStatusFlag() { return this->_spi_status_flags; };

    void setShaftDirection(bool direction);

    void moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax);
    void movePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax);

    int32_t getXActual();

    void toggleToff(bool val);
    
    uint32_t mpsToUStepsConversion(float mps);
};

#endif