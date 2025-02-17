#ifndef TMC5240_H
#define TMC5240_H

#include <stdio.h>

#include "TMC5240_HW_Abstraction.h"
#include "StepperConfig.h"
# include "SpiBase.hpp"

// ---------------
// Status Flags
// ---------------
#define NOERROR 0
#define TIMEOUT 1
// ---------------

class Tmc5240 : public SpiBase
{
private:
    uint8_t _spi_status_flags;

    void _initDevice() override;
    void _checkDevice() override;

    void _initCurrentSetting();
    void _initSpreadCycle();

public:
    Tmc5240(spi_inst_t *spiInstance, uint8_t csPin);
    ~Tmc5240() {};

    uint8_t getStatusFlag() { return this->_spi_status_flags; };

    void setShaftDirection(bool direction);

    void moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax);
    void moveTargetMode(int32_t target_position, uint32_t v1, uint32_t v2, uint32_t a1, uint32_t a2, uint32_t dmax);
    void toggleToff();
};

#endif