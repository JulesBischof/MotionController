#ifndef TMC5240_H
#define TMC5240_H

#include "SpiBase.hpp"

class Tmc5240 : public SpiBase
{
private:
    bool _stdDir;

    void _initCurrentSetting();
    void _initSpreadCycle();

protected:
    void _initDevice() override;
    void _checkDevice() override;

public:
    Tmc5240(){}
    Tmc5240(spi_inst_t *spiInstance, uint8_t csPin, bool stdDir);
    ~Tmc5240();

    uint8_t getLastStatusFlag() { return this->_spiStatus; };
    uint8_t getCurrentStatusFlag();

    void setShaftDirection(bool direction);

    void moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax);
    void moveAbsolutePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax);
    void moveRelativePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax);

    int32_t getXActual();

    int32_t getVmax();

    void toggleToff(bool val);
    
    static int32_t convertDistanceToMicrosteps(float meter);
    static int32_t convertMicrostepsToCentimeter(uint32_t uSteps);
    static int32_t convertDegreeToMicrosteps(int32_t degrees);
    static float convertDeltaDrivenDistanceToDegree(int32_t uStepsDifference);
};

#endif //TMC5240_H