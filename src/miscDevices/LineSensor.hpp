#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "Tla2528.hpp"
#include "LineSensorConfig.h"

class LineSensor : public Tla2528
{
private:

    uint8_t _uvGpio;
    bool _uvLedState;
    void _toggleUvLed(bool state);
    void _initUvLed();

    bool _crossDetected;

public:
    LineSensor(i2c_inst_t i2cInstance, uint8_t i2cAddress, uint8_t uvGpio);
    ~LineSensor();

    int8_t getLinePosition();

    bool getUvLedState() { return _uvLedState; };
};

#endif