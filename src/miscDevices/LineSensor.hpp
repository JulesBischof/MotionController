#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "Tla2528.hpp"

typedef enum LineSensorStatus_t
{
    LINESENSOR_ERROR = 1 << 0,
    LINESENSOR_UV_ACTIVE = 1 << 1,
    LINESENSOR_CROSS_DETECTED = 1 << 2,
    LINESENSOR_NO_LINE = 1 << 3
} LineSensorStatus_t;

class LineSensor
{
private:

    uint8_t _status;

    Tla2528 *_adcInstance;

    uint8_t _uvGpio;

    void _toggleUvLed(bool state);
    void _initUvLed();

public:
    LineSensor(Tla2528 *adcInstance, uint8_t uvGpio);

    ~LineSensor();

    int8_t getLinePosition();

    uint32_t getLinePositionAnalog();

    uint8_t getStatus() { return _status; };
};

#endif