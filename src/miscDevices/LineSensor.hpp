#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "ArduinoAdcSlave.hpp"
#include "LineSensorConfig.h"

#include <stdio.h>

typedef enum LineSensorStatus_t
{
    LINESENSOR_OK = 1,
    LINESENSOR_UV_ACTIVE = 2,
    LINESENSOR_CROSS_DETECTED = 4,
} LineSensorStatus_t;

class LineSensor
{
private:

    uint8_t _status;

    ArduinoAdcSlave *_adcInstance;

    uint8_t _uvGpio;

    void _toggleUvLed(bool state);
    void _initUvLed();

public:
    LineSensor(ArduinoAdcSlave *adcInstance, uint8_t uvGpio);
    ~LineSensor();

    int8_t getLinePosition();

    uint8_t getStatus() { return _status; };
};

#endif