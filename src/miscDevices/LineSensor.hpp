#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "Tla2528.hpp"
#include "LineSensorConfig.h"

namespace miscDevices
{
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

        i2cDevices::Tla2528 *_adcInstance;

        uint8_t _uvGpio;

        uint16_t _calibValuesLow[NUMBER_OF_CELLS];
        uint16_t _calibValuesHigh[NUMBER_OF_CELLS];

        static uint16_t _minMaxNormalize(uint16_t val, uint16_t calibMin, uint16_t calibMax);
        void _toggleUvLed(bool state);
        void _initDefaultCalibration();
        void _initUvLed();

    public:
        LineSensor();

        LineSensor(i2cDevices::Tla2528 *adcInstance, uint8_t uvGpio);
        ~LineSensor();

        int8_t getLinePositionDigital();
        uint32_t getLinePositionAnalog();
        uint8_t getStatus() { return _status; };
    };
}
#endif