#ifndef LINESENSOR_H
#define LINESENSOR_H

#include "Tla2528.hpp"
#include "LineSensorConfig.hpp"

namespace miscDevices
{
    typedef enum LineSensorStatus_t
    {
        LINESENSOR_ERROR = 1 << 0,
        LINESENSOR_UV_ACTIVE = 1 << 1,
        LINESENSOR_CROSS_DETECTED = 1 << 2,
        LINESENSOR_NO_LINE = 1 << 3
    } LineSensorStatus_t;

    /// @brief Object that represents the self made LineFollower
    class LineSensor
    {
    private:
        uint8_t _status;

        i2cDevices::Tla2528 *_adcInstance;

        uint8_t _uvGpio, _nolineCounter;

        uint16_t _calibValuesLow[NUMBER_OF_CELLS];
        uint16_t _calibValuesHigh[NUMBER_OF_CELLS];

        /// @brief normalizes raw Values of the Line Sensor into range 0 ... 1000
        /// @param val single Cell raw Sensor value 
        /// @param calibMin calibration minimum value
        /// @param calibMax calibration maximum value
        /// @return 
        static uint16_t _minMaxNormalize(uint16_t val, uint16_t calibMin, uint16_t calibMax);

        /// @brief initializes default calibration to LineSensor
        void _initDefaultCalibration();

        /// @brief initializes UV-Transmitter
        void _initUvLed();

    public:
        /// @brief default ctor
        LineSensor();

        /// @brief creates an instance of LineSensor
        /// @param adcInstance Instance of ADC - with whom the cell values can be read
        /// @param uvGpio gpio connected to the UV-Tx - LEDs
        LineSensor(i2cDevices::Tla2528 *adcInstance, uint8_t uvGpio);
        
        ~LineSensor();

        /// @brief calibrates the lineSensor based on a Median Value
        /// @param high true if high values should be set - false if low
        void lineSensorCalib(bool high);

        /// @brief get Line Position as an Analog value - see LineSensorConfig.h for maximum value
        /// @return lineposition
        uint32_t getLinePositionAnalog();

        /// @brief turns either on or off UV-tx LED's
        /// @param state true = LED's on; false = LED's off;
        void toggleUvLed(bool state);
        
        uint8_t getStatus() { return _status; };
    };
}
#endif