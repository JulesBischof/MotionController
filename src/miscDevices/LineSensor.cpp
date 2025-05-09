#include "LineSensor.hpp"

#include "pico/stdlib.h"
#include "LoggerService.hpp"

#include "MedianStack.hpp"
#include <cstring>

namespace miscDevices
{

    /* ==================================
          constructor / deconstructor
       ================================== */

    LineSensor::LineSensor(i2cDevices::Tla2528 *adcInstance, uint8_t uvGpio) : _adcInstance(adcInstance), _uvGpio(uvGpio)
    {
        _status = 0;
        _initDefaultCalibration();
        _initUvLed();
    }

    /// @brief default consturctor
    LineSensor::LineSensor() {}

    LineSensor::~LineSensor()
    {
        // not implemended yet
    }

    /* ==================================
                init Members
       ================================== */

    void LineSensor::_initUvLed()
    {
        gpio_init(_uvGpio);
        gpio_set_dir(_uvGpio, GPIO_OUT);
        _toggleUvLed(false);

        return;
    }

    /// @brief sets default values out of LineFollowerConfig.h file to calibration values
    void LineSensor::_initDefaultCalibration()
    {
        uint16_t defValuesLow[NUMBER_OF_CELLS] = LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES;
        uint16_t defValuesHigh[NUMBER_OF_CELLS] = LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES;

        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            this->_calibValuesLow[i] = defValuesLow[i];
            this->_calibValuesHigh[i] = defValuesHigh[i];
        }
        services::LoggerService::debug("LineSensor::_initDefaultCalibration", "default calibration set");
    }

    /* ==================================
                getters & setters
       ================================== */

    int8_t LineSensor::getLinePositionDigital()
    {
        // get ADC-value
        uint16_t adcValues[NUMBER_OF_CELLS] = {0};

        _toggleUvLed(true);
        bool err = _adcInstance->readAdc(adcValues);
        _toggleUvLed(false);

        if (!err)
        {
            services::LoggerService::fatal("LineSensor::getLinePositionDigital", "reading adcValue");
            return 0;
        }

        // convert ADC-values to digital values
        uint8_t digArray[NUMBER_OF_CELLS] = {0};
        int8_t linePosition = 0;
        int8_t lineCounter = 0;
        static uint8_t noLineCounter = 0;

        // convert ADC-values to digital values
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            if (adcValues[i] >= ADC_RAW_LINE_TRESHHOLD)
            {
                digArray[i] = 1;
            }
            else
            {
                digArray[i] = 0;
                lineCounter++;
            }
        }

        // if there is no line - count until LINECOUNTER_MAX_VALUE
        if (!lineCounter)
        {
            noLineCounter++;
        }
        else
        {
            noLineCounter = 0;
        }

        // if there is still no line - set status to no line detected
        if (!lineCounter && noLineCounter >= LINECOUNTER_MAX_VALUE)
        {
            _status |= LINESENSOR_NO_LINE;
        }
        else
        {
            _status &= ~LINESENSOR_NO_LINE;
        }

        // check if there has been a crossway
        if (lineCounter >= LINECOUNTER_CROSS_DETECTED)
        {
            _status |= LINESENSOR_CROSS_DETECTED;
        }
        else
        {
            _status &= ~LINESENSOR_CROSS_DETECTED;
        }

        // determine line position
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            if (digArray[i])
                linePosition++;
            else
                break;
        }

        for (size_t i = NUMBER_OF_CELLS - 1; i > 0; i--)
        {
            if (digArray[i])
                linePosition--;
            else
                break;
        }

        return linePosition;
    }

    uint32_t LineSensor::getLinePositionAnalog()
    {
        // get ADC-value
        uint16_t adcValues[NUMBER_OF_CELLS] = {0};

        _toggleUvLed(true);
        bool err = _adcInstance->readAdc(adcValues);
        _toggleUvLed(false);

        if (!err)
        {
            services::LoggerService::error("LineSensor::getLinePositionAnalog", "reading adcValue");
            _status |= LINESENSOR_ERROR;
            return LINESENSOR_MIDDLE_POSITION;
        }

        uint8_t lineCounter = 0;

        uint16_t normValues[NUMBER_OF_CELLS] = {0};

        // normalize and invert ADC values
        // invert values due to sensor is low active
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            normValues[i] = _minMaxNormalize(adcValues[i], _calibValuesLow[i], _calibValuesHigh[i]);
            normValues[i] = 1000 - normValues[i];
            if (normValues[i] > LINESENSOR_LINE_DETECTED_NORMLIZED) // bigger then due to values were invertet right before
            {
                lineCounter++;
            }
        }

        static uint8_t noLineCounter = 0;

        // if there is no line - count until LINECOUNTER_MAX_VALUE
        if (!lineCounter)
        {
            noLineCounter++;
        }
        else
        {
            noLineCounter = 0;
        }

        // if there is still no line - set status to no line detected
        if (!lineCounter && noLineCounter >= LINECOUNTER_MAX_VALUE)
        {
            _status |= LINESENSOR_NO_LINE;
            return LINESENSOR_MIDDLE_POSITION;
        }
        else
        {
            _status &= ~LINESENSOR_NO_LINE;
        }

        // check if there has been a crossway
        if (lineCounter > LINECOUNTER_CROSS_DETECTED)
            _status |= LINESENSOR_CROSS_DETECTED;
        else
            _status &= ~LINESENSOR_CROSS_DETECTED;

        // sum up all values
        uint32_t numerator = 0;
        uint32_t denumerator = 0;
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            numerator += normValues[i] * i * LINEPOSITION_ANALOG_SCALEFACTOR;
            denumerator += normValues[i];
        }

        if (!denumerator)
        {
            // this is just info logLevel due to this can happen if LineSensor stands on top of a Node
            services::LoggerService::info("LineSensor::_minMaxNormalize", "prevented division by zero");
            _status |= LINESENSOR_ERROR;
            return LINESENSOR_MIDDLE_POSITION;
        }

        int16_t linePosition = numerator / denumerator;
        return linePosition;
    }

    void LineSensor::_toggleUvLed(bool state)
    {
        gpio_put(_uvGpio, state);
        _status = state ? (_status | LINESENSOR_UV_ACTIVE) : (_status & ~LINESENSOR_UV_ACTIVE);
        return;
    }

    /* ==================================
                some helpers
       ================================== */

    uint16_t LineSensor::_minMaxNormalize(uint16_t val, uint16_t calibMin, uint16_t calibMax)
    {
        if (calibMax <= calibMin)
        {
            services::LoggerService::fatal("LineSensor::_minMaxNormalize", "wrong calibration value! calibMax <= calibMin");
            for (;;)
            {
            }
            return 0;
        }

        double val_d = static_cast<double>(val);
        double calibMin_d = static_cast<double>(calibMin);
        double calibMax_d = static_cast<double>(calibMax);

        double normalized = ((val_d - calibMin_d) / (calibMax_d - calibMin_d)) * static_cast<double>(LINESENSOR_NORMALIZE_REFERENCE_HIGH);

        if (normalized >= LINESENSOR_NORMALIZE_REFERENCE_HIGH)
        {
            normalized = LINESENSOR_NORMALIZE_REFERENCE_HIGH;
        }

        if (normalized <= 0)
        {
            normalized = 0;
        }

        uint16_t retVal = static_cast<uint16_t>(normalized);
        return retVal;
    }
    
    void LineSensor::lineSensorCalib(bool high)
    {
        // init buffers
        uint16_t buffer[NUMBER_OF_CELLS] = {0};
        MedianStack<uint16_t> stacks[NUMBER_OF_CELLS];
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            stacks[i] = MedianStack<uint16_t>(LINESENSOR_CONFIG_CALIBRATION_NUMBER_OF_MEASURMENTS);
        }

        // get analog data and push median stack
        for (size_t i = 0; i < LINESENSOR_CONFIG_CALIBRATION_NUMBER_OF_MEASURMENTS; i++)
        {
            _toggleUvLed(true);
            _adcInstance->readAdc(buffer);
            _toggleUvLed(false);

            // fill stack buffers
            for (size_t j; j < NUMBER_OF_CELLS; j++)
            {
                stacks[j].push(buffer[j]);
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // get medianesses
        for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
        {
            buffer[i] = stacks[i].getMedian();
        }

        // write to calibration values
        if(high)
        {
            std::memcpy(_calibValuesHigh, buffer, NUMBER_OF_CELLS * sizeof(buffer[0]));
        } else 
        {
            std::memcpy(_calibValuesLow, buffer, NUMBER_OF_CELLS * sizeof(buffer[0]));
        }
    }
}