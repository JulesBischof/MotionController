#include "LineSensor.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

/* ==================================
      constructor / deconstructor
   ================================== */

/// @brief creates an instance of LineSensor
/// @param adcInstance Instance of ADC - with whom the cell values can be read
/// @param uvGpio gpio connected to the UV-Tx - LEDs
LineSensor::LineSensor(Tla2528 *adcInstance, uint8_t uvGpio) : _adcInstance(adcInstance), _uvGpio(uvGpio)
{
    _status = 0;
    _initDefaultCalibration();
    _initUvLed();
}

/// @brief default consturctor
LineSensor::LineSensor(){}

LineSensor::~LineSensor()
{
    // not implemended yet
}

/* ==================================
            init Members
   ================================== */

/// @brief initializes UV-Transmitter
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

    printf("LINESENSOR - default calibration set \n");
}

/* ==================================
            getters & setters
   ================================== */
   
int8_t LineSensor::getLinePosition()
{
    // get ADC-value
    uint16_t adcValues[NUMBER_OF_CELLS] = {0};

    _toggleUvLed(true);
    bool err = _adcInstance->readAdc(adcValues);
    _toggleUvLed(false);

    if (!err)
    {
        printf("ERROR OCCURRED read adcValue in Line Sensor instance\n");
        return 0;
    }

    // convert ADC-values to digital values
    volatile uint8_t digArray[NUMBER_OF_CELLS] = {0};
    volatile int8_t linePosition = 0;
    int8_t lineCounter = 0;

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

    // check if there is a Line
    if (!lineCounter)
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

/// @brief get Line Position as an Analog value - see LineSensorConfig.h for maximum value
/// @return lineposition
uint32_t LineSensor::getLinePositionAnalog()
{
    // get ADC-value
    uint16_t adcValues[NUMBER_OF_CELLS] = {0};

    _toggleUvLed(true);
    bool err = _adcInstance->readAdc(adcValues);
    _toggleUvLed(false);

    if (!err)
    {
        printf("ERROR OCCURRED read adcValue in Line Sensor instance\n");
        _status |= LINESENSOR_ERROR;
        return -1;
    }

    uint8_t lineCounter = 0;

    volatile uint16_t normValues[NUMBER_OF_CELLS] = {0};

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

    // check if there is a Line
    if (!lineCounter)
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
        printf("LineSensor Read Analog DIVIDE ZERO!");
        _status |= LINESENSOR_ERROR;
        return LINESENSOR_MIDDLE_POSITION;
    }

    int16_t linePosition = numerator / denumerator;
    return linePosition;
}

/// @brief toggles UV-Transmitter due to safety reasons (UV is bad for your Eyes)
/// @param state state the led should be toggled to
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
        printf("ERROR - wrong calibration! _minMaxNormalize \n");
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