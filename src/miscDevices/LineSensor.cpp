#include "LineSensor.hpp"

LineSensor::LineSensor(i2c_inst_t i2cInstance, uint8_t i2cAddress, uint8_t uvGpio) : Tla2528(i2cInstance, i2cAddress)
{
    _uvGpio = uvGpio;
    _uvLedState = false;

    _crossDetected = false;

    _initUvLed();
}

LineSensor::~LineSensor()
{
    // not implemended yet
}

int8_t LineSensor::getLinePosition()
{
    // get ADC-value
    _toggleUvLed(true);
    std::vector<uint16_t> adcValues = readAdc();
    _toggleUvLed(false);

    bool digArray[NUMBER_OF_CELLS] = {0};
    int8_t linePosition = 0;
    int8_t lineCounter = 0;

    // convert ADC-values to digital values
    for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
    {
        if (adcValues[i] > ADC_TRESHHOLD)
        {
            digArray[i] = 1;
        }
        else
        {
            digArray[i] = 0;
            lineCounter++;
        }
    }

    // check if there has been a crossway
    if (lineCounter >= LINECOUNTER_CROSS_DETECTED)
    {
        _crossDetected = true;
    }

    // determine line position (negative means more left, positive more right)
    for (size_t i = 0; i < NUMBER_OF_CELLS; i++)
    {
        if (digArray[i] == true)
            linePosition++;
    }
    for (size_t i = NUMBER_OF_CELLS; i > 0; i--)
    {
        if (digArray[i] == false)
            linePosition--;
    }

    return linePosition;
}

/// @brief initializes UV-Transmitter
void LineSensor::_initUvLed()
{
    gpio_init(_uvGpio);
    gpio_set_dir(_uvGpio, GPIO_OUT);
    gpio_put(_uvGpio, false);
    _uvLedState = false;

    return;
}

/// @brief toggles UV-Transmitter due to safety reasons
/// @param state state the led should be toggled to
void LineSensor::_toggleUvLed(bool state)
{
    gpio_put(_uvGpio, state);
    _uvLedState = state;

    return;
}
