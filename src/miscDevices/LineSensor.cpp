#include "LineSensor.hpp"

LineSensor::LineSensor(ArduinoAdcSlave *adcInstance, uint8_t uvGpio) : _adcInstance(adcInstance), _uvGpio(uvGpio)
{
    _status = 0 | LINESENSOR_OK | !LINESENSOR_UV_ACTIVE | !LINESENSOR_CROSS_DETECTED;

    _initUvLed();
}

LineSensor::~LineSensor()
{
    // not implemended yet
}

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
        if (adcValues[i] >= ADC_TRESHHOLD)
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
        _status |= LINESENSOR_CROSS_DETECTED;
    else
        _status &= ~LINESENSOR_CROSS_DETECTED;

    // determine line position (negative means more left, positive more right)
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

/// @brief initializes UV-Transmitter
void LineSensor::_initUvLed()
{
    gpio_init(_uvGpio);
    gpio_set_dir(_uvGpio, GPIO_OUT);
    _toggleUvLed(false);

    return;
}

/// @brief toggles UV-Transmitter due to safety reasons
/// @param state state the led should be toggled to
void LineSensor::_toggleUvLed(bool state)
{
    gpio_put(_uvGpio, state);
    _status = state ? (_status | LINESENSOR_UV_ACTIVE) : (_status & ~LINESENSOR_UV_ACTIVE);
    return;
}
