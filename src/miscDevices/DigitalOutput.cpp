#include "DigitalOutput.hpp"

#include "pico/stdlib.h"

namespace miscDevices
{

    /// @brief
    DigitalOutput::DigitalOutput()
    {
        /* default ctor */
    }

    DigitalOutput::DigitalOutput(uint8_t gpio, bool initState)
    {
        _gpio = gpio;
        _state = initState;

        _initGpio();
        setState(initState);
    }

    DigitalOutput::~DigitalOutput()
    {
        /* decontructor - not implemented yet */
    }

    void DigitalOutput::_initGpio()
    {
        gpio_set_dir(_gpio, GPIO_OUT);
        return;
    }

    void DigitalOutput::toggle()
    {
        _state = !_state;
        gpio_put(_gpio, _state);
        return;
    }

    bool DigitalOutput::getState()
    {
        return _state;
    }

    void DigitalOutput::setState(bool state)
    {
        _state = state;
        gpio_put(_gpio, _state);
    }

}
