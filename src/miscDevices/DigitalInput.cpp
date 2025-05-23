#include "DigitalInput.hpp"
#include "LoggerService.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace miscDevices
{
    /* ==================================
        Constructor / Deconstructor
       ================================== */

    DigitalInput::DigitalInput(uint8_t gpio)
    {
        _gpio = gpio;
        gpio_init(_gpio);
        gpio_set_dir(_gpio, GPIO_IN);
    }

    /// @brief default Constructor
    DigitalInput::DigitalInput() {};

    DigitalInput::~DigitalInput()
    {
        /* not implemented yet */
    }

    /* ==================================
            getters & setters
       ================================== */

    bool DigitalInput::getValue()
    {
        bool retVal;

        retVal = gpio_get(_gpio);

        return retVal;
    }
}