#include "DigitalInput.hpp"
#include "LoggerService.hpp"
#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace miscDevices
{

    // statics
    std::map<uint, DigitalInput *> DigitalInput::_instancesMap;

    /* ==================================
        Constructor / Deconstructor
       ================================== */

    DigitalInput::DigitalInput(uint8_t gpio)
    {
        _gpio = gpio;
        gpio_init(_gpio);
        gpio_set_dir(_gpio, GPIO_IN);
    }

    void DigitalInput::_dinGlobalIrq(uint gpio, uint32_t events)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        DigitalInput *instance = _instancesMap[gpio];

        // run "personal" instance irq handler
        if (instance != nullptr)
        {
            if (instance->_isrCallback != nullptr)
            {
                instance->_isrCallback(gpio, events);
            }
        }
    } // end global isr

    /// @brief default Constructor
    DigitalInput::DigitalInput() {};

    DigitalInput::~DigitalInput()
    {
        /* not implemented yet */
    }

    void DigitalInput::addIsrHandler(void (*IsrCallback)(uint, uint32_t), uint32_t event)
    {
        DigitalInput::_instancesMap.insert(std::make_pair(_gpio, this));

        _isrCallback = IsrCallback;

        gpio_set_irq_enabled_with_callback(
            _gpio,
            event,
            true, // enable callback
            _dinGlobalIrq);
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