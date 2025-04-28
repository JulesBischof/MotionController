#include "DigitalInput.hpp"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

namespace miscDevices
{

    // statics
    std::map<uint8_t, SemaphoreHandle_t> DigitalInput::_semaphoresMap;

    /* ==================================
        Constructor / Deconstructor
       ================================== */

    DigitalInput::DigitalInput(uint8_t gpio)
    {
        _gpio = gpio;
        gpio_init(_gpio);
        gpio_set_dir(_gpio, GPIO_IN);

        // if semaphore does not exist already, create a new one
        if (_semaphoresMap.find(gpio) == _semaphoresMap.end())
        {
            taskENTER_CRITICAL();
            _semaphoresMap[gpio] = xSemaphoreCreateMutex();
            taskEXIT_CRITICAL();
        }
    }

    /// @brief default Constructor
    DigitalInput::DigitalInput() {};

    DigitalInput::~DigitalInput()
    {
        /* not implemented yet */
    }

    void DigitalInput::addIsrHandler(void (*IsrCallback)(uint, uint32_t), uint32_t event)
    {
        gpio_set_irq_enabled_with_callback(
            _gpio,
            event,
            true,
            IsrCallback);
    }

    /* ==================================
            getters & setters
       ================================== */

    bool DigitalInput::getValue()
    {
        bool retVal;

        xSemaphoreTake(_semaphoresMap[_gpio], pdMS_TO_TICKS(100));
        retVal = gpio_get(_gpio);
        xSemaphoreGive(_semaphoresMap[_gpio]);
        return gpio_get(_gpio);
    }
}