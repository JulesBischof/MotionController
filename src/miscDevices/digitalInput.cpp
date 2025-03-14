#include "digitalInput.hpp"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

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

bool DigitalInput::getValue()
{
    bool retVal;

    xSemaphoreTake(_semaphoresMap[_gpio], pdMS_TO_TICKS(100));
    retVal = gpio_get(_gpio);
    xSemaphoreGive(_semaphoresMap[_gpio]);
    return gpio_get(_gpio);
}
