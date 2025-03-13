#include "digitalInput.hpp"

DigitalInput::DigitalInput(uint8_t gpio)
{
    _gpio = gpio;
    gpio_init(_gpio);
    gpio_set_dir(_gpio, GPIO_IN);
}

bool DigitalInput::getValue()
{
    return gpio_get(_gpio);
}
