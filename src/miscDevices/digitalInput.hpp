#ifndef DIGITALINPUT_H
#define DIGITALINPUT_H

#include "pico/stdlib.h"
#include "hardware/gpio.h"

class DigitalInput
{
private:
    uint8_t _gpio;

public:
    DigitalInput(uint8_t gpio);
    ~DigitalInput();

    bool getValue();
};

#endif // DIGITALINPUT_H