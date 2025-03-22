#ifndef DIGITALINPUT_H
#define DIGITALINPUT_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <map>

class DigitalInput
{
private:
    uint8_t _gpio;
    static std::map<uint8_t, SemaphoreHandle_t> _semaphoresMap;

public:
    DigitalInput(){}
    DigitalInput(uint8_t gpio);
    ~DigitalInput();

    bool getValue();
};

#endif // DIGITALINPUT_H