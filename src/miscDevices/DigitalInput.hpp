#ifndef DIGITALINPUT_H
#define DIGITALINPUT_H

#include "FreeRTOS.h"
#include "semphr.h"
#include <map>

namespace miscDevices
{
    /// @brief Object that represents a Digital Input with it's states. Reentrant
    class DigitalInput
    {
    private:
        uint8_t _gpio;

    public:
        DigitalInput();
        DigitalInput(uint8_t gpio);
        ~DigitalInput();

        bool getValue();
    };

}
#endif // DIGITALINPUT_H