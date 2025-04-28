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
        static std::map<uint8_t, SemaphoreHandle_t> _semaphoresMap;

    public:
        DigitalInput();
        DigitalInput(uint8_t gpio);
        ~DigitalInput();

        void addIsrHandler(void (*IsrCallback)(uint, uint32_t), uint32_t event);

        bool getValue();
    };

}
#endif // DIGITALINPUT_H