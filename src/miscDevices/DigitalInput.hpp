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

        /// @brief look up table for ISR - Who's echo pin called the isr?
        static std::map<uint, DigitalInput *> _instancesMap;
        static SemaphoreHandle_t _instancesMapSemaphore;
        static void _dinGlobalIrq(uint gpio, uint32_t events);
        void (*_isrCallback)(uint gpio, uint32_t events);

    public:
        DigitalInput();
        DigitalInput(uint8_t gpio);
        ~DigitalInput();

        void addIsrHandler(void (*IsrCallback)(uint, uint32_t), uint32_t event);

        bool getValue();
    };

}
#endif // DIGITALINPUT_H