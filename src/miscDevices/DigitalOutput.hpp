#ifndef DIGITALOUTPUT_HPP
#define DIGITALOUTPUT_HPP

#include "pico/stdlib.h"

namespace miscDevices
{

    class DigitalOutput
    {
    private:
        uint8_t _gpio;
        bool _state;

        void _initGpio();

    public:
        DigitalOutput();
        DigitalOutput(uint8_t gpio, bool initState);
        ~DigitalOutput();

        void toggle();

        bool getState();
        void setState(bool state);
    };
}
#endif
