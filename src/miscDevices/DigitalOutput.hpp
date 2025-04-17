#ifndef DIGITALOUTPUT_HPP
#define DIGITALOUTPUT_HPP

#include "pico/stdlib.h"

namespace miscDevices
{
    /// @brief object represents an Digital Output with its states. Reentrant!
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

        /// @brief toggle pin to its negative value
        void toggle();

        /// @brief gets the current state of the Output pin
        /// @return true if High
        bool getState();

        /// @brief Sets the Output to a desired state
        /// @param state true = high; false = low
        void setState(bool state);
    };
}
#endif
