#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include <map>

#include "FreeRTOS.h"
#include "event_groups.h"

class Hcsr04
{
    private: 
        uint16_t _distance;

        void _trigger();

        static void _hcSr04Irq(uint gpio, uint32_t events);
        static std::map<uint, Hcsr04 *> _instancesMap;
        volatile absolute_time_t _timeStampRising;
        volatile absolute_time_t _timeStampFalling;

        void _initGpios();

        uint8_t _triggerPin;
        uint8_t _echoPin;

        uint64_t distance;

        volatile EventGroupHandle_t _eventGroup;
        volatile EventBits_t _echoEvent;

    public : 
        Hcsr04(){}
        Hcsr04(uint8_t triggerPin, uint8_t echoPin);
        ~Hcsr04();

        uint16_t getDistance_mm();
};

#endif
