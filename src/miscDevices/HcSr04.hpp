#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include <map>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "HcSr04KalmanFilter.hpp"

namespace miscDevices
{

    class HcSr04
    {
    private:
        uint8_t _statusFlags;

        void _trigger();

        void _initHcSr04Isr();
        void _deInitHcSr04Isr();

        static void _hcSr04GlobalIrq(uint gpio, uint32_t events);
        void _hcSr04InstanceIrq(uint gpio, uint32_t events);

        uint8_t _triggerPin;
        uint8_t _echoPin;
        void _initGpios();

        HcSr04KalmanFilter _kalmanFilter;

        volatile uint32_t _rawtimediff;
        SemaphoreHandle_t _rawtimediffSemaphore;

        QueueHandle_t _queueHandle;
        void _initHcSr04Queue();

        TaskHandle_t _taskHandle;
        static void _HcSr04TaskWrapper(void *pv);
        void _HcSr04Task();

        SemaphoreHandle_t _currentVelocitySemaphore;
        float _currentVelocity;
        float _getCurrentVelocity();

        uint32_t _getHcSr04RawTimeDiff();

        static std::map<uint, HcSr04 *> _instancesMap;
        static SemaphoreHandle_t _instancesMapSemaphore;

    public:
        HcSr04();
        HcSr04(uint8_t triggerPin, uint8_t echoPin);
        ~HcSr04();

        void initMeasurmentTask();

        void triggerNewMeasurment();
        float getSensorData();

        uint8_t getStatusFlags() { return _statusFlags; }

        void init();

        void setCurrentVelocity(float v);
    };
}
#endif
