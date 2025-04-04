#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"
#include "hardware/pio.h"

#include <map>

#include "FreeRTOS.h"
#include "event_groups.h"
#include "semphr.h"

#include "HcSr04KalmanFilter.hpp"

class HcSr04
{
    private: 
        void _trigger();

        void _initHcSr04ISR();

        static void _hcSr04Irq(uint gpio, uint32_t events);

        volatile absolute_time_t _timeStampRising, _timeStampFalling;
        absolute_time_t _lastDt;

        uint8_t _triggerPin;
        uint8_t _echoPin;
        void _initGpios();

        HcSr04KalmanFilter _kalmanFilter;

        volatile EventGroupHandle_t _eventGroup;
        volatile EventBits_t _echoEvent;

        QueueHandle_t _queueHandle;
        void _initHcSr04Queue();

        TaskHandle_t _taskHandle;
        static void _HcSr04TaskWrapper(void *pv);
        void _HcSr04Task();

        float _getRawDistanceMm();

        SemaphoreHandle_t _currentVelocitySemaphore;
        float _currentVelocity;
        float _getCurrentVelocity();

        static std::map<uint, HcSr04 *> _instancesMap;
        static SemaphoreHandle_t _instancesMapSemaphore;

        static HcSr04 *_getInstaceFromMap(uint8_t gpio);
        static HcSr04 *_getInstaceFromMapFromISR(uint8_t gpio);

        void _startSensorTask();

    public : 
        HcSr04(){}
        HcSr04(uint8_t triggerPin, uint8_t echoPin);
        ~HcSr04();

        void triggerNewMeasurment();
        float getSensorData();

        void init();

        void setCurrentVelocity(float v);
};

#endif
