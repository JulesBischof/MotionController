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

        volatile absolute_time_t _timeStampRising;
        volatile absolute_time_t _timeStampFalling;
        absolute_time_t _lastDt;

        uint8_t _triggerPin;
        uint8_t _echoPin;
        void _initGpios();

        HcSr04KalmanFilter _kalmanFilter;

        volatile EventGroupHandle_t _eventGroup;
        volatile EventBits_t _echoEvent;

        QueueHandle_t _HcSr04QueueHandle;
        void _initHcSr04Queue();

        TaskHandle_t _taskHandle;
        static void _HcSr04TaskWrapper(void *pv);
        void _HcSr04Task();
        void _startHcSr04Task();

        double _getRawDistanceMm();

        SemaphoreHandle_t _currentVelocitySemaphore;
        double _currentVelocity;
        double _getCurrentVelocity();

        static std::map<uint, HcSr04 *> _instancesMap;
        static SemaphoreHandle_t _instancesMapSemaphore;
        static HcSr04 *_getInstaceFromMap(uint8_t gpio);

    public : 
        HcSr04(){}
        HcSr04(uint8_t triggerPin, uint8_t echoPin);
        ~HcSr04();

        QueueHandle_t getQueueHandle();
        TaskHandle_t getTaskHandle();
        
        void setCurrentVelocity(double v);
};

#endif
