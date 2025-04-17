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
    /// @brief Class Represents an instance of a HCSR04 Ultrasonic Sensor 
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
        /// @brief FreeRTOS expects static function pointers. This Wrapper calls an instances method 
        /// @param pv object instance HcSr04
        static void _HcSr04TaskWrapper(void *pv);
        void _HcSr04Task();

        SemaphoreHandle_t _currentVelocitySemaphore;
        float _currentVelocity;
        float _getCurrentVelocity();

        uint32_t _getHcSr04RawTimeDiff();

        /// @brief look up table for ISR - Who's echo pin called the isr? 
        static std::map<uint, HcSr04 *> _instancesMap;
        static SemaphoreHandle_t _instancesMapSemaphore;

    public:
        /// @brief default ctor
        HcSr04();

        /// @brief creates an instance ot HcSr04. Is OK to call before FreeRTOS scheduler is running
        /// @param triggerPin Pin connected to HcSr04 trigger
        /// @param echoPin Pin connected to HcSr04 eco
        HcSr04(uint8_t triggerPin, uint8_t echoPin);
        ~HcSr04();

        /// @brief initializes sensortask as well as ISR's. !!! needs to be called AFTER scheduler is running !!!
        void initMeasurmentTask();

        /// @brief triggers a new measurment cycle
        void triggerNewMeasurment();

        /// @brief reads result of last measurment
        /// @return last distance in mm
        float getSensorData();

        /// @brief getter for HcSr04 status Flags
        /// @return status Flags
        uint8_t getStatusFlags() { return _statusFlags; }

        /// @brief updates the Kalmann Filter state modell
        /// @param v current velocity in usteps/t (TMC5240 values!)
        void setCurrentVelocity(float v);
    };
}
#endif
