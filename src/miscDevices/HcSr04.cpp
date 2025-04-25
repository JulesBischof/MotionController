#include "HcSr04.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

#include "LoggerService.hpp"

#include "HcSr04Config.hpp"

namespace miscDevices
{
    /* ==================================
                static Members
       ================================== */

    std::map<uint, HcSr04 *> HcSr04::_instancesMap;
    SemaphoreHandle_t HcSr04::_instancesMapSemaphore = xSemaphoreCreateMutex();

    /* ==================================
            Constructor / Deconstructor
       ================================== */

    HcSr04::HcSr04(uint8_t triggerPin, uint8_t echoPin)
        : _triggerPin(triggerPin), _echoPin(echoPin)
    {
        // Disable existing IRQs to avoid conflicts during init
        _deInitHcSr04Isr();

        _statusFlags = 0;
        _initGpios();
        _initHcSr04Queue();

        // no semaphore protection neccesary - while declaration no scheduler is active
        _instancesMap.insert(std::make_pair(_echoPin, this));

        _kalmanFilter = HcSr04KalmanFilter(INITIAL_DISTANCE,
                                           INITIAL_VELOCITY,
                                           INITIAL_DT,
                                           FILTER_QD,
                                           FILTER_QV,
                                           FILTER_RD,
                                           FILTER_RV);

        _currentVelocitySemaphore = xSemaphoreCreateMutex();
        _currentVelocity = 0; // ctor runs in advance to scheduler - no need to protect

        _rawtimediffSemaphore = xSemaphoreCreateMutex();
        _rawtimediff = 0; // ctor runs in advance to scheduler - no need to protect

        _lastPredictionTimestampSemaphore = xSemaphoreCreateMutex();
        _lastPredictionTimestamp = get_absolute_time(); // ctor runs in advance to scheduler - no need to protect

        _kalmanFilterSemaphore = xSemaphoreCreateMutex();
    }

    HcSr04::HcSr04()
    {
        /* default ctor */
    }

    /// @brief deconstructor - not implemented yet
    HcSr04::~HcSr04()
    {
        // not implemented yet - maybe disable interrupts or else - instances hardly ever get deleted tho
    }

    /* ==================================
                Init Members
       ================================== */

    /// @brief inits gpios
    void HcSr04::_initGpios()
    {
        gpio_init(_triggerPin);
        gpio_set_dir(_triggerPin, GPIO_OUT);
        gpio_put(_triggerPin, true); // pin is negative active

        gpio_init(_echoPin);
        gpio_set_dir(_echoPin, GPIO_IN);
    }

    float HcSr04::_getTimeDiff()
    {
        xSemaphoreTake(_lastPredictionTimestampSemaphore, pdMS_TO_TICKS(100));
        absolute_time_t timeDiff = absolute_time_diff_us(_lastPredictionTimestamp, get_absolute_time());
        _lastPredictionTimestamp = get_absolute_time();
        xSemaphoreGive(_lastPredictionTimestampSemaphore);
        return timeDiff;
    }

    void HcSr04::_initHcSr04Queue()
    {
        _queueHandle = xQueueCreate(1, sizeof(float));
        if (_queueHandle == nullptr)
        {
            /* ERROR HANDLING ??? */
        }
        else
        {
            float initValue = INITIAL_DISTANCE;
            xQueueSendToFront(_queueHandle, &initValue, pdMS_TO_TICKS(0)); // add initial value
        }
    }

    void HcSr04::_initHcSr04Isr()
    {
        // Register the IRQ for echo pin
        gpio_set_irq_enabled_with_callback(
            _echoPin,
            GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
            true, // enable callback
            _hcSr04GlobalIrq);
        return;
    }

    void HcSr04::_deInitHcSr04Isr()
    {
        gpio_set_irq_enabled(_echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);
        return;
    }

    /* ==================================
            Task Concerning Members
       ================================== */
    void HcSr04::_HcSr04TaskWrapper(void *pv)
    {
        HcSr04 *inst = static_cast<HcSr04 *>(pv);
        inst->_HcSr04Task();
        return;
    }

    void HcSr04::initMeasurmentTask()
    {
        if (xTaskCreate(_HcSr04TaskWrapper,
                        "HcSr04Task",
                        10 * 1024 / sizeof(StackType_t), // 10kb
                        this,
                        tskIDLE_PRIORITY + 2,
                        &_taskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ? */
        }

        _initHcSr04Isr();
        return;
    }

    void HcSr04::triggerNewMeasurment()
    {
        if (_taskHandle == nullptr)
        {
            return;
            /* ERROR ??? */
        }

        xTaskNotifyGive(_taskHandle);
        return;
    }

    void HcSr04::_HcSr04Task()
    {
        TickType_t previousWakeTime = xTaskGetTickCount();

        while (true)
        {

            _trigger();

#if HCSR04CONFIG_USE_KALMAN_FILTER == (1)
            float dt = static_cast<float>(_getTimeDiff()) / 1e6; // us -> s
#endif

            uint32_t rawTimeDiff = 0;
            float distance = INITIAL_DISTANCE;

            // wait for interrupt event
            if (xTaskNotifyWait(0x00, 0xFFFFFFFF, 0, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                rawTimeDiff = _getHcSr04RawTimeDiff();
                distance = static_cast<float>(rawTimeDiff) * SPEEDOFSOUND / CONVERSION_FACTOR; // mm
            }
            else
            {
                /* TIMEOUT ERROR ??? */
            }

#if HCSR04CONFIG_USE_KALMAN_FILTER == (1)
            if (distance < INITIAL_DISTANCE)
            {
            xSemaphoreTake(_kalmanFilterSemaphore, pdMS_TO_TICKS(100));
            _kalmanFilter.update(distance, dt, getCurrentVelocity());
            xSemaphoreGive(_kalmanFilterSemaphore);
            }
            else 
            {
            xSemaphoreTake(_kalmanFilterSemaphore, pdMS_TO_TICKS(100));
            // set v to 0, otherwise kalmanfilter predicts movement - thats wrong
            _kalmanFilter.update(INITIAL_DISTANCE, dt, 0);
            xSemaphoreGive(_kalmanFilterSemaphore);
            }
#endif

#if HCSR04CONFIG_USE_RAW_VALUES == (1)
            float value = distance;
            xQueueOverwrite(_queueHandle, &value);
#endif

#if HCSR04CONFIG_USE_LOW_PASS_IIR == (1)
            static float lastVal = HCSR04CONFIG_DISTANCE_TRESHHOLD;

            value = HCSR04CONFIG_LOW_PASS_IIR_ALPHA * distance + (1.f - HCSR04CONFIG_LOW_PASS_IIR_ALPHA) * lastVal;
            lastVal = distance;
            xQueueOverwrite(_queueHandle, &value);
#endif

#if HCSR04CONFIG_PRINTF_RAW_DATA == (1)
            xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
            printf("%f;%f\n", distance, _currentVelocity);
            xSemaphoreGive(_currentVelocitySemaphore);
#endif
            vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS(HCSR04CONFIG_POLLING_RATE_MS));
        }

        /* never reached */
        return;
    }

    /* ==================================
                getters & setters
       ================================== */

    float HcSr04::getSensorData()
    {
#if HCSR04CONFIG_USE_KALMAN_FILTER == (1)
        float dt = static_cast<float>(_getTimeDiff()) / 1e6; // us -> s

        // access to kalmanFilter is protected by semaphore - so is retVal
        xSemaphoreTake(_kalmanFilterSemaphore, pdMS_TO_TICKS(100));
        float retVal = _kalmanFilter.getDistancePredicted(dt);
        xSemaphoreGive(_kalmanFilterSemaphore);
#else
        float retVal = HCSR04CONFIG_DISTANCE_TRESHHOLD;
        if (_queueHandle == nullptr)
        {
            services::LoggerService::fatal("HcSr04 getSensorData", "queueHandle is nullptr");
            return retVal;
            /* ERROR ??? */
        }
        xQueuePeek(_queueHandle, &retVal, pdMS_TO_TICKS(10));
#endif

#if HCSR04CONFIG_PRINTF_FILTEROUTPUT_DATA == (1)
        xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
        printf("%f;%f\n", retVal, _currentVelocity);
        xSemaphoreGive(_currentVelocitySemaphore);
#endif
        return retVal;
    }

    void HcSr04::_trigger()
    {
        // TODO: trigger pin by PIO possible ???
        gpio_put(_triggerPin, false);
        sleep_us(10);
        gpio_put(_triggerPin, true); // pin is negative active
    }

    void HcSr04::setCurrentVelocity(float v)
    {
        xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
        _currentVelocity = v;
        xSemaphoreGive(_currentVelocitySemaphore);
        services::LoggerService::debug("HcSr04 setCurrentCelocity", "set Current Velocity to %f", _currentVelocity);
    }

    float HcSr04::getCurrentVelocity()
    {
        if (_currentVelocitySemaphore == nullptr)
        {
            return 0;
            /* ERROR ??? */
        }

        float retVal;
        xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
        retVal = _currentVelocity;
        xSemaphoreGive(_currentVelocitySemaphore);

        return retVal;
    }

    uint32_t HcSr04::_getHcSr04RawTimeDiff()
    {
        uint32_t retVal = 0;

        retVal = _rawtimediff; // acces is atomic - no protection neccesseary

        return retVal;
    }

    /* ==================================
                IRQ-Handler
       ================================== */

    void HcSr04::_hcSr04GlobalIrq(uint gpio, uint32_t events)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        if (_instancesMapSemaphore != nullptr)
        {
            // get concerning instance
            xSemaphoreTakeFromISR(_instancesMapSemaphore, &xHigherPriorityTaskWoken);
            HcSr04 *instance = _instancesMap[gpio];
            xSemaphoreGiveFromISR(_instancesMapSemaphore, &xHigherPriorityTaskWoken);

            // run "personal" instance irq handler
            if (instance != nullptr)
            {
                instance->_hcSr04InstanceIrq(gpio, events);
            }
        }
    } // end global isr

    void HcSr04::_hcSr04InstanceIrq(uint gpio, uint32_t events)
    {
        static absolute_time_t risingEdge = 0;
        static absolute_time_t fallingEdge = 0;

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        uint32_t timeDiff = 0;

        if (events & GPIO_IRQ_EDGE_RISE)
        {
            risingEdge = get_absolute_time();
        }
        else if (events & GPIO_IRQ_EDGE_FALL)
        {
            fallingEdge = get_absolute_time();
            timeDiff = static_cast<uint32_t>(absolute_time_diff_us(risingEdge, fallingEdge));

            _rawtimediff = timeDiff; // access is atomic - no protection neccesary

            xTaskNotifyFromISR(_taskHandle, 0, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);
        }

        if (xHigherPriorityTaskWoken == pdTRUE)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    } // end instance isr
}