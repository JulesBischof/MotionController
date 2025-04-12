#include "HcSr04.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

constexpr float SPEEDOFSOUND = 343.2f;
constexpr float CONVERSION_FACTOR = 2000.f;

constexpr float INITIAL_DISTANCE = 500;
constexpr float INITIAL_VELOCITY = 0;
constexpr float INITIAL_DT = 0.1;

constexpr float FILTER_QD = 5.0;  // position
constexpr float FILTER_QV = 1e-5; // velocity - stepper motors are very percise
constexpr float FILTER_R = 9;     // measurment noise HC-SR04 - Excel calculations

/* ==================================
            static Members
   ================================== */

std::map<uint, HcSr04 *> HcSr04::_instancesMap;
SemaphoreHandle_t HcSr04::_instancesMapSemaphore = xSemaphoreCreateMutex();

/* ==================================
        Constructor / Deconstructor
   ================================== */

/// @brief creates instance of HCSR04 Ultrasonic distance-Sensor
/// @param triggerPin GPIO conntected to trigger
/// @param echoPin GPIO connected to echo
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
                                       FILTER_R);

    _currentVelocitySemaphore = xSemaphoreCreateMutex();
    _currentVelocity = 0; // ctor runs in advance to scheduler - no need to protect

    _rawtimediffSemaphore = xSemaphoreCreateMutex();
    _rawtimediff = 0; // ctor runs in advance to scheduler - no need to protect
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
    absolute_time_t lastTriggerTime = get_absolute_time();
    
    while (true)
    {
        // get time increment
        absolute_time_t timeDiff = absolute_time_diff_us(lastTriggerTime, get_absolute_time());
        lastTriggerTime = get_absolute_time();
        float dt = static_cast<float>(timeDiff) / 1e6; // us -> s

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        _trigger();

        uint32_t rawTimeDiff = 0;
        float distance = INITIAL_DISTANCE;

        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, 0, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            rawTimeDiff = _getHcSr04RawTimeDiff();
            distance = static_cast<float>(rawTimeDiff) * SPEEDOFSOUND / CONVERSION_FACTOR; // mm
        }
        else
        {
            /* TIMEOUT ERROR ??? */
        }

        // printf("%f\n", distance); -- debug - get raw values

        _kalmanFilter.setVelocity(_getCurrentVelocity());
        _kalmanFilter.update(distance, dt);
        float _filteredValue = _kalmanFilter.getDistance();

        xQueueOverwrite(_queueHandle, &_filteredValue);
    }

    /* never reached */
    return;
}

/* ==================================
            getters & setters
   ================================== */

float HcSr04::getSensorData()
{
    float buffer = 500;

    if (_queueHandle == nullptr)
    {
        return buffer;
        /* ERROR ??? */
    }

    xQueuePeek(_queueHandle, &buffer, pdMS_TO_TICKS(10));
    return buffer;
}

/// @brief pulls the trigger pin
void HcSr04::_trigger()
{
    // TODO: trigger pin by PIO possible ???
    gpio_put(_triggerPin, false);
    sleep_us(10);
    gpio_put(_triggerPin, true); // pin is negative active
}

void HcSr04::setCurrentVelocity(float v)
{
    if (_currentVelocitySemaphore == nullptr)
    {
        return;
        /* ERROR ??? */
    }

    xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
    _currentVelocity = v;
    xSemaphoreGive(_currentVelocitySemaphore);
}

float HcSr04::_getCurrentVelocity()
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

/// @brief irq callback function for echo-pin interrupt.
/// @param gpio gpio that triggered the interrupt
/// @param events falling or rising edge
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
}

/// @brief instances personal irq handler. Gets triggered by global isr handler
/// @param gpio gpio that triggered the event
/// @param events event such as gpio rising or falling edge
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
}