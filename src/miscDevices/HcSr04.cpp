#include "HcSr04.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

constexpr float SPEEDOFSOUND = 343.2f;
constexpr float CONVERSION_FACTOR = 2000.f;

constexpr float INITIAL_DISTANCE = 500; // mm ## = MAXDISTANCE !
constexpr float INITIAL_VELOCITY = 1;   // m/s
constexpr float INITIAL_DT = 0.1;       // s
constexpr float FILTER_QD = 10;         // mm²
constexpr float FILTER_QV = 1e-3;       // (m/s)²
constexpr float FILTER_R = 100;         // mm²

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
    setCurrentVelocity(0);
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
        xQueueOverwrite(_queueHandle, &initValue);
    }
}

void HcSr04::_initHcSr04Isr()
{
    // Disable existing IRQs to avoid conflicts during init
    gpio_set_irq_enabled(_echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Register the IRQ for echo pin
    gpio_set_irq_enabled_with_callback(
        _echoPin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, // enable callback
        _hcSr04GlobalIrq);
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

void HcSr04::startSensorTask()
{
    _initHcSr04Isr();
    
    if (xTaskCreate(_HcSr04TaskWrapper,
                    "HcSr04Task",
                    10 * 1024 / sizeof(StackType_t), // 10kb
                    this,
                    tskIDLE_PRIORITY + 2,
                    &_taskHandle) != pdTRUE)
    {
        /* ERROR HANDLING ? */
    }
    return;
}

void HcSr04::triggerNewMeasurment()
{
    xTaskNotifyGive(_taskHandle);
}

void HcSr04::_HcSr04Task()
{
    while (true)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        _trigger();

        uint32_t rawTimeDiff = 0;
        float distance = INITIAL_DISTANCE;

        if (xTaskNotifyWait(0x00, 0xFFFFFFFF, &rawTimeDiff, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            distance = static_cast<float>(rawTimeDiff) * SPEEDOFSOUND / CONVERSION_FACTOR; // mm
        }
        else
        {
            /* TIMEOUT ERROR ??? */
        }

        if (distance < INITIAL_DISTANCE)
        {
            _kalmanFilter.setVelocity(_getCurrentVelocity());
            float dt = rawTimeDiff / 1e6; // unit conversion us -> s
            _kalmanFilter.update(distance, dt);
            float _filteredValue = _kalmanFilter.getDistance();

            xQueueOverwrite(_queueHandle, &_filteredValue);
        }
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
    xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
    _currentVelocity = v;
    xSemaphoreGive(_currentVelocitySemaphore);
}

float HcSr04::_getCurrentVelocity()
{
    float retVal;
    xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
    retVal = _currentVelocity;
    xSemaphoreGive(_currentVelocitySemaphore);

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

        xTaskNotifyFromISR(_taskHandle, timeDiff, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
    }

    if (xHigherPriorityTaskWoken == pdTRUE)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}