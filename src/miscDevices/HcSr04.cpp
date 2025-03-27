#include "HcSr04.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

constexpr double SPEEDOFSOUND = 343.2f;
constexpr double CONVERSION_FACTOR = 2000.f;

constexpr double INITIAL_DISTANCE = 500; // mm ## = MAXDISTANCE !
constexpr double INITIAL_VELOCITY = 1;   // m/s
constexpr double INITIAL_DT = 0.1;       // s
constexpr double FILTER_QD = 10;         // mm²
constexpr double FILTER_QV = 1e-3;       // (m/s)²
constexpr double FILTER_R = 100;         // mm²

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
    _initHcSr04Queue();
    _initGpios();

    _eventGroup = xEventGroupCreate();
    _echoEvent = (1 << 0);
    xEventGroupClearBits(_eventGroup, _echoEvent);

    _currentVelocitySemaphore = xSemaphoreCreateMutex();

    // init IRQ
    gpio_set_irq_enabled_with_callback(_echoPin, GPIO_IRQ_EDGE_RISE, true, _hcSr04Irq);
    gpio_set_irq_enabled_with_callback(_echoPin, GPIO_IRQ_EDGE_FALL, true, _hcSr04Irq);

    _timeStampRising = get_absolute_time();
    _timeStampFalling = get_absolute_time();

    xSemaphoreTake(_instancesMapSemaphore, pdMS_TO_TICKS(100));
    _instancesMap.insert(std::make_pair(_echoPin, this));
    xSemaphoreGive(_instancesMapSemaphore);

    _kalmanFilter = HcSr04KalmanFilter(INITIAL_DISTANCE,
                                       INITIAL_VELOCITY,
                                       INITIAL_DT,
                                       FILTER_QD,
                                       FILTER_QV,
                                       FILTER_R);
    _initHcSr04ISR();
    _startHcSr04Task();
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
    gpio_put(_triggerPin, true);

    gpio_init(_echoPin);
    gpio_set_dir(_echoPin, GPIO_IN);
}

void HcSr04::_initHcSr04Queue()
{
    _HcSr04QueueHandle = xQueueCreate(1, sizeof(double));

    if (_HcSr04QueueHandle == nullptr)
    {
        /* ERROR..? */
    }

    double initQueueVal = INITIAL_DISTANCE;
    xQueueOverwrite(_HcSr04QueueHandle, &initQueueVal);

    return;
}

void HcSr04::_initHcSr04ISR()
{
    // Disable existing IRQs to avoid conflicts during init
    gpio_set_irq_enabled(_echoPin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Register the IRQ for echo pin
    gpio_set_irq_enabled_with_callback(
        _echoPin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true, // Enable the callback
        _hcSr04Irq);
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

void HcSr04::_startHcSr04Task()
{
    if (xTaskCreate(_HcSr04TaskWrapper,
                    "HcSr04Task",
                    1024 / sizeof(StackType_t),
                    this,
                    tskIDLE_PRIORITY + 3,
                    &_taskHandle) != pdTRUE)
    {
        /* ERROR HANDLING ? */
    }
    return;
}

void HcSr04::_HcSr04Task()
{
    while (true)
    {
        double rawDistance = _getRawDistanceMm();

        if (rawDistance < INITIAL_DISTANCE)
        {
            _kalmanFilter.setVelocity(_getCurrentVelocity());
            double dt = _lastDt / 1e6; // unit conversion us -> s
            _kalmanFilter.update(rawDistance, dt);
            double _filteredValue = _kalmanFilter.getDistance();

            xQueueOverwrite(_HcSr04QueueHandle, &_filteredValue);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
    /* never reached */
    return;
}

/* ==================================
            getters & setters
   ================================== */

/// @brief measures current distance to distant object
/// @return distance measured by hcsr04, value in mm
double HcSr04::_getRawDistanceMm()
{
    _trigger();
    EventBits_t bits = xEventGroupWaitBits(_eventGroup, _echoEvent, pdTRUE, pdFALSE, portMAX_DELAY);

    double retVal = INITIAL_DISTANCE;

    if (bits & _echoEvent)
    {
        _lastDt = absolute_time_diff_us(_timeStampRising, _timeStampFalling);
        retVal = (_lastDt * SPEEDOFSOUND) / CONVERSION_FACTOR;
    }

    return retVal;
}

/// @brief pulls the trigger pin
void HcSr04::_trigger()
{
    // TODO: trigger pin by PIO possible ???
    gpio_put(_triggerPin, false);
    sleep_us(10);
    gpio_put(_triggerPin, true);
}

HcSr04 *HcSr04::_getInstaceFromMap(uint8_t gpio)
{
    HcSr04 *inst;
    xSemaphoreTake(_instancesMapSemaphore, pdMS_TO_TICKS(100));
    inst = HcSr04::_instancesMap[gpio];
    xSemaphoreGive(_instancesMapSemaphore);

    return inst;
}

void HcSr04::setCurrentVelocity(double v)
{
    xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
    _currentVelocity = v;
    xSemaphoreGive(_currentVelocitySemaphore);
}

double HcSr04::_getCurrentVelocity()
{
    double retVal;
    xSemaphoreTake(_currentVelocitySemaphore, pdMS_TO_TICKS(100));
    retVal = _currentVelocity;
    xSemaphoreGive(_currentVelocitySemaphore);

    return retVal;
}

QueueHandle_t HcSr04::getQueueHandle()
{
    return _HcSr04QueueHandle;
}

TaskHandle_t HcSr04::getTaskHandle()
{
    return _taskHandle;
}

/* ==================================
            IRQ-Handler
   ================================== */

/// @brief irq callback function for echo-pin interrupt
/// @param gpio gpio that triggered the interrupt
/// @param events falling or rising edge
void HcSr04::_hcSr04Irq(uint gpio, uint32_t events)
{
    HcSr04 *inst = HcSr04::_getInstaceFromMap(gpio);

    if (events == GPIO_IRQ_EDGE_FALL)
        inst->_timeStampFalling = get_absolute_time();
    else
        inst->_timeStampRising = get_absolute_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(inst->_eventGroup, inst->_echoEvent, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}