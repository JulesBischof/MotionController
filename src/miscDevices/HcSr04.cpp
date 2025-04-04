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
    _initGpios();
    // no semaphore protection neccesary - while declaration no scheduler is active
    _instancesMap.insert(std::make_pair(_echoPin, this));

    _kalmanFilter = HcSr04KalmanFilter(INITIAL_DISTANCE,
                                       INITIAL_VELOCITY,
                                       INITIAL_DT,
                                       FILTER_QD,
                                       FILTER_QV,
                                       FILTER_R);
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
    _queueHandle = xQueueCreate(1, sizeof(float));

    if (_queueHandle == nullptr)
    {
        /* ERROR..? */
    }

    float initQueueVal = INITIAL_DISTANCE;
    xQueueOverwrite(_queueHandle, &initQueueVal);

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
        true, // enable callback
        _hcSr04Irq);
}

void HcSr04::init()
{
    _initHcSr04Queue();

    _eventGroup = xEventGroupCreate();
    _echoEvent = (1 << 0);
    xEventGroupClearBits(_eventGroup, _echoEvent);

    _currentVelocitySemaphore = xSemaphoreCreateMutex();

    _timeStampRising = get_absolute_time();
    _timeStampFalling = get_absolute_time();

    _initHcSr04ISR();
    _startSensorTask();

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

void HcSr04::_startSensorTask()
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

        float rawDistance = _getRawDistanceMm();

        if (rawDistance < INITIAL_DISTANCE)
        {
            _kalmanFilter.setVelocity(_getCurrentVelocity());
            float dt = _lastDt / 1e6; // unit conversion us -> s
            _kalmanFilter.update(rawDistance, dt);
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

/// @brief measures current distance to distant object
/// @return distance measured by hcsr04, value in mm
float HcSr04::_getRawDistanceMm()
{
    _trigger();
    EventBits_t bits = xEventGroupWaitBits(_eventGroup, _echoEvent, pdTRUE, pdFALSE, pdMS_TO_TICKS(100));

    float retVal = INITIAL_DISTANCE;

    if (bits & _echoEvent)
    {
        _lastDt = absolute_time_diff_us(_timeStampRising, _timeStampFalling);
        retVal = (_lastDt * SPEEDOFSOUND) / CONVERSION_FACTOR;
    }

    return retVal;
}

float HcSr04::getSensorData()
{
    float buffer = 500;
    xQueuePeek(_queueHandle, &buffer, pdMS_TO_TICKS(10));
    return buffer;
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

HcSr04 *HcSr04::_getInstaceFromMapFromISR(uint8_t gpio)
{
    HcSr04 *inst;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreTakeFromISR(_instancesMapSemaphore, &xHigherPriorityTaskWoken);
    inst = HcSr04::_instancesMap[gpio];
    xSemaphoreGiveFromISR(_instancesMapSemaphore, &xHigherPriorityTaskWoken);
    return inst;
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

/// @brief irq callback function for echo-pin interrupt
/// @param gpio gpio that triggered the interrupt
/// @param events falling or rising edge
void HcSr04::_hcSr04Irq(uint gpio, uint32_t events)
{
    HcSr04 *inst = HcSr04::_getInstaceFromMapFromISR(gpio);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // rising edge: measurment started
    if (events == GPIO_IRQ_EDGE_RISE)
    {
        inst->_timeStampRising = get_absolute_time();
    }

    // Falling edge: measurment ready
    if (events == GPIO_IRQ_EDGE_FALL)
    {
        inst->_timeStampFalling = get_absolute_time();
        xEventGroupSetBitsFromISR(inst->_eventGroup, inst->_echoEvent, &xHigherPriorityTaskWoken);
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}