#include "hcsr04.hpp"

/// @brief creates instance of HCSR04 Ultrasonic distance-Sensor
/// @param triggerPin GPIO conntected to trigger
/// @param echoPin GPIO connected to echo
hcsr04::hcsr04(uint8_t triggerPin, uint8_t echoPin)
    : _triggerPin(triggerPin), _echoPin(echoPin)
{
    _initGpios();
    _eventGroup = xEventGroupCreate();
    _echoEvent = 0;

    // init IRQ
    gpio_set_irq_enabled_with_callback(_echoPin, GPIO_IRQ_EDGE_RISE, true, _hcSr04Irq);
    gpio_set_irq_enabled_with_callback(_echoPin, GPIO_IRQ_EDGE_FALL, true, _hcSr04Irq);

    _timeStampRising = get_absolute_time();
    _timeStampFalling = get_absolute_time();

    _instancesMap.insert(std::make_pair(_echoPin, this));
}

/// @brief deconstructor - not implemented yet
hcsr04::~hcsr04()
{
    // not implemented yet - maybe disable interrupts or else - instances hardly ever get deleted tho
}

/// @brief measures current distance to distant object 
/// @return distance measured by hcsr04, value in mm
uint16_t hcsr04::getDistance_mm()
{
    _trigger();
    EventBits_t bits = xEventGroupWaitBits(_eventGroup, _echoEvent, pdTRUE, pdFALSE, portMAX_DELAY);

    uint16_t distance_mm;
    
    if (bits & _echoEvent)
    {
        absolute_time_t diff = absolute_time_diff_us(_timeStampRising, _timeStampFalling);
        distance_mm = (diff * 343) / 2000;
    }

    return distance_mm;
}

/// @brief pulls the trigger pin
void hcsr04::_trigger()
{
    // TODO: trigger pin by PIO possible ??? 
    gpio_put(_triggerPin, true);
    sleep_us(10);
    gpio_put(_triggerPin, false);
}

/// @brief irq callback function for echo-pin interrupt
/// @param gpio gpio that triggered the interrupt
/// @param events falling or rising edge
void hcsr04::_hcSr04Irq(uint gpio, uint32_t events)
{
    hcsr04 *inst = hcsr04::_instancesMap[gpio];

    if (events = GPIO_IRQ_EDGE_FALL)
        inst->_timeStampFalling = get_absolute_time();
    else
        inst->_timeStampRising = get_absolute_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(inst->_eventGroup, inst->_echoEvent, &xHigherPriorityTaskWoken);
}

/// @brief inits gpios
void hcsr04::_initGpios()
{
        gpio_init(_triggerPin);
        gpio_set_dir(_triggerPin, GPIO_OUT);
        gpio_put(_triggerPin, false);

        gpio_init(_echoPin);
        gpio_set_dir(_echoPin, GPIO_IN);
}
