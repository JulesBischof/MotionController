#include "Hcsr04.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

/* ==================================
        Constructor / Deconstructor
   ================================== */

/// @brief creates instance of HCSR04 Ultrasonic distance-Sensor
/// @param triggerPin GPIO conntected to trigger
/// @param echoPin GPIO connected to echo
Hcsr04::Hcsr04(uint8_t triggerPin, uint8_t echoPin)
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
Hcsr04::~Hcsr04()
{
    // not implemented yet - maybe disable interrupts or else - instances hardly ever get deleted tho
}

/* ==================================
            Init Memebers
   ================================== */

/// @brief inits gpios
void Hcsr04::_initGpios()
{
    gpio_init(_triggerPin);
    gpio_set_dir(_triggerPin, GPIO_OUT);
    gpio_put(_triggerPin, false);

    gpio_init(_echoPin);
    gpio_set_dir(_echoPin, GPIO_IN);
}

/* ==================================
            getters & setters
   ================================== */

/// @brief measures current distance to distant object 
/// @return distance measured by hcsr04, value in mm
uint16_t Hcsr04::getDistance_mm()
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
void Hcsr04::_trigger()
{
    // TODO: trigger pin by PIO possible ??? 
    gpio_put(_triggerPin, true);
    sleep_us(10);
    gpio_put(_triggerPin, false);
}

/* ==================================
            IRQ-Handler
   ================================== */

/// @brief irq callback function for echo-pin interrupt
/// @param gpio gpio that triggered the interrupt
/// @param events falling or rising edge
void Hcsr04::_hcSr04Irq(uint gpio, uint32_t events)
{
    Hcsr04 *inst = Hcsr04::_instancesMap[gpio];

    if (events = GPIO_IRQ_EDGE_FALL)
        inst->_timeStampFalling = get_absolute_time();
    else
        inst->_timeStampRising = get_absolute_time();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xEventGroupSetBitsFromISR(inst->_eventGroup, inst->_echoEvent, &xHigherPriorityTaskWoken);
}