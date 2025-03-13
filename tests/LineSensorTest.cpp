#include "LineSensorTest.h"

#include "TestConfig.h"

#include "LineSensor.hpp"
#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

#include "FreeRTOS.h"
#include "task.h"

void LineSensorTest(void)
{
    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

#if TEST_LINESENSOR_ANALOGMODE == 0
    while(1)
    {
        int8_t linePosition = 0;
        linePosition = lineSensor.getLinePosition();

        printf("linePosition = %d\n", linePosition);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    #endif
#if TEST_LINESENSOR_ANALOGMODE == 1
    while(1)
    {
        int16_t linePositionAnalog = 0;
        linePositionAnalog = lineSensor.getLinePositionAnalog();

        printf("linePositionAnalog = %d\n", linePositionAnalog);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    #endif

}