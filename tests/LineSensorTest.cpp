#include "LineSensorTest.h"

#include "LineSensor.hpp"
#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

#include "FreeRTOS.h"
#include "task.h"

void LineSensorTest(void)
{
    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);
    LineSensor lineSensor = LineSensor(&adc, 0);

    while(1)
    {
        int8_t linePosition = 0;
        linePosition = lineSensor.getLinePosition();

        printf("linePosition = %d\n", linePosition);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}