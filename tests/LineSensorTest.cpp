#include "LineSensorTest.hpp"

#include "TestConfig.hpp"

#include "LineSensor.hpp"
#include "MotionControllerConfig.hpp"
#include "MotionControllerPinning.hpp"

#include "Tla2528.hpp"

#include "FreeRTOS.h"
#include "task.h"

void LineSensorTest(void)
{

#if TEST_LINESENSOR_ANALOGMODE == 0 && TEST_LINISENSOR_USING_TLA2528 == 1
    Tla2528 adc = Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

    while (1)
    {
        int8_t linePosition = 0;
        linePosition = lineSensor.getLinePosition();

        printf(" linePosition = %d\n", linePosition);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
#if TEST_LINESENSOR_ANALOGMODE == 1 && TEST_LINISENSOR_USING_TLA2528 == 1
    while (1)
    {
        int16_t linePositionAnalog = 0;
        linePositionAnalog = lineSensor.getLinePositionAnalog();

        printf("linePositionAnalog = %d\n", linePositionAnalog);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif

#if TEST_LINESENSOR_ANALOGMODE == 0 && TEST_LINESENSOR_USING_ARDUINO == 1

    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

    while(1)
    {
        int8_t linePosition = 0;
        linePosition = lineSensor.getLinePosition();

        printf("linePosition = %d\n", linePosition);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    #endif
#if TEST_LINESENSOR_ANALOGMODE == 1 && TEST_LINESENSOR_USING_ARDUINO == 1

    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

    while(1)
    {
        int16_t linePositionAnalog = 0;
        linePositionAnalog = lineSensor.getLinePositionAnalog();

        printf("linePositionAnalog = %d\n", linePositionAnalog);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    #endif

}