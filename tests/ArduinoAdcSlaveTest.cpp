#include "ArduinoAdcSlaveTest.h"

void ArduinoAdcSlaveTest(void)
{
    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);

    printf("Arduino adc Initialized ...\n");

    while (1)
    {
        uint16_t buffer[8] = {0};

        adc.readAdc(buffer);

        for(size_t i = 0; i < 8; i++)
        {
            printf("ADC[%d]: %d\n", i, buffer[i]);
        }

        sleep_ms(1000);
    }
}