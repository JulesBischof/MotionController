#ifndef ARDUINOADCSLAVE_H
#define ARDUINOADCSLAVE_H

#define ADC_REQUEST_COMMAND ('R')

#include <vector>
#include "pico/stdlib.h"
#include "hardware/uart.h"

class ArduinoAdcSlave
{

private:
    uart_inst_t *_uart;

public:
    ArduinoAdcSlave(uart_inst_t *uart);
    ~ArduinoAdcSlave();

    bool readAdc(uint16_t *buffer);
};

#endif // ARDUINOADCSLAVE_H
