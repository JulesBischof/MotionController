#include "ArduinoAdcSlave.hpp"

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstring>

ArduinoAdcSlave::ArduinoAdcSlave(uart_inst_t *uart)
{
    _uart = uart;
}

ArduinoAdcSlave::~ArduinoAdcSlave()
{
    // not implemented
}

bool ArduinoAdcSlave::readAdc(uint16_t *buffer)
{
    // Flush the UART buffer before reading
    while (uart_is_readable(_uart))
        uart_getc(_uart);
    
    // Send the request command
    uart_putc(_uart, ADC_REQUEST_COMMAND);

    // Wait for response
    while (!uart_is_readable(_uart))
        tight_loop_contents();

    char response[64] = {0}; // Buffer to hold the entire response string

    // Read the entire response string
    for (int i = 0; i < sizeof(response) -1 && uart_is_readable; i++)
    {
        response[i] = uart_getc(_uart);
        if (response[i] == '\n')
        {
            response[i] = '\0'; // Null-terminate the string
            break;
        }
    }

    // Parse the response string
    char *token = strtok(response, ":");
    int bufferIndex = 0;

    while (token != NULL && bufferIndex < 8)
    {
        buffer[bufferIndex++] = static_cast<uint16_t>(atoi(token));
        token = strtok(NULL, ":");
    }

    return (bufferIndex == 8);
}
