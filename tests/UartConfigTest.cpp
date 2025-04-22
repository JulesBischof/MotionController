#include "UartConfigTest.hpp"

#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include "MotionControllerPinning.hpp"

static void on_uart_rx()
{
    while (uart_is_readable(uart0))
    {
        uint8_t data = uart_getc(uart0);
        printf("recieved: %c\n", data);
    }
}

void UartConfigTest()
{
    uart_init(uart0, 115200);

    // init gpios
    gpio_set_function(Tx_UART0, UART_FUNCSEL_NUM(uart0, Tx_UART0));
    gpio_set_function(Rx_UART0, UART_FUNCSEL_NUM(uart0, Rx_UART0));

    // Set UART flow control CTS/RTS, we don't want these, so turn them off
    uart_set_hw_flow(uart0, false, false);

    // Set our data format
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);

    // Turn off FIFO's - use uart character by character
    uart_set_fifo_enabled(uart0, true);

    // set up and enable isr
    irq_set_exclusive_handler(UART0_IRQ, on_uart_rx);
    irq_set_enabled(UART0_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(uart0, true, false);

    for (;;)
    {
    }
}