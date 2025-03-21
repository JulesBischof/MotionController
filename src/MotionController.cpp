#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"
#include "LineFollowerTaskConfig.h"
#include "RaspberryHatComTaskConfig.h"

#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

#include "queues.h"

/* ==================================
        start Task and Wrapper
   ================================== */

// line follower

void MotionController::_LineFollerTaskWrapper(void *pvParameters)
{
    MotionController *obj = static_cast<MotionController *>(pvParameters);
    obj->_lineFollowerTask();
}

void MotionController::_startLineFollowerTask()
{
    if (xTaskCreate(_LineFollerTaskWrapper,
                    MESSAGEDISPATCHERTASK_NAME,
                    MESSAGEDISPATCHERTASK_STACKSIZE / sizeof(StackType_t),
                    this,
                    MESSAGEDISPATCHERTASK_PRIORITY,
                    &_lineFollowerTaskHandle) != pdTRUE)
    {
        /* ERROR HANDLING ??? */
    }
}

// raspi com

void MotionController::_RaspberryComTaskWrapper(void *pvParameters)
{
    MotionController *obj = static_cast<MotionController *>(pvParameters);
    obj->_raspberryHatComTask();
}

void MotionController::_startRaspberryHatComTask()
{
    if (xTaskCreate(_RaspberryComTaskWrapper,
                    RASPBERRYHATCOMTASK_NAME,
                    RASPBERRYHATCOMTASK_STACKSIZE / sizeof(StackType_t),
                    this,
                    RASPBERRYHATCOMTASK_PRIORITY,
                    &_raspberryComTaskHandle) != pdTRUE)
    {
        /* ERROR HANDLING ??? */
    }
}

// msg dispatcher

void MotionController::_MessageDispatcherTaskWrapper(void *pvParameters)
{
    MotionController *obj = static_cast<MotionController *>(pvParameters);
    obj->_raspberryHatComTask();
}

void MotionController::_startMessageDispatcherTask()
{
    if (xTaskCreate(_MessageDispatcherTaskWrapper,
                    MESSAGEDISPATCHERTASK_NAME,
                    MESSAGEDISPATCHERTASK_STACKSIZE / sizeof(StackType_t),
                    this,
                    MESSAGEDISPATCHERTASK_PRIORITY,
                    &_messageDispatcherTaskHandle) != pdTRUE)
    {
        /* ERROR HANDLING ??? */
    }
}

void MotionController::startScheduler()
{
    _startLineFollowerTask();
    _startMessageDispatcherTask();
    _startRaspberryHatComTask();

    vTaskStartScheduler();
    for (;;)
    {
        /* never reached */
    }
}

/* ==================================
                Init
   ================================== */

bool MotionController::_initHardware()
{
    /* ---------- UART ---------- */
    gpio_set_function(Tx_UART0, UART_FUNCSEL_NUM(UART_INSTANCE_RASPBERRYHAT, Tx_UART0));
    gpio_set_function(Rx_UART0, UART_FUNCSEL_NUM(UART_INSTANCE_RASPBERRYHAT, Rx_UART0));
    uart_init(uart0, UART0_BAUDRATE);

    gpio_set_function(Tx_UART1, UART_FUNCSEL_NUM(UART_INSTANCE_GRIPCONTROLLER, Tx_UART1));
    gpio_set_function(Rx_UART1, UART_FUNCSEL_NUM(UART_INSTANCE_GRIPCONTROLLER, Rx_UART1));
    uart_init(UART_INSTANCE_GRIPCONTROLLER, UART1_BAUDRATE);

    /* ---------- I2C ---------- */
    i2c_init(I2C_INSTANCE_DEVICES, I2C_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(I2C0_SCK, GPIO_FUNC_I2C);
    gpio_set_function(I2C0_SDA, GPIO_FUNC_I2C);

    /* ---------- SPI ---------- */
    spi_init(TMC5240_SPI_INSTANCE, SPI_BAUDRATE_KHZ * 1e3); // set Baudrate
    gpio_set_function(SPI_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SPI_SCK, GPIO_FUNC_SPI);

    gpio_init(SPI_CS_DRIVER_0);
    gpio_set_dir(SPI_CS_DRIVER_0, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_0, 1); // pull up CS

    gpio_init(SPI_CS_DRIVER_1);
    gpio_set_dir(SPI_CS_DRIVER_1, GPIO_OUT);
    gpio_put(SPI_CS_DRIVER_1, 1); // pull up CS

    spi_set_format(TMC5240_SPI_INSTANCE, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    /* ERROR HANDLING ??? */
    return true;
}

bool MotionController::_initPeripherals()
{
    Tmc5240 _driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
    Tmc5240 _driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);

    Tla2528 _adc = Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
    LineSensor _lineSensor = LineSensor(&_adc, UV_LED_GPIO);

    _safetyButton = DigitalInput(DIN_4);

    /* ERROR HANDLING ??? */
    return true;
}

bool MotionController::_initQueues()
{
    _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));
    _lineFollowerQueue = xQueueCreate(LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));
    _messageDispatcherQueue = xQueueCreate(MESSAGEDISPATCHERTASKCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));
    /* ERROR HANDLING ??? */
    return true;
}

bool MotionController::_initUartIsr(uart_inst_t uartId)
{
    int uartIRQId = (uartId == (uart0) ? UART0_IRQ : UART1_IRQ);

    /*  ---- enable uart interrupt on Rx events ---- */

    // Set UART flow control CTS/RTS, we don't want these, so turn them off ### sure???
    // uart_set_hw_flow(UART_INSTANCE_RASPBERRYHAT, false, false);

    // Set our data format
    // uart_set_format(UART_INSTANCE_RASPBERRYHAT, 8, 1, UART_PARITY_NONE);

    // Turn on FIFO's
    uart_set_fifo_enabled(UART_INSTANCE_RASPBERRYHAT, true);
    // set up and enable isr
    irq_set_exclusive_handler(uartIRQId, _uartRxIrqHandler);
    irq_set_enabled(uartIRQId, true);
    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);
}

/* ==================================
        Constructor/Deconstructor
   ================================== */

MotionController::~MotionController()
{
    /* NOT IMPLEMENTED YET */
}

MotionController::MotionController()
{
    _initHardware();
    _initPeripherals();
    _initQueues();

    _lineFollowerStatusFlags = 0;
}