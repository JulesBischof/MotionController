#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"
#include "LineFollowerTaskConfig.h"
#include "RaspberryHatComTaskConfig.h"

#include "Tmc5240.hpp"

#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"

namespace MotionController
{

    /* ==================================
        static members & their getters
       ================================== */

    // queue handles delared as static due to they get messages from uart ISR
    QueueHandle_t MotionController::_raspberryHatComQueue = nullptr;
    QueueHandle_t MotionController::_lineFollowerQueue = nullptr;
    QueueHandle_t MotionController::_messageDispatcherQueue = nullptr;

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
                        LINEFOLLOWERTASK_NAME,
                        LINEFOLLOWERTASK_STACKSIZE,
                        this,
                        LINEFOLLOWERTASK_PRIORITY,
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
                        RASPBERRYHATCOMTASK_STACKSIZE,
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
        obj->_messageDispatcherTask();
    }

    void MotionController::_startMessageDispatcherTask()
    {
        if (xTaskCreate(_MessageDispatcherTaskWrapper,
                        MESSAGEDISPATCHERTASK_NAME,
                        MESSAGEDISPATCHERTASK_STACKSIZE,
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

        /*  
            alle 3 bumm
            dispatcher und raspicom: okay
            dispatcher und line follower: bumm
            raspicom und line follower: bumm
        */

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

    bool MotionController::_initQueues()
    {
        _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _lineFollowerQueue = xQueueCreate(LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _messageDispatcherQueue = xQueueCreate(MESSAGEDISPATCHERTASKCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));

        if (_raspberryHatComQueue == nullptr || _lineFollowerQueue == nullptr || _messageDispatcherQueue == nullptr)
        {
            for (;;)
            { /* ERROR??? */
            }
            return false; // never reached
        }
        return true;
    }

    void MotionController::_initUartIsr()
    {
        /* -------- UART 0 --------- */

        // Set UART flow control CTS/RTS, we don't want these, so turn them off ### sure???
        // uart_set_hw_flow(UART_INSTANCE_RASPBERRYHAT, false, false);

        // Set our data format
        // uart_set_format(UART_INSTANCE_RASPBERRYHAT, 8, 1, UART_PARITY_NONE);

        uart_set_fifo_enabled(UART_INSTANCE_RASPBERRYHAT, true);
        irq_set_exclusive_handler(UART0_IRQ, _uart0RxIrqHandler);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);

        // TODO: enable interrupts for uart 1 (gripctrl)
    }

    /* ==================================
                UART - Methods
       ================================== */

    void MotionController::sendUartMsg(frame *data, uart_inst_t *uartId)
    {
        uint64_t rawValue = reinterpret_cast<uint64_t>(data);
        for (int i = 0; i < 8; i++)
        { // 64 Bit = 8 Bytes
            uint8_t byte = (rawValue >> (i * 8)) & 0xFF;
            uart_putc_raw(UART_INSTANCE_RASPBERRYHAT, byte);
        }

        _uartFlushTxWithTimeout(UART_INSTANCE_RASPBERRYHAT, 10);
        return;
    }

    void MotionController::_uartFlushTxWithTimeout(uart_inst_t *uart, uint32_t timeout_ms)
    {
        uint8_t msTicks = 0;
        while (!uart_is_writable(UART_INSTANCE_RASPBERRYHAT))
        {
            msTicks++;
            vTaskDelay(pdMS_TO_TICKS(1));
            if (msTicks > timeout_ms)
            {
                printf("UART TX ERROR - TIMEOUT");
                return;
            }
        }
        return;
    }

    void MotionController::_uart0RxIrqHandler()
    {
        /* some message arrived!! send read command to RaspberryHatComQueue
        buffer-read and decoding shouldn't be handled inside a isr */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        QueueHandle_t queueHandle = getRaspberryHatComQueue();

        if (queueHandle != nullptr)
        {
            DispatcherMessage msg(
                DispatcherTaskId::RaspberryHatComTask,
                DispatcherTaskId::RaspberryHatComTask,
                TaskCommand::DecodeMessage,
                0);

            xQueueSendFromISR(queueHandle, &msg, &xHigherPriorityTaskWoken);
        }
        // disable uart interrupts
        uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, false, false);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        _initQueues();

        _driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
        _driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);
        _adc = Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
        _lineSensor = LineSensor(&_adc, UV_LED_GPIO);
        _safetyButton = DigitalInput(DIN_4);
        _lineFollowerStatusFlags = 0;
    }

    /* ==================================
                getters & setters
       ================================== */

    QueueHandle_t MotionController::getRaspberryHatComQueue()
    {
        return _raspberryHatComQueue;
    }

    QueueHandle_t MotionController::getLineFollowerQueue()
    {
        return _lineFollowerQueue;
    }

    QueueHandle_t MotionController::getMessageDispatcherQueue()
    {
        return _messageDispatcherQueue;
    }
}