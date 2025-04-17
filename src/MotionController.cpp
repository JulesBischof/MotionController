#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"
#include "LineFollowerTaskConfig.h"
#include "RaspberryHatComTaskConfig.h"

#include "Tmc5240.hpp"

#include "MotionControllerConfig.h"
#include "MotionControllerPinning.h"
#include "Tmc5240Config.h"
#include <string.h>

namespace MtnCtrl
{

    /* ==================================
        static members & their getters
       ================================== */

    // queue handles delared as static due to they get messages from uart ISR
    QueueHandle_t MotionController::_raspberryHatComQueue = nullptr;
    QueueHandle_t MotionController::_lineFollowerQueue = nullptr;
    QueueHandle_t MotionController::_messageDispatcherQueue = nullptr;

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
        _initPeripherals();
    }

    /* ==================================
             Init Members
   ================================== */

    void MotionController::_initHardware()
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

        /* --------- DOUT ------------ */
        _tmc5240Eval_R2 = miscDevices::DigitalOutput(IREF_R2_DRIVER, STATE_EVALBOARD_R2);
        _tmc5240Eval_R3 = miscDevices::DigitalOutput(IREF_R3_DRIVER, STATE_EVALBOARD_R3);

        /* ERROR HANDLING ??? */
        return;
    }

    void MotionController::_initPeripherals()
    {
        _driver0 = spiDevices::Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
        _driver1 = spiDevices::Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);
        _adc = i2cDevices::Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
        _lineSensor = miscDevices::LineSensor(&_adc, UV_LED_GPIO);
        _safetyButton = miscDevices::DigitalInput(DIN_4);
        _tmc5240Eval_R2 = miscDevices::DigitalOutput(IREF_R2_DRIVER, STATE_EVALBOARD_R2);
        _tmc5240Eval_R3 = miscDevices::DigitalOutput(IREF_R3_DRIVER, STATE_EVALBOARD_R3);

        // The HcSr04 object requires dynamic allocation due to the need for the ISR
        // (Interrupt Service Routine) to access the object's pointer directly.
        // With static allocation, the object's values are copied to the _hcSr04 instance inside
        // the MotionController object, but the object's memory address does not remain the same,
        // which can lead to issues.
        // memory allocation is done using pvPortMalloc, which is a FreeRTOS function
        _hcSr04 = static_cast<miscDevices::HcSr04 *>(pvPortMalloc(sizeof(miscDevices::HcSr04)));
        if (_hcSr04 != nullptr)
        {
            new (_hcSr04) miscDevices::HcSr04(HCSR04_TRIGGER, HCSR04_ECHO);
        }
        else
        {
            for (;;)
            { /* ERROR ??? */
            }
        }

        return;
    }

    void MotionController::_initQueues()
    {
        _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _lineFollowerQueue = xQueueCreate(LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _messageDispatcherQueue = xQueueCreate(MESSAGEDISPATCHERTASKCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));

        if (_raspberryHatComQueue == nullptr || _lineFollowerQueue == nullptr || _messageDispatcherQueue == nullptr)
        {
            for (;;)
            { /* ERROR??? */
            }
        }
        return;
    }

    void MotionController::_initUart0Isr()
    {
        // Set UART flow control CTS/RTS, we don't want these, so turn them off ### sure???
        // uart_set_hw_flow(UART_INSTANCE_RASPBERRYHAT, false, false);

        // Set our data format
        // uart_set_format(UART_INSTANCE_RASPBERRYHAT, 8, 1, UART_PARITY_NONE);

        uart_set_fifo_enabled(UART_INSTANCE_RASPBERRYHAT, true);
        irq_set_exclusive_handler(UART0_IRQ, _uart0RxIrqHandler);
        irq_set_enabled(UART0_IRQ, true);
        uart_set_irq_enables(UART_INSTANCE_RASPBERRYHAT, true, false);
    }

    /* ==================================
             UART concerning Methods
       ================================== */

    void MotionController::sendUartMsg(frame *data, uart_inst_t *uartId)
    {
        // cpy value from data pointer
        char rawValue[8] = {0};
        memcpy(rawValue, data, sizeof(frame));

        // send bytewise
        for (int i = 0; i < 8; i++)
        { // 64 Bit = 8 Bytes
            uart_putc_raw(uartId, rawValue[i]);
        }

        _uartFlushTxWithTimeout(uartId, 10);

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
#if ENABLE_PRINTF_DEBUG_INFO
                printf("UART TX ERROR - TIMEOUT");
#endif
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

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }

    /* ==================================
          start Task and Methodwrapper
       ================================== */

    void MotionController::_LineFollerTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_lineFollowerTask();
    }

    /// @brief Creates LineFollower Task.
    void MotionController::_startLineFollowerTask()
    {
        if (xTaskCreate(_LineFollerTaskWrapper,
                        LINEFOLLOWERTASK_NAME,
                        LINEFOLLOWERTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + LINEFOLLOWERTASK_PRIORITY,
                        &_lineFollowerTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_RaspberryComTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_raspberryHatComTask();
    }

    /// @brief creates RaspberryHatComTask
    void MotionController::_startRaspberryHatComTask()
    {
        if (xTaskCreate(_RaspberryComTaskWrapper,
                        RASPBERRYHATCOMTASK_NAME,
                        RASPBERRYHATCOMTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + RASPBERRYHATCOMTASK_PRIORITY,
                        &_raspberryComTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_MessageDispatcherTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_messageDispatcherTask();
    }

    void MotionController::_startMessageDispatcherTask()
    {
        if (xTaskCreate(_MessageDispatcherTaskWrapper,
                        MESSAGEDISPATCHERTASK_NAME,
                        MESSAGEDISPATCHERTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + MESSAGEDISPATCHERTASK_PRIORITY,
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