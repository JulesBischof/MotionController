#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.hpp"
#include "LineFollowerTaskConfig.hpp"
#include "RaspberryHatComTaskConfig.hpp"
#include "GripControllerComTaskConfig.hpp"
#include "BarrierHandlerConfig.hpp"

#include "Tmc5240.hpp"

#include "MotionControllerConfig.hpp"
#include "MotionControllerPinning.hpp"
#include "Tmc5240Config.hpp"
#include <string.h>

namespace MtnCtrl
{

    /* ==================================
        static members & their getters
       ================================== */

    // queue handles delared as static due to they might get messages from ISR
    QueueHandle_t MotionController::_raspberryHatComQueue = nullptr;
    QueueHandle_t MotionController::_gripControllerComQueue = nullptr;
    QueueHandle_t MotionController::_lineFollowerQueue = nullptr;
    QueueHandle_t MotionController::_messageDispatcherQueue = nullptr;
    QueueHandle_t MotionController::_barrierHandlerQueue = nullptr;

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

#if USE_TEST_SHELL == 1
        _testShell = services::TestShell(_messageDispatcherQueue);
#endif
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
        _lamp = miscDevices::DigitalOutput(DOUT_1, false);

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

    void MotionController::_safetyButtonIrqHandler(uint gpio, uint32_t event)
    {
        services::LoggerService::debug("_safetyButtonIrqHandler", "safety Button pressed!");
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // send info to RaspberryHat
        DispatcherMessage msg(DispatcherTaskId::NoTask,
                              DispatcherTaskId::RaspberryHatComTask,
                              TaskCommand::SafetyButtonInfo,
                              0);
        xQueueSendFromISR(_raspberryHatComQueue, &msg, &xHigherPriorityTaskWoken);

        // send a stop Broadcast to all Tasks and yield (stop yield is more important)
        msg = DispatcherMessage(DispatcherTaskId::NoTask,
                                DispatcherTaskId::Broadcast,
                                TaskCommand::Stop,
                                0);

        xQueueSendFromISR(_messageDispatcherQueue, &msg, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }

    void MotionController::_initQueues()
    {
        _raspberryHatComQueue = xQueueCreate(RASPBERRYHATCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _lineFollowerQueue = xQueueCreate(LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _messageDispatcherQueue = xQueueCreate(MESSAGEDISPATCHERTASKCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
        _barrierHandlerQueue = xQueueCreate(BARRIERHANDLERTASK_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));

        _safetyButtonPressed = xEventGroupCreate();
        if (_safetyButtonPressed == NULL)
        {
            // ERROR
            for (;;)
                ;
        }
#if INCLUDE_GRIPCONTROLLER_AS_INSTANCE == (0)
        _gripControllerComQueue = xQueueCreate(GRIPCONTROLLERCOMTASK_QUEUESIZE_N_ELEMENTS, sizeof(DispatcherMessage));
#endif

        if (_raspberryHatComQueue == nullptr ||
            _lineFollowerQueue == nullptr ||
            _messageDispatcherQueue == nullptr ||
            _barrierHandlerQueue == nullptr)
        {
            services::LoggerService::fatal("_initQueues()", "nullptr");
            for (;;)
            { /* ERROR??? */
            }
        }
        return;
    }

    void MotionController::_initUartRxIsr(uart_inst_t *uartId, void (*callback)())
    {
        irq_num_rp2350 UARTX_IRQ = (uartId == uart0) ? UART0_IRQ : UART1_IRQ;

        // Set UART flow control CTS/RTS, we don't want these, so turn them off ### sure???
        // uart_set_hw_flow(UART_INSTANCE_RASPBERRYHAT, false, false);

        // Set our data format
        // uart_set_format(UART_INSTANCE_RASPBERRYHAT, 8, 1, UART_PARITY_NONE);

        uart_set_fifo_enabled(uartId, true);
        irq_set_exclusive_handler(UARTX_IRQ, callback);
        irq_set_enabled(UARTX_IRQ, true);
        uart_set_irq_enables(uartId, true, false);
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
                services::LoggerService::error("_uartFlushWithTimeout", "TIMEOUT");
                return;
            }
        }
        return;
    }

    /* ==================================
          start Task and Methodwrapper
       ================================== */

    void MotionController::_lineFollerTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_lineFollowerTask();
    }

    void MotionController::_startLineFollowerTask()
    {
        if (xTaskCreate(_lineFollerTaskWrapper,
                        LINEFOLLOWERTASK_NAME,
                        LINEFOLLOWERTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + LINEFOLLOWERTASK_PRIORITY,
                        &_lineFollowerTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_raspberryComTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_raspberryHatComTask();
    }

    void MotionController::_startRaspberryHatComTask()
    {
        if (xTaskCreate(_raspberryComTaskWrapper,
                        RASPBERRYHATCOMTASK_NAME,
                        RASPBERRYHATCOMTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + RASPBERRYHATCOMTASK_PRIORITY,
                        &_raspberryComTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_gripControllerComTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_gripControllerComTask();
    }

    void MotionController::_startGripControllerComTask()
    {
        if (xTaskCreate(_gripControllerComTaskWrapper,
                        GRIPCONTROLLERCOMTASK_NAME,
                        GRIPCONTROLLERCOMTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + GRIPCONTROLLERCOMTASK_PRIORITY,
                        &_gripControllerTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_messageDispatcherTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_messageDispatcherTask();
    }

    void MotionController::_startMessageDispatcherTask()
    {
        if (xTaskCreate(_messageDispatcherTaskWrapper,
                        MESSAGEDISPATCHERTASK_NAME,
                        MESSAGEDISPATCHERTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + MESSAGEDISPATCHERTASK_PRIORITY,
                        &_messageDispatcherTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_barrierHandlerTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_barrierHandlerTask();
    }

    void MotionController::_startBarrierHandlerTask()
    {
        if (xTaskCreate(_barrierHandlerTaskWrapper,
                        BARRIERHANDLERTASK_NAME,
                        BARRIERHANDLERTASK_STACKSIZE / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + BARRIERHANDLERTASK_PRIORITY,
                        &_barrierHandlerTaskHandle) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::_safetyButtonPollTaskWrapper(void *pvParameters)
    {
        MotionController *obj = static_cast<MotionController *>(pvParameters);
        obj->_safetyButtonPollTask();
    }

    void MotionController::_startsafetyButtonPollTask()
    {
        if (xTaskCreate(_safetyButtonPollTaskWrapper,
                        "safetyButton Task wrapper",
                        1024 / sizeof(StackType_t),
                        this,
                        tskIDLE_PRIORITY + 4,
                        NULL) != pdTRUE)
        {
            /* ERROR HANDLING ??? */
        }
    }

    void MotionController::startTasks()
    {
        _startLineFollowerTask();
        _startMessageDispatcherTask();
        _startRaspberryHatComTask();
        _startBarrierHandlerTask();
        _startsafetyButtonPollTask();

#if INCLUDE_GRIPCONTROLLER_AS_INSTANCE == (0)
        _startGripControllerComTask();
#endif
    }

    QueueHandle_t MotionController::getMessageDispatcherQueue()
    {
        return _messageDispatcherQueue;
    }

    EventGroupHandle_t MotionController::getSafetyButtonEventHandle()
    {
        return _safetyButtonPressed;
    }

#if INCLUDE_GRIPCONTROLLER_AS_INSTANCE
    void MotionController::registerGripControllerInstance(QueueHandle_t gripControllerQueue)
    {
        _gripControllerComQueue = gripControllerQueue;
    }
#endif

    void MotionController::_safetyButtonPollTask()
    {
        TickType_t lastWakeTime = xTaskGetTickCount();
        bool lastState = true;
        bool state = false;

        for (;;)
        {
            state = _safetyButton.getValue();
            /* NOT logic - Tasks wait until event got set */
            if (!state)
            {
                xEventGroupSetBits(_safetyButtonPressed, EMERGENCY_STOP_BIT);

                if (lastState)
                {
                    services::LoggerService::info("_safetyButtonPollTask::run()", "SafetyButton Pressed! ");

                    DispatcherMessage msg = DispatcherMessage(
                        DispatcherTaskId::NoTask,
                        DispatcherTaskId::Broadcast,
                        TaskCommand::Stop,
                        0);
                    if (xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY) != pdPASS)
                    { /* ERROR!!?? */
                        while (1)
                        {
                            services::LoggerService::fatal("_safetyButtonPollTask::run() STOP EVENT TIMEOUT", "_messagDispatcherQueue TIMEOUT");
                        }
                    }

                    msg = DispatcherMessage(
                        DispatcherTaskId::NoTask,
                        DispatcherTaskId::RaspberryHatComTask,
                        TaskCommand::SafetyButtonInfo,
                        0);
                    if (xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY) != pdPASS)
                    { /* ERROR!!?? */
                        while (1)
                        {
                            services::LoggerService::fatal("_safetyButtonPollTask::run() STOP EVENT TIMEOUT", "_messagDispatcherQueue TIMEOUT");
                        }
                    }

                    taskYIELD();
                }
            }
            else
            {
                xEventGroupClearBits(_safetyButtonPressed, EMERGENCY_STOP_BIT);
            }

            lastState = state;

            vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
        }
    }
}