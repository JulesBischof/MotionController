#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include <cstddef>
#include <cstdint>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "HcSr04.hpp"
#include "Tmc5240.hpp"
#include "LineSensor.hpp"
#include "DigitalInput.hpp"
#include "DigitalOutput.hpp"
#include "Tla2528.hpp"

#include "DispatcherMessage.hpp"

#include "HandleBarrierStm.hpp"
#include "CheckSafetyButtonStm.hpp"
#include "LineFollowerStm.hpp"
#include "MovePositionModeStm.hpp"
#include "SendStatusFlagsStm.hpp"

#include "prain_uart/protocol.hpp"
    using namespace prain_uart;

namespace MtnCtrl
{
    class MotionController
    {
    private:
        /// @brief initializes all GPIOs and Communication Channels connected to the MotionController
        void _initHardware();

        /// @brief initializes all Queues used for intertask communication
        /// @return true if all queues got created successfully
        void _initQueues();

        void _initUartRxIsr(uart_inst_t *uartId, void (*callback)());

        /// @brief initializes all Classmembers such as sensors and drivers
        void _initPeripherals();

        static QueueHandle_t _raspberryHatComQueue, _lineFollowerQueue, _messageDispatcherQueue, _gripControllerComQueue;
        TaskHandle_t _raspberryComTaskHandle, _lineFollowerTaskHandle, _messageDispatcherTaskHandle, _gripControllerTaskHandle;

        // peripherals
        spiDevices::Tmc5240 _driver0, _driver1;
        miscDevices::LineSensor _lineSensor;
        i2cDevices::Tla2528 _adc;
        miscDevices::DigitalInput _safetyButton;
        miscDevices::DigitalOutput _tmc5240Eval_R2, _tmc5240Eval_R3;
        miscDevices::HcSr04 *_hcSr04;

        void _lineFollowerTask();
        void _raspberryHatComTask();
        void _gripControllerComTask();
        void _messageDispatcherTask();

        /// @brief main loop LineFollowerTask. Checks on Barrierdistance as  well as on linepossition and moves vehicle accordingly
        void _startLineFollowerTask();
        
        /// @brief main loop rapberryHatComTask. either waits on uart0 Rx Interrupts or waits on incoming Tx-commands
        void _startRaspberryHatComTask();

        /// @brief main loop rapberryHatComTask. either waits on uart0 Rx Interrupts or waits on incoming Tx-commands
        void _startGripControllerComTask();

        /// @brief main loop MessageDispatcherTask. Takes Messages and Routes them accordingly to the Reciever
        void _startMessageDispatcherTask();

        /// @brief FreeRTOS expects a static Functionpointer. This Wrapping helps to start a memberfunction as its own Task. 
        /// @param pvParameters MotionController Instance
        static void _lineFollerTaskWrapper(void *pvParameters);
        
        /// @brief FreeRTOS expects a static Functionpointer. This Wrapping helps to start a memberfunction as its own Task.
        /// @param pvParameters MotionController Instance
        static void _raspberryComTaskWrapper(void *pvParameters);

        /// @brief FreeRTOS expects a static Functionpointer. This Wrapping helps to start a memberfunction as its own Task.
        /// @param pvParameters MotionController Instance
        static void _gripControllerComTaskWrapper(void *pvParameters);

        /// @brief FreeRTOS expects a static Functionpointer. This Wrapping helps to start a memberfunction as its own Task.
        /// @param pvParameters MotionController Instance
        static void _messageDispatcherTaskWrapper(void *pvParameters);

        // lineFollower members
        stm::HandleBarrierStm _handleBarrierStm;
        stm::CheckSafetyButtonStm _checkSafetyButtonStm;
        stm::LineFollowerStm _lineFollowerStm;
        stm::MovePositionModeStm _movePositionModeStm;
        stm::SendStatusFlagsStm _sendStatusFlagsStm;

        /// @brief reads out UART channel and copys raw frame into a DispatcherMessage Format
        /// @param uartId uart channel - either uart0 or uart1
        /// @return dispatcherMessage for intertask communication
        DispatcherMessage _getCommand(uart_inst_t *uartId);

        /// @brief sends one package of data using the prain_uart library
        /// @param data frame of data which is supposed to be send. Needs to be pre-encoded using the encoder-class!
        /// @param uartId Uart-Channel: uart0 -> RaspberryHat, uart1-> GripController
        void sendUartMsg(frame *data, uart_inst_t *uartId);

        /// @brief Flushes Uart TxChannel after one package of data was send.
        /// @param uart uart id: uart1/uart0
        /// @param timeout_ms timeout in ms
        void _uartFlushTxWithTimeout(uart_inst_t *uartId, uint32_t timeout_ms);

        /// @brief handles incoming messages. Does not decode data but sends Decode command to RaspberryComTask
        static void _uart0RxIrqHandler();
        static void _uart1RxIrqHandler();

    public:
        /// @brief default ctor
        MotionController();
        ~MotionController();

        /// @brief Starts FreeRTOS Scheduler
        void startScheduler();

        /// @brief getter RaspberryHatComQueue
        /// @return QueueHandle for RaspberryHatComQueue
        static QueueHandle_t getRaspberryHatComQueue();

        /// @brief getter RaspberryHatComQueue
        /// @return QueueHandle for RaspberryHatComQueue
        static QueueHandle_t getGripControllerComQueue();

        /// @brief getter LineFollowerQueue
        /// @return QueueHandle for LineFollowerQueue
        static QueueHandle_t getLineFollowerQueue();

        /// @brief getter MessageDispatcherQueue
        /// @return QueueHandle for MessageDispatcherQueue
        static QueueHandle_t getMessageDispatcherQueue();
    };
}

#endif // MOTIONCONTROLLER_H