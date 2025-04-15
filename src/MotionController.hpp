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
        bool _initHardware();
        bool _initQueues();
        void _initUart0Isr();
        void _initPeripherals();
        void _checkLineFollowerStatus();

        static QueueHandle_t _raspberryHatComQueue, _lineFollowerQueue, _messageDispatcherQueue;
        TaskHandle_t _raspberryComTaskHandle, _lineFollowerTaskHandle, _messageDispatcherTaskHandle;

        spiDevices::Tmc5240 _driver0, _driver1;
        miscDevices::LineSensor _lineSensor;
        i2cDevices::Tla2528 _adc;
        miscDevices::DigitalInput _safetyButton;
        miscDevices::DigitalOutput _tmc5240Eval_R2, _tmc5240Eval_R3;

        miscDevices::HcSr04 *_hcSr04;

        void _lineFollowerTask();
        void _raspberryHatComTask();
        void _messageDispatcherTask();

        void _startLineFollowerTask();
        void _startRaspberryHatComTask();
        void _startMessageDispatcherTask();

        static void _LineFollerTaskWrapper(void *pvParameters);
        static void _RaspberryComTaskWrapper(void *pvParameters);
        static void _MessageDispatcherTaskWrapper(void *pvParameters);

        // lineFollower members
        int32_t _getRotationRelativeToStart();
        int32_t _getDrivenDistance(int32_t drivenDistanceDirver0, int32_t drivenDistanceDirver1);
        stm::HandleBarrierStm _handleBarrierStm;
        stm::CheckSafetyButtonStm _checkSafetyButtonStm;
        stm::LineFollowerStm _lineFollowerStm;
        stm::MovePositionModeStm _movePositionModeStm;
        stm::SendStatusFlagsStm _sendStatusFlagsStm;

        // raspberryHatCom relevant members
        DispatcherMessage
        _getCommand(uart_inst_t *uartId);
        void sendUartMsg(frame *data, uart_inst_t *uartId);
        void _uartFlushTxWithTimeout(uart_inst_t *uartId, uint32_t timeout_ms);
        static void _uart0RxIrqHandler();
        // static void _uart1RxIrqHandler();

    public:
        MotionController();
        ~MotionController();

        void startScheduler();

        static QueueHandle_t getRaspberryHatComQueue();
        static QueueHandle_t getLineFollowerQueue();
        static QueueHandle_t getMessageDispatcherQueue();
    };
}

#endif // MOTIONCONTROLLER_H