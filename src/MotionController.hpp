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

namespace nMotionController
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

        Tmc5240 _driver0, _driver1;
        LineSensor _lineSensor;
        Tla2528 _adc;
        DigitalInput _safetyButton;
        DigitalOutput _tmc5240Eval_R2, _tmc5240Eval_R3;

        HcSr04 *_hcSr04;

        void _lineFollowerTask();
        void _raspberryHatComTask();
        void _messageDispatcherTask();

        void _startLineFollowerTask();
        void _startRaspberryHatComTask();
        void _startMessageDispatcherTask();

        static void _LineFollerTaskWrapper(void *pvParameters);
        static void _RaspberryComTaskWrapper(void *pvParameters);
        static void _MessageDispatcherTaskWrapper(void *pvParameters);

        // lineFollower relevant members
        int32_t _getRotationRelativeToStart();
        int32_t _getDrivenDistance(int32_t drivenDistanceDirver0, int32_t drivenDistanceDirver1);
        HandleBarrierStm _handleBarrierStm;
        CheckSafetyButtonStm _checkSafetyButtonStm;
        LineFollowerStm _lineFollowerStm;
        MovePositionModeStm _movePositionModeStm;
        SendStatusFlagsStm _sendStatusFlagsStm;

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