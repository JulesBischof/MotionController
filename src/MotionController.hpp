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

#include "prain_uart/protocol.hpp"
using namespace prain_uart;

namespace MotionController
{

    /* ==================================
                typedefs
       ================================== */

    enum class DispatcherTaskId : uint8_t
    {
        NoTask,
        DispatcherTask,
        LineFollowerTask,
        RaspberryHatComTask,
        GripControllerComTask
    };

    enum class TaskCommand : uint8_t
    {
        NoCommand,
        Move,
        Reverse,
        Turn,
        Stop,
        Info,
        Ping,
        Pong,
        Error,
        PollDistance,
        PollLineSensor,
        PollDegree,
        PollStatusFlags,
        HandThroughMessage,
        DecodeMessage
    };

    /// @brief struct containing Messages for intertask communication
    /// data is split into 2 32bit values due to memory allignment 
    /// difficulties in the beginnning. Defining one uint64_t for data 
    /// leads to hardfaults
    struct DispatcherMessage
    {
        DispatcherTaskId senderTaskId;
        DispatcherTaskId receiverTaskId;
        TaskCommand command;
        alignas(8) uint32_t data[2];

        DispatcherMessage()
            : senderTaskId(DispatcherTaskId::NoTask),
              receiverTaskId(DispatcherTaskId::NoTask),
              command(TaskCommand::NoCommand),
              data{0, 0}
        {
        }
        DispatcherMessage(DispatcherTaskId sender, DispatcherTaskId receiver, TaskCommand cmd, uint64_t d)
            : senderTaskId(sender), receiverTaskId(receiver), command(cmd) { setData(d); }

        uint64_t getData() const { return ((uint64_t)data[1] << 32) | data[0]; }

        void setData(uint64_t d)
        {
            data[0] = (uint32_t)(d & 0xFFFFFFFF);
            data[1] = (uint32_t)((d >> 32) & 0xFFFFFFFF);
        }
    };

    /* ==================================
              class declaration
       ================================== */

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
        uint32_t _lineFollowerStatusFlags;

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
        bool _checkForStandstill();
        void _movePositionMode(int32_t distance);
        void _followLine();
        void _turnRobot(int32_t angle);
        int32_t _controllerC(int8_t e);
        void _stopDrives();
        int32_t _getRotationRelativeToStart();
        int32_t _getDrivenDistance(int32_t drivenDistanceDirver0, int32_t drivenDistanceDirver1);
        void _handleBarrier(float distance);

        // raspberryHatCom relevant members
        DispatcherMessage _getCommand(uart_inst_t *uartId);
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