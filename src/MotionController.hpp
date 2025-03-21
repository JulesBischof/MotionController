#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"
#include "DigitalInput.hpp"
#include "Tla2528.hpp"

class MotionController
{
    private:

        bool _initHardware();
        bool _initPeripherals();
        bool _initQueues();

        QueueHandle_t _raspberryHatComQueue, _lineFollowerQueue, _messageDispatcherQueue;
        TaskHandle_t _raspberryComTaskHandle, _lineFollowerTaskHandle, _messageDispatcherTaskHandle;

        Tmc5240 _driver0, _driver1;
        LineSensor _lineSensor;
        Tla2528 _adc;
        DigitalInput _safetyButton;

        uint32_t _lineFollowerStatusFlags;

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

        // raspberryHatCom relevant members
        void _initUartIsr(uart_inst_t uartId);
        dispatcherMessage_t _getCommand(uart_inst_t uartId);
        bool sendUartMsg(uart_inst_t uartId);
        bool _uartFlushTxWithTimeout(uart_inst_t uartId);
        static void _uart0RxIrqHandler();
        static void _uart1RxIrqHandler();

    public:
        MotionController();
        ~MotionController();

        void startScheduler();
};

#endif // MOTIONCONTROLLER_H