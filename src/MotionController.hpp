#ifndef MOTIONCONTROLLER_H
#define MOTIONCONTROLLER_H

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "queues.h"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"
#include "DigitalInput.hpp"
#include "Tla2528.hpp"

#include "prain_uart/protocol.hpp"
using namespace prain_uart;

class MotionController
{
private:
    bool _initHardware();
    bool _initQueues();
    void _initUartIsr();

    static QueueHandle_t _raspberryHatComQueue, _lineFollowerQueue, _messageDispatcherQueue;
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
    dispatcherMessage_t _getCommand(uart_inst_t *uartId);
    void sendUartMsg(frame *data, uart_inst_t *uartId);
    void _uartFlushTxWithTimeout(uart_inst_t *uartId, uint32_t timeout_ms);
    static void _uart0RxIrqHandler();
    // static void _uart1RxIrqHandler();

public:
    MotionController();
    ~MotionController();

    void startScheduler();

    static QueueHandle_t getRaspberryHatComQueue();
};

#endif // MOTIONCONTROLLER_H