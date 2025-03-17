#ifndef LINE_FOLLOWER_TASK_H
#define LINE_FOLLOWER_TASK_H

#include "Tmc5240.hpp"
#include "LineSensor.hpp"
#include "DigitalInput.hpp"

#include "FreeRTOS.h"
#include "queue.h"

class LineFollowerTask
{
private:
    // singleton
    LineFollowerTask(QueueHandle_t *dispatcherQueue);
    static LineFollowerTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue, _lineFollowerQueue;

    static uint32_t _statusFlags;

    static Tmc5240 *_driver0;
    static Tmc5240 *_driver1;

    static Tla2528 *_adc;
    static LineSensor *_lineSensor;
    static DigitalInput *_safetyButton;

    static void _initDevices();

    static bool _checkForStandstill();

    static void _followLine();
    static void _turnRobot(uint32_t angle);
    static void _movePositionMode(int32_t distance);
    static int32_t _controllerC(int8_t e);
    static void _stopDrives();
    static void _run(void *pvParameters);

public:
    ~LineFollowerTask();
    // singleton
    static LineFollowerTask getInstance(QueueHandle_t *dispatcherQueue);

    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // LINE_FOLLOWER_TASK_H
