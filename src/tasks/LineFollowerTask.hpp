#ifndef LINE_FOLLOWER_TASK_H
#define LINE_FOLLOWER_TASK_H

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

#include "FreeRTOS.h"
#include "queue.h"

class LineFollowerTask
{
private:
    // singleton
    LineFollowerTask(QueueHandle_t *dispatcherQueue);
    static LineFollowerTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue;
    static QueueHandle_t _lineFollowerQueue;

    static void _followLine(Tmc5240 *Driver0, Tmc5240 *Driver1, LineSensor *lineSensor, uint32_t *flags);
    static int32_t _controllerC(int8_t e);
    static void _stopDrives(Tmc5240 *Driver0, Tmc5240 *Driver1, uint32_t *flags);
    static void _run(void *pvParameters);

public:
    ~LineFollowerTask();
    // singleton
    static LineFollowerTask getInstance(QueueHandle_t *dispatcherQueue);
    static QueueHandle_t getQueue();
    static TaskHandle_t getTaskHandle();
};

#endif // LINE_FOLLOWER_TASK_H
