#ifndef V_LINE_FOLLOWER_TASK_H
#define V_LINE_FOLLOWER_TASK_H

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

class LineFollowerTask
{
private:
    // singleton
    LineFollowerTask(QueueHandle_t *dispatcherQueue, QueueHandle_t *lineFollowerQueue);
    static LineFollowerTask *_instance;

    static TaskHandle_t _taskHandle;
    static QueueHandle_t _dispatcherQueue;
    static QueueHandle_t _lineFollowerQueue;

    static void _followLine(Tmc5240 *Driver0, Tmc5240 *Driver1, LineSensor *lineSensor, uint32_t *flags);
    static int32_t _controllerC(int8_t e);
    static void _stopDrives(Tmc5240 *Driver0, Tmc5240 *Driver1, uint32_t *flags);
    static void _vLineFollowerTask(void *pvParameters);

public:
    ~LineFollowerTask();
    // singleton
    static LineFollowerTask getInstance(QueueHandle_t *dispatcherQueue, QueueHandle_t *lineFollowerQueue);
    static QueueHandle_t getQueue();
};

#endif // V_LINE_FOLLOWER_TASK_H
