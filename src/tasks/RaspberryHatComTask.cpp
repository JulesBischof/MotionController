#include "RaspberryHatComTask.hpp"

RaspberryHatComTask::RaspberryHatComTask(QueueHandle_t *dispatcherQueue)
{
}

void RaspberryHatComTask::_run(void *pvParameters)
{
}

RaspberryHatComTask::~RaspberryHatComTask()
{
}

/* ================================= */
/*              getters              */
/* ================================= */

RaspberryHatComTask RaspberryHatComTask::getInstance(QueueHandle_t *raspberryHatComQueue)
{
    if (_instance == nullptr)
    {
        _instance = new RaspberryHatComTask(raspberryHatComQueue);
    }
    return *_instance;
}

QueueHandle_t RaspberryHatComTask::getQueue()
{
    return QueueHandle_t();
}

TaskHandle_t RaspberryHatComTask::getTaskHandle()
{
    return TaskHandle_t();
}
