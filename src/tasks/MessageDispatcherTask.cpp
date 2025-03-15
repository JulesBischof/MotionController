#include "MessageDispatcherTask.hpp"

#include "MessageDispatcherTaskConfig.h"

/// @brief creates instance of MessageDispatcherTask - private due to singleton pattern
/// @param lineFollowerQueue QueueHandle of LineFollowerTask
/// @param raspberryComQueue QueueHandle of RaspberryComTask
/// @param gripControllerComQueue QueueHandle of GripControllerComQueue
MessageDispatcherTask::MessageDispatcherTask(QueueHandle_t *lineFollowerQueue, QueueHandle_t *raspberryComQueue, QueueHandle_t *gripControllerComQueue)
{
    _lineFollowerQueue = *lineFollowerQueue;
    _raspberryHatComQueue = *raspberryComQueue;
    _gripControllerComQueue = *gripControllerComQueue;

    _dispatcherQueue = xQueueCreate(MESSAGEDISPATCHERTASKCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));

    _statusFlags = 0;

    xTaskCreate(_run, MESSAGEDISPATCHERTASK_NAME, MESSAGEDISPATCHERTASK_STACKSIZE, this, MESSAGEDISPATCHERTASK_PRIORITY, &_taskHandle);
} // end ctor

/// @brief MessageDispatcherTask loop
/// @param pvParameters void pointer param - contains MessageDispatcherTask instance
void MessageDispatcherTask::_run(void *pvParameters)
{
    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(_dispatcherQueue) > 0)
        {
            dispatcherMessage_t message;
            xQueueReceive(_dispatcherQueue, &message, pdMS_TO_TICKS(10));

            // take messages and put them somewhere else
            switch (message.recieverTaskId)
            {
            case (TASKID_DISPATCHER_TASK):
                // shouldn't happen - error Handling..? send ERRORCODE to RaspberryHat
                break;
            case (TASKID_LINE_FOLLOWER_TASK):
                xQueueSend(_lineFollowerQueue, &message, 0);
                break;
            case (TASKID_RASPBERRY_HAT_COM_TASK):
                xQueueSend(_raspberryHatComQueue, &message, 0);
                break;
            case (TASKID_GRIPCONTROLLER_COM_TASK):
                xQueueSend(_gripControllerComQueue, &message, 0);
                break;
            default:
                break;
            }
        }
    }
}

/// @brief deconstructot
MessageDispatcherTask::~MessageDispatcherTask()
{
    if (_instance != nullptr)
    {
        delete _instance;
    }
}; // end dctor

/* ================================= */
/*              getters              */
/* ================================= */

/// @brief signleton - creates a new instance of MessageDispatcherTask - if not already existing
/// @param lineFollowerQueue QueueHandle of LineFollowerTask
/// @param raspberryComQueue QueueHandle of RaspberryComTask
/// @param gripControllerComQueue QueueHandle of GripControllerComQueue
/// @return
MessageDispatcherTask MessageDispatcherTask::getInstance(QueueHandle_t *lineFollowerQueue, QueueHandle_t *raspberryComQueue, QueueHandle_t *gripControllerComQueue)
{
    if (_instance == nullptr)
    {
        _instance = new MessageDispatcherTask(lineFollowerQueue, raspberryComQueue, gripControllerComQueue);
    }
    return *_instance;
}

/// @brief getter to MessageDispatcherTask queue
/// @return QueueHandle to MessageDispatcherTask queue
QueueHandle_t MessageDispatcherTask::getQueue()
{
    return _dispatcherQueue;
}

/// @brief getter to MessageDispatcherTask Handle
/// @return TaskHandle MessageDispatcherTask
TaskHandle_t MessageDispatcherTask::getTaskHandle()
{
    return _taskHandle;
}
