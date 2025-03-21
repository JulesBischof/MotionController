#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"
#include "queues.h"

void MotionController::_messageDispatcherTask()
{
    // loop forever
    for (;;)
    {
        dispatcherMessage_t message = {};

        // suspend task until something is waiting in Queue
        if (xQueueReceive(_messageDispatcherQueue, &message, portMAX_DELAY) == pdTRUE)
        {
            // take messages and put them to other Queues ... (if possible)
            switch (message.recieverTaskId)
            {
            case (TASKID_DISPATCHER_TASK):
                // shouldn't happen - error Handling..? send ERRORCODE to RaspberryHat
                break;
            case (TASKID_LINE_FOLLOWER_TASK):
                xQueueSend(_lineFollowerQueue, &message, 0);
                break;
            case (TASKID_RASPBERRY_HAT_COM_TASK):
                // xQueueSend(_raspberryHatComQueue, &message, 0);
                break;
            case (TASKID_GRIPCONTROLLER_COM_TASK):
                // xQueueSend(_gripControllerComQueue, &message, 0);
                break;
            default:
                break;
            }
        } // end Queue msg handling

        // ERROR HANDLING IF FALSE ????

    } // end loop

    /* never reached */
}