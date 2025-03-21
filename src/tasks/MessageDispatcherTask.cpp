#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"

namespace MotionController
{

    void MotionController::_messageDispatcherTask()
    {
        // loop forever
        for (;;)
        {
            DispatcherMessage message(DispatcherTaskId::NoTask, DispatcherTaskId::RaspberryHatComTask, TaskCommand::NoCommand);

            // suspend task until something is waiting in Queue
            if (xQueueReceive(_messageDispatcherQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                // take messages and put them to other Queues ... (if possible)
                switch (message.receiverTaskId)
                {
                case (DispatcherTaskId::DispatcherTask):
                    // shouldn't happen - error Handling..? send ERRORCODE to RaspberryHat
                    break;
                case (DispatcherTaskId::LineFollowerTask):
                    xQueueSend(_lineFollowerQueue, &message, 0);
                    break;
                case (DispatcherTaskId::RaspberryHatComTask):
                    // xQueueSend(_raspberryHatComQueue, &message, 0);
                    break;
                case (DispatcherTaskId::GripControllerComTask):
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

}