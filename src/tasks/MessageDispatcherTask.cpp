#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"

namespace MotionController
{

    void MotionController::_messageDispatcherTask()
    {
        // loop forever
        for (;;)
        {
            DispatcherMessage message;

            // get QueueHandles
            QueueHandle_t lineFollowerQueue = getLineFollowerQueue();
            if (lineFollowerQueue == nullptr)
            {
                while (1)
                { /*  ERROR  */
                }
            }
            QueueHandle_t raspberryHatComQueue = getRaspberryHatComQueue();
            if (raspberryHatComQueue == nullptr)
            {
                while (1)
                { /*  ERROR  */
                }
            }
            QueueHandle_t messageDispatcherQueue = getMessageDispatcherQueue();
            if (messageDispatcherQueue == nullptr)
            {
                while (1)
                { /*  ERROR  */
                }
            }

            // suspend task until something is waiting in Queue
            if (xQueueReceive(messageDispatcherQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                // take messages and put them to other Queues ... (if possible)
                switch (message.receiverTaskId)
                {
                case (DispatcherTaskId::DispatcherTask):
                    // shouldn't happen - error Handling..? send ERRORCODE to RaspberryHat
                    break;
                case (DispatcherTaskId::LineFollowerTask):
                    xQueueSend(lineFollowerQueue, &message, pdMS_TO_TICKS(10));
                    break;
                case (DispatcherTaskId::RaspberryHatComTask):
                    // xQueueSend(_raspberryHatComQueue, &message, pdMS_TO_TICKS(10));
                    break;
                case (DispatcherTaskId::GripControllerComTask):
                    // xQueueSend(_gripControllerComQueue, &message, pdMS_TO_TICKS(10));
                    break;
                default:
                    break;
                }
            } // end Queue msg handling

            // ERROR HANDLING IF FALSE ????
            vTaskDelay(pdMS_TO_TICKS(10));
        } // end loop

        /* never reached */
    }

}