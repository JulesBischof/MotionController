#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.h"

namespace MtnCtrl
{
    void MotionController::_messageDispatcherTask()
    {
        // get QueueHandles
        QueueHandle_t lineFollowerQueue = getLineFollowerQueue();
        if (lineFollowerQueue == nullptr)
        {
            printf("ERROR #messageDispatcherTask# NULLREFERENCE lineFollowerQueueHandle\n");
            while (1)
            { /*  ERROR  */
            }
        }
        QueueHandle_t raspberryHatComQueue = getRaspberryHatComQueue();
        if (raspberryHatComQueue == nullptr)
        {
            printf("ERROR #messageDispatcherTask# NULLREFERENCE raspberryHatComQueue\n");
            while (1)
            { /*  ERROR  */
            }
        }
        QueueHandle_t messageDispatcherQueue = getMessageDispatcherQueue();
        if (messageDispatcherQueue == nullptr)
        {
            printf("ERROR #messageDispatcherTask# NULLREFERENCE messageDispatcherQueue\n");
            while (1)
            { /*  ERROR  */
            }
        }

        // loop forever
        for (;;)
        {
            DispatcherMessage message;

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
                    if(xQueueSend(lineFollowerQueue, &message, pdMS_TO_TICKS(10)) != pdTRUE)
                    {
                        printf("ERROR #messageDispatcherTask# NULLREFERENCE messageDispatcher - send to LineFollowerTask\n");
                        while(1){} /* ERROR */
                    }
                    break;
                case (DispatcherTaskId::RaspberryHatComTask):
                    if (xQueueSend(_raspberryHatComQueue, &message, pdMS_TO_TICKS(10)) != pdTRUE)
                    {
                        printf("ERROR #messageDispatcherTask# NULLREFERENCE messageDispatcher - send to RaspiComTask\n");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    break;
                case (DispatcherTaskId::GripControllerComTask):
                    // xQueueSend(_gripControllerComQueue, &message, pdMS_TO_TICKS(10));
                    break;
                default:
                    break;
                }
            } // end Queue msg handling

            vTaskDelay(pdMS_TO_TICKS(10));
        } // end loop

        /* never reached */
    }

}