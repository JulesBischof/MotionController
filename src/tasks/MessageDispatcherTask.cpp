#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.hpp"

namespace MtnCtrl
{
    void MotionController::_messageDispatcherTask()
    {
        // get QueueHandles
        QueueHandle_t lineFollowerQueue = _lineFollowerQueue;
        QueueHandle_t raspberryHatComQueue = _raspberryHatComQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;

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