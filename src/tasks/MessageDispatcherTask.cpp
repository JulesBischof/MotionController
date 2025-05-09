#include "MotionController.hpp"

#include "MessageDispatcherTaskConfig.hpp"

namespace MtnCtrl
{
    void MotionController::_messageDispatcherTask()
    {
        // get QueueHandles
        QueueHandle_t lineFollowerQueue = _lineFollowerQueue;
        QueueHandle_t raspberryHatComQueue = _raspberryHatComQueue;
        QueueHandle_t gripControllercomQueue = _gripControllerComQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;
        QueueHandle_t barrierHandlerQueue = _barrierHandlerQueue;

        // loop forever
        for (;;)
        {
            DispatcherMessage message;

            // suspend task until something is waiting in Queue
            if (xQueueReceive(messageDispatcherQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                services::LoggerService::debug("MessageDispatcher", "Recieved Command: %x # data: %d", message.command ,message.getData());

                // take messages and put them to other Queues ... (if possible)
                switch (message.receiverTaskId)
                {
                case (DispatcherTaskId::DispatcherTask):
                    // shouldn't happen - error Handling..? send ERRORCODE to RaspberryHat
                    break;

                case (DispatcherTaskId::LineFollowerTask):
                    if (xQueueSend(lineFollowerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to lineFollowerTaskQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    break;

                case (DispatcherTaskId::RaspberryHatComTask):
                    if (xQueueSend(raspberryHatComQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to raspberryHatComQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    break;

                case (DispatcherTaskId::GripControllerComTask):
                    message.receiverTaskId = DispatcherTaskId::ServoDriveTask;
                    if (xQueueSend(gripControllercomQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to gripControllercomQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    break;

                case (DispatcherTaskId::BarrierHandlerTask):
                    if (xQueueSend(barrierHandlerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to barrierHandlerQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    break;

                case (DispatcherTaskId::Broadcast):
                    if (xQueueSend(lineFollowerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to lineFollowerTaskQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    if (xQueueSend(barrierHandlerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to barrierHandlerQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    if (xQueueSend(raspberryHatComQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to raspberryHatComQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                    if (xQueueSend(gripControllercomQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::error("messageDispatcherTask", "writing to gripControllercomQueue");
                        // while (1)
                        // {
                        // } /* ERROR */
                    }
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