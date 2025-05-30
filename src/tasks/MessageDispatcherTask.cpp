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

                if ((message.receiverTaskId & DispatcherTaskId::LineFollowerTask) == DispatcherTaskId::LineFollowerTask)
                {
                    if (xQueueSend(lineFollowerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::fatal("messageDispatcherTask", "writing to lineFollowerTaskQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                }

                if ((message.receiverTaskId & DispatcherTaskId::RaspberryHatComTask) == DispatcherTaskId::RaspberryHatComTask)
                {
                    if (xQueueSend(raspberryHatComQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::fatal("messageDispatcherTask", "writing to raspberryHatComQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                }

                if ((message.receiverTaskId & DispatcherTaskId::GripControllerComTask) == DispatcherTaskId::GripControllerComTask)
                {
                    if (xQueueSend(gripControllercomQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::fatal("messageDispatcherTask", "writing to gripControllercomQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                }

                if ((message.receiverTaskId & DispatcherTaskId::BarrierHandlerTask) == DispatcherTaskId::BarrierHandlerTask)
                {
                    if (xQueueSend(barrierHandlerQueue, &message, pdMS_TO_TICKS(1000)) != pdTRUE)
                    {
                        services::LoggerService::fatal("messageDispatcherTask", "writing to barrierHandlerQueue");
                        while (1)
                        {
                        } /* ERROR */
                    }
                }
            } // end Queue msg handling
        }
    }
}