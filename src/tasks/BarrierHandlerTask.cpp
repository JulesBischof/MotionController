#include "MotionController.hpp"

namespace MtnCtrl
{
    void MotionController::_barrierHandlerTask()
    {
        // vars
        uint32_t statusFlags = 0;
        TickType_t xLastWakeTime = xTaskGetTickCount();

        // get queues
        QueueHandle_t barrierHandlerTaskQueue = _barrierHandlerQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;

        // init stm
        stm::HandleBarrierStm _handleBarrierStm(&statusFlags, _hcSr04, &_lineSensor, messageDispatcherQueue);

        _handleBarrierStm.init();

        // init members
        _hcSr04->initMeasurmentTask();

        // loop forever
        for (;;)
        {
            DispatcherMessage message;
            DispatcherMessage response;

            // check on if some new command got in
            if (uxQueueMessagesWaiting(barrierHandlerTaskQueue) > 0)
            {
                xQueueReceive(barrierHandlerTaskQueue, &message, 0);

                if (message.receiverTaskId != DispatcherTaskId::BarrierHandlerTask)
                {
                    services::LoggerService::error("_barrierHandlerTask", "wrong TaskId: %x", message.receiverTaskId);
                    continue;
                }

                switch (message.command)
                {
                case TaskCommand::Move:
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::PositionReached:
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::Stop:
                    _handleBarrierStm.reset();
                    break;

                case TaskCommand::GcAck:
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::PollUltrasonic:
                    response = DispatcherMessage(DispatcherTaskId::BarrierHandlerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _hcSr04->getSensorData());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(1000));
                    break;
                }
            }

            // otherwise - run stm Object
            _handleBarrierStm.run();

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        }
    }
}
