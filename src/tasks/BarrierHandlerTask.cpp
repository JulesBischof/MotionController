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
        stm::HandleBarrierStm _handleBarrierStm(_hcSr04, messageDispatcherQueue);

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

                if (message.receiverTaskId != DispatcherTaskId::BarrierHandlerTask &&
                    message.receiverTaskId != DispatcherTaskId::Broadcast)
                {
                    services::LoggerService::error("_barrierHandlerTask", "wrong TaskId: %x", message.receiverTaskId);
                    continue;
                }

                switch (message.command)
                {
                case TaskCommand::Move:
                    services::LoggerService::debug("BarrierHandlerTask", "Recieved Command: MOVE # data: %d", message.getData());
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;
                
                case TaskCommand::PositionReached:
                    services::LoggerService::debug("BarrierHandlerTask", "Recieved Command: POSITION_REACHED # data: %d", message.getData());
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::Stop:
                    services::LoggerService::debug("BarrierHandlerTask", "Recieved Command: STOP # data: %d", message.getData());
                    _handleBarrierStm.reset();
                    break;

                case TaskCommand::GcAck:
                    services::LoggerService::debug("BarrierHandlerTask", "Recieved Command: GCACK # data: %d", message.getData());
                    _handleBarrierStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::PollUltrasonic:
                    services::LoggerService::debug("BarrierHandlerTask", "Recieved Command: POLL ULTRASONIC # data: %d", message.getData());
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
