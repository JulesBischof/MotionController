#include "MotionController.hpp"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "task.h"

#include "LineFollowerTaskConfig.hpp"
#include "LineFollowerTaskStatusFlags.hpp"

#include "StepperService.hpp"

#include "MovementTracker.hpp"

namespace MtnCtrl
{
    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
        // init stms
        uint32_t _lineFollowerStatusFlags = 0;
        stm::LineFollowerStm _lineFollowerStm(&_lineSensor,
                                              &_driver0,
                                              &_driver1,
                                              _lineFollowerQueue,
                                              _messageDispatcherQueue);
        _lineFollowerStm.init();
        stm::MovePositionModeStm _movePositionModeStm(&_driver0,
                                                      &_driver1,
                                                      &_lineSensor,
                                                      _messageDispatcherQueue);
        _movePositionModeStm.init();

        // init misc members
        MovementTracker _movementTracker(&_driver0, &_driver1);

        // get Queues - access is atomic, no protection neccesary
        QueueHandle_t lineFollowerQueue = _lineFollowerQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;

        uint32_t maxDistance = 0;

        TickType_t xLastWakeTime = xTaskGetTickCount();

        // loop forever
        for (;;)
        {
            /* -------- check safety button ---------- */
            EventBits_t safetyButtonBits = xEventGroupWaitBits(
                _safetyButtonPressed,
                EMERGENCY_STOP_BIT,
                pdFALSE,
                pdTRUE,
                0);

            DispatcherMessage message;
            DispatcherMessage response;

            if (uxQueueMessagesWaiting(lineFollowerQueue) > 0)
            {
                xQueueReceive(lineFollowerQueue, &message, 0);

                if (message.receiverTaskId != DispatcherTaskId::LineFollowerTask &&
                    message.receiverTaskId != DispatcherTaskId::Broadcast)
                {
                    services::LoggerService::error("_lineFollowerTask", "wrong TaskId: %x", message.receiverTaskId);
                    continue;
                }

                // if safety button is pressed: all commands are stop commands
                if ((safetyButtonBits & EMERGENCY_STOP_BIT) != 0)
                {
                    message.command = TaskCommand::Stop;
                }

                switch (message.command)
                {
                case TaskCommand::Move:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: MOVE # data: %d", message.getData());
                    _lineFollowerStatusFlags &= ~(uint32_t)RunModeFlag::POSITION_REACHED;
                    _lineFollowerStm.update(message.getData(), message.command);
                    _movePositionModeStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::CalibLineSensor:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: CALIBLINESENSOR TILE# data: %d", message.getData());
                    _lineSensor.lineSensorCalib(message.getData() > 0);
                    break;

                case TaskCommand::SlowDown:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: SLOW DOWN # data: %d", message.getData());
                    _lineFollowerStatusFlags &= ~(uint32_t)RunModeFlag::POSITION_REACHED;
                    _lineFollowerStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::Stop:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: STOP # data: %d", message.getData());
                    _lineFollowerStm.reset();
                    _movePositionModeStm.update(message.getData(), message.command);
                    break;

                case TaskCommand::Turn:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: TURN # data: %d", message.getData());
                    _movePositionModeStm.update(message.getData(), TaskCommand::Turn);
                    break;

                case TaskCommand::PollDistance:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL DISTANCE # data: %d", message.getData());
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _movementTracker.getDistance());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollLineSensor:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL LINESENSOR # data: %d", message.getData());
                    _lineSensor.toggleUvLed(true);
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _lineSensor.getLinePositionAnalog());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    _lineSensor.toggleUvLed(false);
                    break;

                case TaskCommand::PollStatusFlags:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL STATUSFLAGS # data: %d", message.getData());
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollStatusFlags,
                                                 _lineFollowerStatusFlags);
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollDegree:
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL DEGREE # data: %d", message.getData());
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDegree,
                                                 _movementTracker.getRotation());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                default:
                    services::LoggerService::error("_lineFollowerTask", "command unknown: %x", message.command);
                    break;
                }
            } // end of message handling

            // run statemaschines
            _lineFollowerStm.run();
            _movePositionModeStm.run();

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LINEFOLLOWERCONFIG_POLLING_RATE_MS));
        }
    } // end Task
}