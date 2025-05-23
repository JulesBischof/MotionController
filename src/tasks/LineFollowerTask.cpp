#include "MotionController.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "LineFollowerTaskConfig.hpp"
#include "LineSensor.hpp"
#include "StepperService.hpp"
#include "MovementTracker.hpp"
#include "MedianStack.hpp"
#include "CheckForLineStm.hpp"

namespace MtnCtrl
{
    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
        // init stms
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

#if USE_CHECKFORLINE_STM == (1)
        stm::CheckForLineStm _checkForLineStm = stm::CheckForLineStm(&_lineSensor, _messageDispatcherQueue);
#endif

        // init misc members
        MovementTracker _movementTracker(&_driver0, &_driver1);

        // get Queues - access is atomic, no protection neccesary
        QueueHandle_t lineFollowerQueue = _lineFollowerQueue;
        QueueHandle_t messageDispatcherQueue = _messageDispatcherQueue;

        TickType_t xLastWakeTime = xTaskGetTickCount();

        DispatcherMessage message;

        // loop forever
        for (;;)
        {
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LINEFOLLOWERCONFIG_POLLING_RATE_MS));

            /* -------- check safety button ---------- */
            EventBits_t safetyButtonBits = xEventGroupWaitBits(
                _safetyButtonPressed,
                EMERGENCY_STOP_BIT,
                pdFALSE,
                pdTRUE,
                0);

            if (uxQueueMessagesWaiting(lineFollowerQueue) > 0)
            {
                xQueueReceive(lineFollowerQueue, &message, 0);

                if (message.receiverTaskId != DispatcherTaskId::LineFollowerTask &&
                    message.receiverTaskId != DispatcherTaskId::Broadcast)
                {
                    services::LoggerService::error("_lineFollowerTask", "wrong TaskId: %x", message.receiverTaskId);
                    continue;
                }

                // if safety button is pressed: dont react unless its a stop command
                if ((safetyButtonBits & EMERGENCY_STOP_BIT) != 0 && message.command != TaskCommand::Stop)
                {
                    continue;
                }

                switch (message.command)
                {
                case TaskCommand::Move:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: MOVE # data: %d", message.getData());
                    _lineFollowerStm.update(message.getData(), message.command);
                    _movePositionModeStm.update(message.getData(), message.command);
                }
                break;

                case TaskCommand::CalibLineSensor:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: CALIBLINESENSOR TILE# data: %d", message.getData());
                    _lineSensor.lineSensorCalib(message.getData() > 0);
                }
                break;

                case TaskCommand::SlowDown:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: SLOW DOWN # data: %d", message.getData());
                    _lineFollowerStm.update(message.getData(), message.command);
                }
                break;

                case TaskCommand::Stop:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: STOP # data: %d", message.getData());
                    _lineFollowerStm.reset();
                    _movePositionModeStm.update(message.getData(), message.command);
#if USE_CHECKFORLINE_STM == (1)
                    _checkForLineStm.reset();
#endif
                }
                break;

                case TaskCommand::Turn:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: TURN # data: %d", message.getData());
                    _movePositionModeStm.update(message.getData(), TaskCommand::Turn);
                }
                break;

                case TaskCommand::PollDistance:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL DISTANCE # data: %d", message.getData());
                    DispatcherMessage response(DispatcherTaskId::LineFollowerTask,
                                               DispatcherTaskId::RaspberryHatComTask,
                                               TaskCommand::PollDistance,
                                               _movementTracker.getDistance());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(1000));

                    _movementTracker.resetDistance();
                }
                break;

                case TaskCommand::PollLineSensor:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL LINESENSOR # data: %d", message.getData());

#if USE_CHECKFORLINE_STM == (1)
                    _checkForLineStm.update(message.command, message.getData());
#else
                    miscDevices::MedianStack<uint16_t> stack(LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS);
                    _lineSensor.toggleUvLed(true);
                    bool lostFlag = false;
                    for (uint8_t i = 0; i < LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS; i++)
                    {
                        stack.push(_lineSensor.getLinePositionAnalog());
                        // check for Line
                        if (_lineSensor.getStatus() & miscDevices::LINESENSOR_NO_LINE)
                        {
                            lostFlag = true;
                            services::LoggerService::info("LineFollowerTask", "Lost Line");
                            break;
                        }
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    _lineSensor.toggleUvLed(false);

                    if (lostFlag)
                    {
                        response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                     message.senderTaskId,
                                                     TaskCommand::LostLineInfo,
                                                     0);
                    }
                    else
                    {
                        response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                     message.senderTaskId,
                                                     TaskCommand::PollLineSensor,
                                                     stack.getMedian());
                    }
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(100));
#endif
                }
                break;

                case TaskCommand::PositionReached:
#if USE_CHECKFORLINE_STM == (1)
                    _checkForLineStm.update(message.command, message.getData());
#endif
                    break;

                case TaskCommand::PollDegree:
                {
                    services::LoggerService::debug("LineFollowerTask", "Recieved Command: POLL DEGREE # data: %d", message.getData());
                    DispatcherMessage response(DispatcherTaskId::LineFollowerTask,
                                              message.senderTaskId,
                                              TaskCommand::PollDegree,
                                              _movementTracker.getRotation());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(1000));
                    }
                    break;

                default:
                    services::LoggerService::error("_lineFollowerTask", "command unknown: %x", message.command);
                    break;
                }
            } // end of message handling

            // run statemaschines
            _lineFollowerStm.run();
            _movePositionModeStm.run();
#if USE_CHECKFORLINE_STM == (1)
            _checkForLineStm.run();
#endif

#if TRIGGER_LINEPOS_SAMPLES == 1
            _lineSensor.toggleUvLed(true);
            uint32_t linePos = _lineSensor.getLinePositionAnalog();
            printf("%d\n", linePos);
#endif

#if TRIGGER_LINESENSOR_SAMPLES == 1
            _lineSensor.toggleUvLed(true);
            _lineSensor.getLinePositionAnalog();
#endif
        }
    } // end Task
}