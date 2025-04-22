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
    /*            consts                 */
    /* ================================= */

    constexpr float ROTATIONS_PER_SEC = (LINEFOLLERCONFIG_VMAX_REGISTER_VALUE / (16777216.f / TMC5240CLOCKFREQUENCY) / MICROSTEPS_PER_REVOLUTION);
    constexpr float V_MAX_IN_MMPS = (LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM * M_PI * ROTATIONS_PER_SEC); // mm per second

    constexpr float ROTATIONS_PER_SEC_SQUARED = (LINEFOLLERCONFIG_AMAX_REGISTER_VALUE / (131072.f / TMC5240CLOCKFREQUENCY));
    constexpr float A_MAX_IN_MMPSS = (LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM * M_PI * ROTATIONS_PER_SEC_SQUARED) / MICROSTEPS_PER_REVOLUTION; // mm per s^2

    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
        // init stms
        uint32_t _lineFollowerStatusFlags = 0;
        stm::CheckSafetyButtonStm _checkSafetyButtonStm(&_lineFollowerStatusFlags, &_safetyButton, getLineFollowerQueue());
        _checkSafetyButtonStm.init();
        stm::LineFollowerStm _lineFollowerStm(&_lineFollowerStatusFlags, &_lineSensor, &_driver0, &_driver1, getLineFollowerQueue());
        _lineFollowerStm.init();
        stm::MovePositionModeStm _movePositionModeStm(&_lineFollowerStatusFlags, &_driver0, &_driver1);
        _movePositionModeStm.init();
        stm::HandleBarrierStm _handleBarrierStm(&_lineFollowerStatusFlags, _hcSr04, &_lineSensor, getLineFollowerQueue(), getMessageDispatcherQueue());
        _handleBarrierStm.init();
        stm::SendStatusFlagsStm _sendStatusFlagsStm(&_lineFollowerStatusFlags, getMessageDispatcherQueue());
        _sendStatusFlagsStm.init();

        // init misc members
        _hcSr04->initMeasurmentTask();
        MovementTracker _movementTracker(&_driver0, &_driver1);

        // get Queues
        QueueHandle_t lineFollowerQueue = getLineFollowerQueue();
        if (lineFollowerQueue == nullptr)
        {
            printf("ERROR #lineFollowerTask# NULLREFERENCE lineFollowerQueueHandle");
            while (1)
            { /*  ERROR  */
            }
        }
        QueueHandle_t messageDispatcherQueue = getMessageDispatcherQueue();
        if (messageDispatcherQueue == nullptr)
        {
            printf("ERROR #lineFollowerTask# NULLREFERENCE messageDispatcherQueueHandle");
            while (1)
            { /*  ERROR  */
            }
        }

        uint32_t maxDistance = 0;

        TickType_t xLastWakeTime = xTaskGetTickCount();

        // loop forever
        for (;;)
        {
            DispatcherMessage message;
            DispatcherMessage response;

            if (uxQueueMessagesWaiting(lineFollowerQueue) > 0)
            {
                xQueueReceive(lineFollowerQueue, &message, 0);

                if (message.receiverTaskId != DispatcherTaskId::LineFollowerTask)
                {
                    printf("ERROR LINEFOLLOWERTASK - Message contains wrong Task ID \n");
                    continue;
                }

                // you cant declare vars inside a switch statement, thats why dataContainer fol polling
                uint64_t dataContainer = 0;

                switch (message.command)
                {
                case TaskCommand::Move:
                    _lineFollowerStatusFlags &= ~(uint32_t)RunModeFlag::POSITION_REACHED;

                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);

                    _lineFollowerStm.update(message.getData());
                    _movePositionModeStm.update(message.getData(), message.command);
                    _handleBarrierStm.update(message.getData(), message.command);

                    break;

                case TaskCommand::Stop:
                    _lineFollowerStm.reset();
                    _movePositionModeStm.update(message.getData(), message.command);

                    break;

                case TaskCommand::Turn:
                    _movePositionModeStm.update(message.getData(), TaskCommand::Turn);
                    break;

                case TaskCommand::PollDistance:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _movementTracker.getDistance());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollLineSensor:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _lineSensor.getLinePositionAnalog());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollUltrasonic:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 _hcSr04->getSensorData());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollStatusFlags:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollStatusFlags,
                                                 _lineFollowerStatusFlags);
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollDegree:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDegree,
                                                 _movementTracker.getRotation());
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                default:
                    printf("ERROR LineFollowerTask - COMMAND UNKNOWN ?");
                    break;
                }
            } // end of message handling

            // run statemaschines
            _checkSafetyButtonStm.run();
            _handleBarrierStm.run();
            _lineFollowerStm.run();
            _movePositionModeStm.run();
            _sendStatusFlagsStm.run();

            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LINEFOLLOWERCONFIG_POLLING_RATE_MS));
        }
    } // end Task
}