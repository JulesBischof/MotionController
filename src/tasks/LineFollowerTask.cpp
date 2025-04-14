#include "MotionController.hpp"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "TMC5240_HW_Abstraction.h"

#include "LineFollowerTaskConfig.h"
#include "Tmc5240Config.h"

#include "LineFollowerTaskStatusFlags.hpp"

namespace nMotionController
{


    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
        // init stms
        uint32_t _lineFollowerStatusFlags = 0;
        CheckSafetyButtonStm _checkSafetyButtonStm(&_lineFollowerStatusFlags, &_safetyButton, getLineFollowerQueue());
        _checkSafetyButtonStm.init();
        LineFollowerStm _lineFollowerStm(&_lineFollowerStatusFlags, &_lineSensor, &_driver0, &_driver1, getLineFollowerQueue());
        _lineFollowerStm.init();
        MovePositionModeStm _movePositionModeStm(&_lineFollowerStatusFlags, &_driver0, &_driver1);
        _movePositionModeStm.init();
        HandleBarrierStm _handleBarrierStm(&_lineFollowerStatusFlags, _hcSr04, &_driver0, &_driver1, getLineFollowerQueue());
        _handleBarrierStm.init();
        SendStatusFlagsStm _sendStatusFlagsStm(&_lineFollowerStatusFlags, getMessageDispatcherQueue());
        _sendStatusFlagsStm.init();
        
        _hcSr04->initMeasurmentTask();

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

        // Variables
        int32_t X_ACTUAL_startValueDriver0 = _driver0.getXActual();
        int32_t drivenDistanceDriver0 = 0;

        int32_t X_ACTUAL_startValueDriver1 = _driver1.getXActual();
        int32_t drivenDistanceDriver1 = 0;

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

                    X_ACTUAL_startValueDriver0 = _driver0.getXActual();
                    X_ACTUAL_startValueDriver1 = _driver1.getXActual();

                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);

                    _lineFollowerStm.update(message.getData());
                    _movePositionModeStm.update(message.getData(), TaskCommand::Move); // 10 due to data gets send in cm but motion controller works with mm

                    break;

                case TaskCommand::Stop:
                    _lineFollowerStm.reset();
                    _movePositionModeStm.update(message.getData(), TaskCommand::Stop);
                    break;

                case TaskCommand::Turn:
                    _movePositionModeStm.update(message.getData(), TaskCommand::Turn);
                    break;

                case TaskCommand::PollDistance:
                    dataContainer = static_cast<uint64_t>(_getDrivenDistance(drivenDistanceDriver0, drivenDistanceDriver1));

                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDistance,
                                                 dataContainer);
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollLineSensor:
                    // TODO
                    break;

                case TaskCommand::PollStatusFlags:
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollStatusFlags,
                                                 _lineFollowerStatusFlags);
                    xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
                    break;

                case TaskCommand::PollDegree:
                    dataContainer = static_cast<uint64_t>(_getRotationRelativeToStart());
                    response = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                 message.senderTaskId,
                                                 TaskCommand::PollDegree,
                                                 dataContainer);
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


    /* ================================= */
    /*       getters & Conversion        */
    /* ================================= */

    /// @brief Basic implementation to get the relative rotation of the vehicle seen from startpoint
    /// @return degree * 10
    int32_t MotionController::_getRotationRelativeToStart()
    {
        int32_t xActualDriver0 = _driver0.getXActual();
        int32_t xActualDriver1 = _driver1.getXActual();

        int32_t ds = xActualDriver0 - xActualDriver1;

        float degree = Tmc5240::convertDeltaDrivenDistanceToDegree(ds);

        // times 10 due to prain_uart protocoll! ( e.g. 180° => send 1800 in protocoll )
        int32_t retVal = static_cast<int32_t>(std::round(degree * 10));
        return retVal;
    }

    /// @brief calc mean value of driven distance
    /// @param drivenDistanceDirver0 driven Distance driver0 [MICROSTEPS]
    /// @param drivenDistanceDirver1 driven Distance driver1 [MICROSTEPS]
    /// @return mean Value of driven distance
    int32_t MotionController::_getDrivenDistance(int32_t drivenDistanceDirver0, int32_t drivenDistanceDirver1)
    {
        float d1 = static_cast<float>(drivenDistanceDirver0);
        float d2 = static_cast<float>(drivenDistanceDirver1);
        int32_t res = static_cast<int32_t>(std::round((d1 + d2) / 2)); // get mean value

        int32_t distance = Tmc5240::convertMicrostepsToCentimeter(res);
        return distance;
    }
}