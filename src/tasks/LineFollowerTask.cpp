#include "MotionController.hpp"

#include <stdio.h>
#include "pico/stdlib.h"

#include "hardware/uart.h"

#include "FreeRTOS.h"
#include "queue.h"

#include "TMC5240_HW_Abstraction.h"

#include "LineFollowerTaskConfig.h"
#include "Tmc5240Config.h"

#include <cmath>

namespace MotionController
{

    /* ================================= */
    /*           status Flags            */
    /* ================================= */

    constexpr uint32_t RUNMODEFLAG_T_UPPER_BITMASK(0xFFFF0000);
    constexpr uint32_t RUNMODEFLAG_T_LOWER_BITMASK(0x0000FFFF);

    enum RunModeFlag_t
    {
        // lower 16 bits statemaschine relevant flags
        MOTOR_RUNNING = 1 << 0,
        MOTOR_POSITIONMODE = 1 << 1,
        MOTOR_POSITIONMODE_REQUEST_SEND = 1 << 2,
        MOTOR_STOPREQUEST_SEND = 1 << 3,
        RUNMODE_SLOW = 1 << 4,
        LINE_FOLLOWER_MODE = 1 << 5,
        TURN_MODE = 1 << 6,
        TURNREQUEST_SEND = 1 << 7,
        STATUSFLAGS_SEND = 1 << 8,

        // upper 16 bits events and infos
        CROSSPOINT_DETECTED = 1 << 15,
        LOST_LINE = 1 << 16,
        POSITION_REACHED = 1 << 17,
        SAFETY_BUTTON_PRESSED = 1 << 18
    } RunModeFlag_t;

    constexpr uint32_t STM_LINEFOLLOWER_BITSET = 0 | (MOTOR_RUNNING | LINE_FOLLOWER_MODE);
    constexpr uint32_t STM_MOVE_POSITIONMODE_BITSET = 0 | (MOTOR_RUNNING | MOTOR_POSITIONMODE);
    constexpr uint32_t STM_STOPMOTOR_BITSET = 0;
    constexpr uint32_t STM_TURNROBOT_BITSET = 0 | (TURN_MODE | MOTOR_RUNNING);

    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    void MotionController::_lineFollowerTask()
    {
        // Variables
        int32_t X_ACTUAL_startValueDriver0 = _driver0.getXActual();
        int32_t drivenDistanceDriver0 = 0;

        int32_t X_ACTUAL_startValueDriver1 = _driver1.getXActual();
        int32_t drivenDistanceDriver1 = 0;

        uint32_t maxDistance = 0;

        // loop forever
        for (;;)
        {
            DispatcherMessage message(DispatcherTaskId::NoTask, DispatcherTaskId::NoTask, TaskCommand::NoCommand);
            if (uxQueueMessagesWaiting(_lineFollowerQueue) > 0)
            {
                xQueueReceive(_lineFollowerQueue, &message, pdMS_TO_TICKS(10));

                if (message.receiverTaskId != DispatcherTaskId::LineFollowerTask)
                {
                    printf("LINEFOLLOWERTASK - Message contains wrong Task ID \n");
                    continue;
                }

                switch (message.command)
                {
                case TaskCommand::Move:
                    _lineFollowerStatusFlags &= ~POSITION_REACHED;

                    X_ACTUAL_startValueDriver0 = _driver0.getXActual();
                    X_ACTUAL_startValueDriver1 = _driver1.getXActual();

                    // move infinit as Line Follower
                    if (message.data == 0)
                    {
                        maxDistance = 0; // TODO: 2m in usteps
                        _lineFollowerStatusFlags = STM_LINEFOLLOWER_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    }

                    // move in Position Mode
                    if (message.data != 0)
                    {
                        maxDistance = Tmc5240::meterToUStepsConversion(message.data) / 1e2; // 1e2 due to distance gets send in cm to avoid floats in protocoll
                        _lineFollowerStatusFlags = STM_MOVE_POSITIONMODE_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    }
                    break;

                case TaskCommand::Stop:
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    break;

                case TaskCommand::Turn:
                    _lineFollowerStatusFlags &= ~POSITION_REACHED;
                    _lineFollowerStatusFlags &= ~TURNREQUEST_SEND;
                    _lineFollowerStatusFlags = STM_TURNROBOT_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    break;

                case TaskCommand::PollDistance:
                    // TODO
                    break;

                case TaskCommand::PollLineSensor:
                    // TODO
                    break;

                case TaskCommand::PollStatusFlags:
                    DispatcherMessage response(DispatcherTaskId::LineFollowerTask,
                                               message.senderTaskId,
                                               TaskCommand::PollStatusFlags,
                                               _lineFollowerStatusFlags);
                    xQueueSend(_messageDispatcherQueue, &response, 0);
                    break;

                default:
                    // error! command not found assert() ?
                    break;
                }
            } // end of message handling

            /// ------- stm check safetybutton -------
            if (!_safetyButton.getValue())
            {
                _lineFollowerStatusFlags &= ~MOTOR_RUNNING;
                _lineFollowerStatusFlags | SAFETY_BUTTON_PRESSED;
            }
            else
            {
                _lineFollowerStatusFlags &= ~SAFETY_BUTTON_PRESSED;
            }

            // ------- stm check hcsr04 distance -------

            // TODO: check distance

            // ------- stm line follower -------
            if ((_lineFollowerStatusFlags & STM_LINEFOLLOWER_BITSET) == STM_LINEFOLLOWER_BITSET)
            {
                _followLine();
                drivenDistanceDriver0 = _driver0.getXActual() - X_ACTUAL_startValueDriver0;
                drivenDistanceDriver1 = _driver1.getXActual() - X_ACTUAL_startValueDriver1;

                // maxdistance reached - send Info to RaspberryHAT
                if ((maxDistance != 0 && drivenDistanceDriver0 > maxDistance) ||
                    (maxDistance != 0 && drivenDistanceDriver1 > maxDistance))
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            // ------- stm move Positionmode -------
            if ((_lineFollowerStatusFlags & STM_MOVE_POSITIONMODE_BITSET) == STM_MOVE_POSITIONMODE_BITSET)
            {
                if (!(_lineFollowerStatusFlags & MOTOR_POSITIONMODE_REQUEST_SEND))
                {
                    _lineFollowerStatusFlags |= MOTOR_POSITIONMODE_REQUEST_SEND;
                    _movePositionMode(message.data);
                }

                if (_checkForStandstill())
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            /// ------- stm turn vehicle -------
            if ((_lineFollowerStatusFlags & STM_TURNROBOT_BITSET) == STM_TURNROBOT_BITSET)
            {
                _turnRobot(message.data);

                if (_checkForStandstill())
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            /// ------- stm check and send info flags -------
            if ((_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK) && !(_lineFollowerStatusFlags & STATUSFLAGS_SEND))
            {
                DispatcherMessage response(DispatcherTaskId::LineFollowerTask,
                                           DispatcherTaskId::RaspberryHatComTask,
                                           TaskCommand::Info,
                                           _lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);

                _lineFollowerStatusFlags |= STATUSFLAGS_SEND;
                xQueueSend(_messageDispatcherQueue, &response, 0);
            }

            /// ------- stm stop drives -------
            if (!(_lineFollowerStatusFlags & MOTOR_RUNNING))
            {
                _stopDrives();
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    bool MotionController::_checkForStandstill()
    {
        uint8_t status_driver0 = _driver0.getStatus();
        uint8_t status_driver1 = _driver1.getStatus();

        if ((status_driver0 & TMC5240_SPI_STATUS_POSITION_REACHED_MASK) &&
            (status_driver1 & TMC5240_SPI_STATUS_POSITION_REACHED_MASK))
        {
            return true;
        }
        return false;
    }

    /* ================================= */
    /*           Drive Control           */
    /* ================================= */

    void MotionController::_movePositionMode(int32_t distance)
    {
        _driver0.moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    }

    void MotionController::_turnRobot(int32_t angle)
    {
        // check if turn signal got send already
        if (_lineFollowerStatusFlags & TURNREQUEST_SEND)
        {
            return;
        }
        // printf("Stack high water mark: %u\n", uxTaskGetStackHighWaterMark(_taskHandle));

        // Tmc5240::degreeToUStepsConversion(angle) / 10; // divide by 10 due to unit conversion - angle gets send in (° * 10) to avoid float
        // do some math - get microsteps
        double dAngle = static_cast<double>(angle);
        double rawVal = (MICROSTEPS_PER_REVOLUTION / 3600.0f) * dAngle; // magic number - steps per revolution divided by 360°

        // int32_t nStepsDriver = 25599;
        int32_t nStepsDriver = static_cast<int32_t>(std::round(rawVal));

        // move drives in different directions
        _driver0.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _lineFollowerStatusFlags |= TURNREQUEST_SEND;
    }

    void MotionController::_stopDrives()
    {
        if (_lineFollowerStatusFlags & MOTOR_STOPREQUEST_SEND)
        {
            return;
        }
        _driver0.moveVelocityMode(1, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveVelocityMode(0, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _lineFollowerStatusFlags |= MOTOR_STOPREQUEST_SEND;

        return;
    } // end stop Drives

    /* ================================= */
    /*           Controller              */
    /* ================================= */

    void MotionController::_followLine()
    {
        // init vars
        bool speedMode = (_lineFollowerStatusFlags & RUNMODE_SLOW) ? true : false;

        int32_t v1 = 0;
        int32_t v2 = 0;

        // get Sensor values
#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
        int8_t y = lineSensor.getLinePosition();
#else
        uint16_t y = _lineSensor.getLinePositionAnalog();
#endif

        // check for LineSensor events
        if (_lineSensor.getStatus() & LINESENSOR_CROSS_DETECTED)
        {
            _lineFollowerStatusFlags |= CROSSPOINT_DETECTED;
            _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);

            printf("LINESENSOR detected Crosspoint \n");
            return;
        }
        if (_lineSensor.getStatus() & LINESENSOR_NO_LINE)
        {
            _lineFollowerStatusFlags |= LOST_LINE;
            _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);

            printf("LINESENSOR lost Line \n");
            return;
        }

        // TODO: HCSR04 get distance

        // get error
        int8_t e = 0;

#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
        e = LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL - y;
#else
        e = LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG - y;
#endif

        // calc control variable
        int32_t u = 0;
        u = _controllerC(e);

        // set Motor values (Process P)
        if (!speedMode)
        {
            v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST + u;
            v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST - u;
        }
        else
        {
            v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW + u;
            v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW - u;
        }

        // set Motor values
        _driver0.moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    } // end followLine

    int32_t MotionController::_controllerC(int8_t e)
    {
#if LINEFOLLERCONFIG_USE_P_CONTROLLER == 1
        // P-Type Controller
        int32_t u = 0; // default 0
        u = e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP;
#endif
        int32_t retVal = u * LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P;
        return retVal;
    } // end Control

}