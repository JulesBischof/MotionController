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

#include <cmath>

namespace MotionController
{
    /* ================================= */
    /*           status Flags            */
    /* ================================= */

    constexpr uint32_t RUNMODEFLAG_T_UPPER_BITMASK(0xFFFF0000);
    constexpr uint32_t RUNMODEFLAG_T_LOWER_BITMASK(0x0000FFFF);

    enum RunModeFlag
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
        CROSSPOINT_DETECTED = 1 << 16,
        LOST_LINE = 1 << 17,
        POSITION_REACHED = 1 << 18,
        SAFETY_BUTTON_PRESSED = 1 << 19,
        LINEFOLLOWER_ERROR = 1 << 20,
    } RunModeFlag;

    /* TODO Flags 4 Errors - upper 16 RunModeFlags get send as Info, not as Error */

    constexpr uint32_t STM_LINEFOLLOWER_BITSET = 0 | (MOTOR_RUNNING | LINE_FOLLOWER_MODE);
    constexpr uint32_t STM_MOVE_POSITIONMODE_BITSET = 0 | (MOTOR_RUNNING | MOTOR_POSITIONMODE);
    constexpr uint32_t STM_STOPMOTOR_BITSET = 0;
    constexpr uint32_t STM_TURNROBOT_BITSET = 0 | (TURN_MODE | MOTOR_RUNNING);

    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
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

        volatile uint32_t maxDistance = 0;

        _lineFollowerStatusFlags = 0;

        // loop forever
        for (;;)
        {

            DispatcherMessage message;
            DispatcherMessage response;

            if (uxQueueMessagesWaiting(lineFollowerQueue) > 0)
            {
                xQueueReceive(lineFollowerQueue, &message, pdMS_TO_TICKS(10));

                if (message.receiverTaskId != DispatcherTaskId::LineFollowerTask)
                {
                    printf("LINEFOLLOWERTASK - Message contains wrong Task ID \n");
                    continue;
                }

                // you cant declare vars inside a switch statement, thats why dataContainer
                uint64_t dataContainer = 0;

                switch (message.command)
                {
                case TaskCommand::Move:
                    _lineFollowerStatusFlags &= ~POSITION_REACHED;

                    X_ACTUAL_startValueDriver0 = _driver0.getXActual();
                    X_ACTUAL_startValueDriver1 = _driver1.getXActual();

                    // move infinit as Line Follower
                    if (message.getData() == 0)
                    {
                        maxDistance = 0; // TODO: 2m in usteps
                        _lineFollowerStatusFlags = STM_LINEFOLLOWER_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    }

                    // move in Position Mode
                    if (message.getData() != 0)
                    {
                        _lineFollowerStatusFlags = STM_MOVE_POSITIONMODE_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    }
                    break;

                case TaskCommand::Stop:
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    break;

                case TaskCommand::Turn:
                    _lineFollowerStatusFlags &= ~POSITION_REACHED;
                    _lineFollowerStatusFlags &= ~TURNREQUEST_SEND;
                    _lineFollowerStatusFlags = STM_TURNROBOT_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
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
                    printf("LineFollowerTask - COMMAND UNKNOWN ?");
                    break;
                }
            } // end of message handling

            /// ------- stm check safetybutton -------
            if (!_safetyButton.getValue())
            {
                _lineFollowerStatusFlags &= ~MOTOR_RUNNING;
                _lineFollowerStatusFlags |= SAFETY_BUTTON_PRESSED;
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
                _checkLineFollowerStatus();

                drivenDistanceDriver0 = _driver0.getXActual() - X_ACTUAL_startValueDriver0;
                drivenDistanceDriver1 = _driver1.getXActual() - X_ACTUAL_startValueDriver1;

                // maxdistance reached - send Info to RaspberryHAT
                if ((maxDistance != 0 && drivenDistanceDriver0 > maxDistance) ||
                    (maxDistance != 0 && drivenDistanceDriver1 > maxDistance))
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            // ------- stm move Positionmode -------
            if ((_lineFollowerStatusFlags & STM_MOVE_POSITIONMODE_BITSET) == STM_MOVE_POSITIONMODE_BITSET)
            {
                if (!(_lineFollowerStatusFlags & MOTOR_POSITIONMODE_REQUEST_SEND))
                {
                    _lineFollowerStatusFlags |= MOTOR_POSITIONMODE_REQUEST_SEND;
                    int32_t data = static_cast<int32_t>(message.getData());  // convert to a signed value!
                    float distanceInMeters = static_cast<float>(data) / 1e2; // data contains cm!
                    int32_t microsteps = Tmc5240::convertDistanceToMicrosteps(distanceInMeters);
                    _movePositionMode(microsteps);
                }

                if (_checkForStandstill())
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            /// ------- stm turn vehicle -------
            if ((_lineFollowerStatusFlags & STM_TURNROBOT_BITSET) == STM_TURNROBOT_BITSET)
            {
                _turnRobot(message.getData());

                if (_checkForStandstill())
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
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
                xQueueSend(messageDispatcherQueue, &response, pdMS_TO_TICKS(10));
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

    /// @brief moves whole Robot in positionmode.
    /// @param distance distance [IN MICROSTEPS]
    void MotionController::_movePositionMode(int32_t distance)
    {
        _driver0.moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 1);
        _driver1.moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
        return;
    }

    /// @brief Turn Robort a certain angle.
    /// @param angle angle in [DEGREE ° * 10] (ref prain_uart lib)
    void MotionController::_turnRobot(int32_t angle)
    {
        // check if turn signal got send already
        if (_lineFollowerStatusFlags & TURNREQUEST_SEND)
        {
            return;
        }

        /*
            do some math...

            ds = (phi * width * pi) / (180°)
            => distance one wheel has to move more than the other to rotate the vehicle a certain amount of degree
        */

        float fphi = static_cast<float>(angle);
        float num = fphi * LINEFOLLOWERCONFIG_AXIS_WIDTH_m * M_PI;
        float const dnum = 180.f;

        // divide by 2 due to one wheel is supposed to move forward, one backward
        float ds = num / (2 * dnum);

        // unit conversion meters -> uSteps
        float res = Tmc5240::convertDistanceToMicrosteps(ds);

        int32_t nStepsDriver = static_cast<int32_t>(std::round(res));

        // move drives in different directions
        _driver0.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 1);
        _driver1.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 1);
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

    void MotionController::_checkLineFollowerStatus()
    {
        // check for LineSensor events
        if (_lineSensor.getStatus() & LINESENSOR_CROSS_DETECTED)
        {
            printf("INFO LINEFOLLOWER detected Crosspoint \n");

            _lineFollowerStatusFlags |= CROSSPOINT_DETECTED;

            // send MSG to LineFollowers own - to change Mode into PositionMode and drive until Axis is on top of Crosspoint
            DispatcherMessage msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                      DispatcherTaskId::LineFollowerTask,
                                                      TaskCommand::Move,
                                                      LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_cm);
                                                      
            if(xQueueSend(_lineFollowerQueue, &msg, pdMS_TO_TICKS(100)) != pdTRUE)
            {
                /*   ERROR   stop Drives and send error msg to raspi */
                _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                _lineFollowerStatusFlags |= LINEFOLLOWER_ERROR; 
            }
        }
        if (_lineSensor.getStatus() & LINESENSOR_NO_LINE)
        {
            _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
            _lineFollowerStatusFlags |= LOST_LINE;

            printf("INFO LINEFOLLOWER lost Line \n");
        }
        return;
    }

    /// @brief controller for linefollowing. Config behaviour by settings values in LineFollwoerConfig.h file
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