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

namespace MotionController
{


    /* ================================= */
    /*          Running Task             */
    /* ================================= */

    /// @brief main loop lineFollowerTask. Statemaschine controlled by Statusflags to control Robot.
    void MotionController::_lineFollowerTask()
    {
        // init vars
        _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET;
        TickType_t xLastWakeTime = xTaskGetTickCount();

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

                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);

                    // move infinit as Line Follower
                    if (message.getData() == 0)
                    {
                        _driver0.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_LINEFOLLOWER_PERCENTAGE);
                        _driver1.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_LINEFOLLOWER_PERCENTAGE);

                        maxDistance = 0; // TODO: 2m in usteps
                        _lineFollowerStatusFlags = STM_LINEFOLLOWER_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    }

                    // move in Position Mode
                    if (message.getData() != 0)
                    {
                        _driver0.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);
                        _driver1.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);

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
            if ((_lineFollowerStatusFlags & STM_LINEFOLLOWER_BITSET) == STM_LINEFOLLOWER_BITSET)
            {
                float distance = _hcSr04->getSensorData();
                // _hcSr04->triggerNewMeasurment();

                if (distance < BRAKEDISTANCE_BARRIER_IN_MM)
                {
                    // stop drives and start Barrier Detected state maschine
                    _lineFollowerStatusFlags |= LINEFOLLOWER_BARRIER_DETECTED;
                }
            }
            // if ((_lineFollowerStatusFlags & LINEFOLLOWER_BARRIER_DETECTED) == LINEFOLLOWER_BARRIER_DETECTED)
            // {
            //     _handleBarrier(distance);
            // }

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
                    int32_t signedData = static_cast<int32_t>(message.getData());               // convert to signed value
                    float distance = static_cast<float>(signedData);                            // convert to a signed value!
                    int32_t microsteps = 10 * Tmc5240::convertDistanceMmToMicrosteps(distance); // 10 due to unit conversion
                    _movePositionMode(microsteps);
                }

                if (_checkForStandstill())
                {
                    _lineFollowerStatusFlags = STM_STOPMOTOR_BITSET | (_lineFollowerStatusFlags & RUNMODEFLAG_T_UPPER_BITMASK);
                    _lineFollowerStatusFlags |= POSITION_REACHED;
                }
            }

            // ------- stm find line (center robot on Line) -------
            if ((_lineFollowerStatusFlags & STM_FIND_LINE_BITSET) == STM_FIND_LINE_BITSET)
            {
                // get Line position
                uint8_t linePos = _lineSensor.getLinePositionDigital();

                float angle = (linePos * 1.254f / LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm) * 180 / M_PI;
                // send to Queue TODO
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
                _hcSr04->setCurrentVelocity(0);
                _stopDrives();
            }
            _hcSr04->triggerNewMeasurment();
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(LINEFOLLOWERCONFIG_POLLING_RATE_MS));
        }
    } // end Task

    /* ================================= */
    /*           Drive Control           */
    /* ================================= */

    void MotionController::_handleBarrier(float distance)
    {
        // _handle barrier as a state machine
        enum stm
        {
            IDLE,
            WAIT_FOR_STOP_0,          // wait for robot to stop
            MIDDLE_ON_LINE,           // send driver command: middle on line
            WAIT_FOR_STOP_1,          // wait for robot to stop
            POSITION_DISTANCE,        // send driver command: correct barrierdistance
            WAIT_FOR_STOP_2,          // wait for robot to stop
            SEND_GRIP_COMMAND,        // send GC-command: grip barrier
            WAIT_FOR_GC_ACK_0,        // wait for GC-ACK
            TURN_ROBOT_1,             // send driver command: turn robot
            WAIT_FOR_STOP_3,          // wait for robot to stop
            SET_BACK_ROBOT,           // send driver command: move back
            WAIT_FOR_STOP_4,          // wait for robot to stop
            SEND_RELEASE_COMMAND,     // send GC-command: release barrier
            WAIT_FOR_GC_ACK_1,        // wait for GC-ACK
            TURN_ROBOT_2,             // send driver command: turn robot
            WAIT_FOR_STOP_5,          // wait for robot to stop
            BACK_TO_LINEFOLLOWERMODE, // send driver command: move robot
        };

        static uint8_t state = stm::IDLE;

        switch (state)
        {
        case IDLE:
            _lineFollowerStatusFlags &= ~MOTOR_RUNNING;
            state++;
            break;

        case WAIT_FOR_STOP_0:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case MIDDLE_ON_LINE:
            break;

        case WAIT_FOR_STOP_1:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case POSITION_DISTANCE:
            break;

        case WAIT_FOR_STOP_2:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case SEND_GRIP_COMMAND:
            break;

        case WAIT_FOR_GC_ACK_0:
            break;

        case TURN_ROBOT_1:
            _turnRobot(1800); // 180° in [° * 10]
            state++;
            break;

        case WAIT_FOR_STOP_3:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case SET_BACK_ROBOT:
            break;

        case WAIT_FOR_STOP_4:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case SEND_RELEASE_COMMAND:
            break;

        case WAIT_FOR_GC_ACK_1:
            break;

        case TURN_ROBOT_2:
            _turnRobot(-1800); // 180° in [° * 10]
            state++;
            break;

        case WAIT_FOR_STOP_5:
            if (_checkForStandstill())
            {
                state++;
            }
            break;

        case BACK_TO_LINEFOLLOWERMODE:
        
            break;

        default:
            break;
        }
    }

    /// @brief moves whole Robot in positionmode.
    /// @param distance distance [IN MICROSTEPS]
    void
    MotionController::_movePositionMode(int32_t distance)
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

        _driver0.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);
        _driver1.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);

        /*
            do some math...

            ds = (phi * width * pi) / (180°)
            => distance one wheel has to move more than the other to rotate the vehicle a certain amount of degree
        */

        float num = static_cast<float>(angle * LINEFOLLOWERCONFIG_AXIS_WIDTH_mm * M_PI);

        // 10 due to angle gets send in [°] * 10; 180° -> data = 1800
        // divide by 2 due to one wheel has to drive forward, one backward;
        float const dnum = 2 * 180.f * 10;

        // distance per wheel in mm
        float ds = num / dnum;

        // unit conversion mm -> uSteps
        int32_t nStepsDriver = Tmc5240::convertDistanceMmToMicrosteps(ds);

        // move drives in different directions
        _driver0.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
        _driver1.moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
        _lineFollowerStatusFlags |= TURNREQUEST_SEND;
    }

    /// @brief stops the drives
    /// @details stops the drives in velocity mode. In position mode, the drives are stopped by the driver itself.
    void MotionController::_stopDrives()
    {
        if (_lineFollowerStatusFlags & MOTOR_STOPREQUEST_SEND)
        {
            return;
        }
        _driver0.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);
        _driver1.setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);

        _driver0.moveVelocityMode(1, 0, 5 * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveVelocityMode(0, 0, 5 * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _lineFollowerStatusFlags |= MOTOR_STOPREQUEST_SEND;

        return;
    } // end stop Drives

    /* ================================= */
    /*           Controller              */
    /* ================================= */

    /// @brief controller for linefollowing. Config behaviour by settings values in LineFollwoerConfig.h file
    void MotionController::_followLine()
    {
        // init vars
        bool speedMode = (_lineFollowerStatusFlags & RUNMODE_SLOW) ? true : false;

        int32_t v1 = 0;
        int32_t v2 = 0;

        // get Sensor values
#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
        int16_t y = _lineSensor.getLinePositionDigital();
#else
        int16_t y = _lineSensor.getLinePositionAnalog();
#endif

        // get error
        int16_t e = 0;

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

    /// @brief Controller Method for LineFollowing
    /// @param e error as input
    /// @return returns controller parameter u as output
    int32_t MotionController::_controllerC(int8_t e)
    {
        static int32_t last_e = 0;

#if LINEFOLLERCONFIG_USE_P_CONTROLLER == (1) && LINEFOLLERCONFIG_USE_PD_CONTROLLER == (0)
        // P-Type Controller
        int32_t u = 0; // default 0
        u = e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP;
#endif

#if LINEFOLLERCONFIG_USE_P_CONTROLLER == (0) && LINEFOLLERCONFIG_USE_PD_CONTROLLER == (1)
        // D-Controller with low Pass Filter
        int32_t u = 0; // default 0

        int32_t deltaE = (e - last_e) / LINEFOLLOWERCONFIG_POLLING_RATE_MS;
        last_e = e;

        u = LINEFOLLERCONFIG_CONTROLLERVALUE_KP * e + LINEFOLLERCONFIG_CONTROLLERVALUE_KD * deltaE;
#endif

        int32_t retVal = u * LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P;
        return retVal;
    } // end Control

    /* ================================= */
    /*           Helpers                 */
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
                                                      LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm / 10); // convert to cm

            if (xQueueSend(_lineFollowerQueue, &msg, pdMS_TO_TICKS(100)) != pdTRUE)
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