#include "LineFollowerStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

#include "DispatcherMessage.hpp"

namespace nMotionController
{
    LineFollowerStm::LineFollowerStm(uint32_t *_statusFlags, LineSensor *lineSensor, Tmc5240 *driver0, Tmc5240 *driver1, QueueHandle_t LineFollowerTaskQueue)
        : StmBase(_statusFlags), _lineSensor(lineSensor), _driver0(driver0), _driver1(driver1), _lineFollowerTaskQueue(LineFollowerTaskQueue)
    {
    }

    LineFollowerStm::LineFollowerStm()
    {
    }

    LineFollowerStm::~LineFollowerStm()
    {
    }

    void LineFollowerStm::init()
    {
        lastMsgData = 0;
        _state = LineFollowerStmState::IDLE;
    }

    bool LineFollowerStm::run()
    {
        bool retVal = false;

        DispatcherMessage msg;

        switch(_state)
        {
            case LineFollowerStmState::IDLE:
                retVal = true;
                break;

            case LineFollowerStmState::FOLLOW_LINE:
                _followLine();

                if (_lineSensor->getStatus() & LINESENSOR_CROSS_DETECTED)
                {
                    _state = LineFollowerStmState::CROSSPOINT_DETECTED;
                }
                if (_lineSensor->getStatus() & LINESENSOR_NO_LINE)
                {
                    _state = LineFollowerStmState::LOST_LINE;
                }

                break;

            case LineFollowerStmState::LOST_LINE:
                // stop motors
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Stop, 
                    0);
                if(xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                {/* ERROR!!?? */}
                *_statusFlags |= (uint32_t)RunModeFlag::LOST_LINE;

                _state = LineFollowerStmState::IDLE;
                break;

            case LineFollowerStmState::CROSSPOINT_DETECTED:
                // move in positionmode on top of node
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm);
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                *_statusFlags |= (uint32_t)RunModeFlag::CROSSPOINT_DETECTED;

                _state = LineFollowerStmState::IDLE;
                break;

            default:
                /* ERROR..? */
                break;
            }

        return retVal;
    }

    void LineFollowerStm::reset()
    {
        _state = LineFollowerStmState::IDLE;
    }

    void LineFollowerStm::update(uint32_t msgData)
    {
        if(msgData == 0)
        {
            _state = LineFollowerStmState::FOLLOW_LINE;
        }
    }

    /// @brief controller for linefollowing. Config behaviour by settings values in LineFollwoerConfig.h file
    void LineFollowerStm::_followLine()
    {
        // init vars
        uint32_t statusFlags = *_statusFlags;
        bool speedMode = (statusFlags & (uint32_t)RunModeFlag::RUNMODE_SLOW) ? true : false;

        int32_t v1 = 0;
        int32_t v2 = 0;

        // get Sensor values
#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
        int16_t y = _lineSensor->getLinePositionDigital();
#else
        int16_t y = _lineSensor->getLinePositionAnalog();
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
        _driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    } // end followLine

    /// @brief Controller Method for LineFollowing
    /// @param e error as input
    /// @return returns controller parameter u as output
    int32_t LineFollowerStm::_controllerC(int32_t e)
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
    } // end ControllerC
}