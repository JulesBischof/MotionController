#include "MovePositionModeStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

namespace MotionController
{
    MovePositionModeStm::MovePositionModeStm(uint32_t *_statusFlags, Tmc5240 *driver0, Tmc5240 *driver1)
        : StmBase(_statusFlags)
    {
        _driver0 = driver0;
        _driver1 = driver1;

        lastMsgData = 0;

        _state = State::IDLE;
    }

    MovePositionModeStm::~MovePositionModeStm()
    {
    }

    void MovePositionModeStm::init()
    {
    }

    bool MovePositionModeStm::run()
    {
        uint32_t statusFlags = *_statusFlags;
        // chek if a stop request was send (statusFlag is set)
        if (statusFlags & ~MOTOR_RUNNING)
        {
            _state = State::STOP_MODE;
        }

        bool retVal = false;

        switch (_state)
        {
        case State::IDLE:
            retVal = true;
            break;

        case State::POSITION_MODE:

            // set motorcurrent
            _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);
            _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);
            // now send position request to drives
            _movePositionMode(lastMsgData);
            // now wait for standstill
            _state = State::WAIT_FOR_STOP;
            break;

        case State::TURN_MODE:
            // set motorcurrent
            _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);
            _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);
            // send position request to drives
            _turnRobot(lastMsgData);
            // now wait for standstill
            _state = State::WAIT_FOR_STOP;
            break;

        case State::STOP_MODE:
            // set motorcurrent
            _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);
            _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);
            // send position request to drives
            _stopDrives();
            // now wait for standstill
            _state = State::WAIT_FOR_STOP;

            break;
        case State::WAIT_FOR_STOP:
            bool val0 = _driver0->checkForStandstill();
            bool val1 = _driver1->checkForStandstill();

            if (val0 && val1)
            {
                _state = State::STOPPED;
            }

            break;
        case State::STOPPED:
            *_statusFlags |= MOTORS_AT_STANDSTILL;
            _state = State::IDLE;
            break;
        default:
            /* ERROR..? */
            break;
        }

        return retVal;
    }

    void MovePositionModeStm::reset()
    {
        _state = State::IDLE;
        lastMsgData = 0;
    }

    void MovePositionModeStm::update(uint32_t msgData)
    {
        // move in Position Mode
        if (msgData != 0)
        {
            lastMsgData = msgData;
            _state = State::POSITION_MODE;
        }
    }

    /// @brief moves whole Robot a certain distance in positionmode.
    /// @param distance distance [IN MICROSTEPS]
    void MovePositionModeStm::_movePositionMode(int32_t distance)
    {
        _driver0->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 1);
        _driver1->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
        return;
    }

    /// @brief Turn Robort a certain angle.
    /// @param angle angle in [DEGREE ° * 10] (ref prain_uart lib)
    void MovePositionModeStm::_turnRobot(int32_t angle)
    {
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
        _driver0->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
        _driver1->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED, 0);
    }

    /// @brief stops the drives
    /// @details stops the drives in velocity mode. In position mode, the drives are stopped by the driver itself.
    void MovePositionModeStm::_stopDrives()
    {
        _driver0->moveVelocityMode(1, 0, 5 * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1->moveVelocityMode(0, 0, 5 * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        return;
    }

} // namespace MotionController