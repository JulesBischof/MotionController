#include "MovePositionModeStm.hpp"

#include "LineFollowerTaskConfig.hpp"
#include "LineFollowerTaskStatusFlags.hpp"

#include "Tmc5240.hpp"
#include "StepperService.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        MovePositionModeStm::MovePositionModeStm() {}

        MovePositionModeStm::MovePositionModeStm(uint32_t *_statusFlags, spiDevices::Tmc5240 *driver0, spiDevices::Tmc5240 *driver1)
            : StmBase(_statusFlags)
        {
            _driver0 = driver0;
            _driver1 = driver1;

            lastMsgData = 0;

            _state = MovePositionModeStmState::IDLE;
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

            bool retVal = false;

            switch (_state)
            {
            case MovePositionModeStmState::IDLE:
                retVal = true;
                break;

            case MovePositionModeStmState::POSITION_MODE:
                // clear flag
                *_statusFlags &= ~(uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
                // set motorcurrent
                _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);
                _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE);
                // now send position request to drives;
                _movePositionMode(spiDevices::StepperService::convertMillimeterToMicrosteps(static_cast<int32_t>(lastMsgData)));
                // now wait for standstill
                _state = MovePositionModeStmState::WAIT_FOR_STOP;
                break;

            case MovePositionModeStmState::TURN_MODE:
                // clear flag
                *_statusFlags &= ~(uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
                // set motorcurrent
                _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);
                _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE);
                // send position request to drives
                _turnRobot(static_cast<int32_t>(lastMsgData));
                // now wait for standstill
                _state = MovePositionModeStmState::WAIT_FOR_STOP;
                break;

            case MovePositionModeStmState::STOP_MODE:
                // clear flag
                *_statusFlags &= ~(uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
                // set motorcurrent
                _driver0->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);
                _driver1->setRunCurrent(LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE);
                // send position request to drives
                _stopDrives();
                // now wait for standstill
                _state = MovePositionModeStmState::WAIT_FOR_STOP;

                break;
            case MovePositionModeStmState::WAIT_FOR_STOP:
                if (_checkForStandstill())
                {
                    _state = MovePositionModeStmState::STOPPED;
                }

                break;
            case MovePositionModeStmState::STOPPED:
                // set flag
                *_statusFlags |= (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
                _state = MovePositionModeStmState::IDLE;
                break;
            default:
                /* ERROR..? */
                break;
            }

            return retVal;
        }

        void MovePositionModeStm::reset()
        {
            _state = MovePositionModeStmState::IDLE;
            lastMsgData = 0;
        }

        void MovePositionModeStm::update(uint32_t msgData)
        {
            /* ERROR shouldnt be reached */
        }

        void MovePositionModeStm::update(uint32_t msgData, TaskCommand cmd)
        {
            switch (cmd)
            {
            case TaskCommand::Move:
                lastMsgData = msgData;
                if (msgData != 0)
                {
                    _state = MovePositionModeStmState::POSITION_MODE;
                }
                break;

            case TaskCommand::Turn:
                lastMsgData = msgData;
                _state = MovePositionModeStmState::TURN_MODE;
                break;

            case TaskCommand::Stop:
                _state = MovePositionModeStmState::STOP_MODE;
                break;
            default:
                /* ERROR shouldnt be reached */
                break;
            }
        }

        /// @brief moves whole Robot a certain distance in positionmode.
        /// @param distance distance [IN MICROSTEPS]
        void MovePositionModeStm::_movePositionMode(int32_t distance)
        {
            _driver0->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_REGISTER_VALUE, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE, 1);
            _driver1->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_REGISTER_VALUE, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE, 0);
            return;
        }

        bool MovePositionModeStm::_checkForStandstill()
        {
            bool val_driver0 = _driver0->checkForStandstill();
            bool val_driver1 = _driver1->checkForStandstill();
            if (val_driver0 && val_driver1)
            {
                *_statusFlags |= (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
                return true;
            }
            return false;
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
            int32_t nStepsDriver = spiDevices::StepperService::convertMillimeterToMicrosteps(ds);

            // move drives in different directions
            _driver0->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_REGISTER_VALUE * 2, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE, 0);
            _driver1->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_REGISTER_VALUE * 2, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE, 0);
        }

        /// @brief stops the drives
        /// @details stops the drives in velocity mode. In position mode, the drives are stopped by the driver itself.
        void MovePositionModeStm::_stopDrives()
        {
            _driver0->moveVelocityMode(1, 0, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE);
            _driver1->moveVelocityMode(0, 0, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE);
            return;
        }

    } // namespace MotionController
}