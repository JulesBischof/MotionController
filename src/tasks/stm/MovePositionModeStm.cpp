#include "MovePositionModeStm.hpp"

#include "LineFollowerTaskConfig.hpp"
#include "LineFollowerTaskStatusFlags.hpp"

#include "LineSensorService.hpp"
#include "LoggerService.hpp"

#include "Tmc5240.hpp"
#include "StepperService.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        MovePositionModeStm::MovePositionModeStm() {}

        MovePositionModeStm::MovePositionModeStm(uint32_t *_statusFlags,
                                                 spiDevices::Tmc5240 *driver0,
                                                 spiDevices::Tmc5240 *driver1,
                                                 miscDevices::LineSensor *lineSensor,
                                                 QueueHandle_t messageDispatcherQueue
                                                 )
            : StmBase(_statusFlags)
        {
            _driver0 = driver0;
            _driver1 = driver1;
            _lineSensor = lineSensor;

            _lastMsgData = 0;
            _messageDispatcherQueue = messageDispatcherQueue;

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
            DispatcherMessage msg;

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
                _movePositionMode(services::StepperService::convertMillimeterToMicrosteps(static_cast<int32_t>(_lastMsgData)));
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
                _turnRobot(static_cast<int32_t>(_lastMsgData));
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

            case MovePositionModeStmState::MIDDLE_ON_LINE_MODE:
                _lastMsgData = services::LineSensorService::getVehicleRotation(_lineSensor);
                _state = MovePositionModeStmState::TURN_MODE;
                break;

            case MovePositionModeStmState::WAIT_FOR_STOP:
                if (_checkDriversForStandstill())
                {
                    _stoppedTimeStamp = get_absolute_time();
                    services::LoggerService::debug("MovePositionModeStm::run() state#WAIT_FOR_STOP", "standstill detected");
                    _state = MovePositionModeStmState::STOPPED;
                }

                break;
            case MovePositionModeStmState::STOPPED:
                // for safety resons - wait a certain time
                if ((_stoppedTimeStamp + (TIME_UNTIL_STANDSTILL_IN_MS * 1000)) <= get_absolute_time())
                {
                    *_statusFlags |= (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;

                    services::LoggerService::debug("MovePositionModeStm::run() state#STOPPED", "drives stopped");

                    // inform other Tasks
                    msg = DispatcherMessage(
                        DispatcherTaskId::LineFollowerTask,
                        DispatcherTaskId::BarrierHandlerTask,
                        TaskCommand::PositionReached,
                        0);

                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                        services::LoggerService::fatal("MovePositionModeStm::run() state#STOPPED", "_messagDispatcherQueue TIMEOUT");
                        while (1)
                        {
                        }
                    }

                    _state = MovePositionModeStmState::IDLE;
                }
                break;
            default:
                /* ERROR..? */
                break;
            }

            return retVal;
        }

        void MovePositionModeStm::reset()
        {
            services::LoggerService::debug("MovePositionModeStm::reset()", "reset stm");
            _state = MovePositionModeStmState::IDLE;
            _lastMsgData = 0;
        }

        void MovePositionModeStm::update(uint32_t msgData)
        {
            /* ERROR shouldnt be reached */
            services::LoggerService::fatal("MovePositionModeStm::update()", "WRONG OVERLOADED FUNCTION");
            while (1)
            {
            }
        }

        void MovePositionModeStm::update(uint32_t msgData, TaskCommand cmd)
        {
            switch (cmd)
            {
            case TaskCommand::Move:
                _lastMsgData = msgData;
                if (msgData != 0)
                {
                    services::LoggerService::debug("MovePositionModeStm::update()", "recieved command: move in Position Mode");
                    _state = MovePositionModeStmState::POSITION_MODE;
                }
                break;

            case TaskCommand::Turn:
                _lastMsgData = msgData;
                services::LoggerService::debug("MovePositionModeStm::update()", "recieved command: move in Turn Mode");
                _state = MovePositionModeStmState::TURN_MODE;
                break;

            case TaskCommand::Stop:
                services::LoggerService::debug("MovePositionModeStm::update()", "recieved command: Stop Drives");
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

        bool MovePositionModeStm::_checkDriversForStandstill()
        {
            bool val_driver0 = _driver0->checkForStandstill();
            bool val_driver1 = _driver1->checkForStandstill();

            return (val_driver0 && val_driver1);
            // if (val_driver0 && val_driver1)
            // {
            //     *_statusFlags |= (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL;
            //     return true;
            // }
            // return false;
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
            int32_t nStepsDriver = services::StepperService::convertMillimeterToMicrosteps(ds);

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