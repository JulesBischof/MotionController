#include "HandleBarrierStm.hpp"

#include "LineFollowerTaskConfig.hpp"
#include "LineFollowerTaskStatusFlags.hpp"

#include "LineSensorService.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        HandleBarrierStm::HandleBarrierStm(uint32_t *statusFlags,
                                           miscDevices::HcSr04 *hcSr04,
                                           miscDevices::LineSensor *lineSensor,
                                           QueueHandle_t lineFollowerTaskQueue,
                                           QueueHandle_t messageDispatcherQueue)
            : StmBase(statusFlags),
              _hcSr04(hcSr04),
              _lineSensor(lineSensor),
              _lineFollowerTaskQueue(lineFollowerTaskQueue),
              _messageDispatcherQueue(messageDispatcherQueue)
        {
            init();
        }

        HandleBarrierStm::HandleBarrierStm()
        {
        }

        HandleBarrierStm::~HandleBarrierStm()
        {
        }

        void HandleBarrierStm::init()
        {
            _state = HandleBarrierStmState::IDLE;
            _gcAck = false;
        }

        bool HandleBarrierStm::run()
        {
            bool retVal = false;

            DispatcherMessage msg;

            float distance = _hcSr04->getSensorData();

            switch (_state)
            {
            case HandleBarrierStmState::IDLE:
                retVal = true;
                break;

            case HandleBarrierStmState::CHECK_DISTANCE:
                if (distance < BRAKEDISTANCE_BARRIER_IN_MM) // BRAKEDISTANCE_BARRIER_IN_MM
                {
                    // stop drives and start Barrier Detected state maschine
                    msg = DispatcherMessage(
                        DispatcherTaskId::LineFollowerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::Stop,
                        0);
                    if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                    { /* ERROR!!?? */
                    }
                    *_statusFlags |= (uint32_t)RunModeFlag::LINEFOLLOWER_BARRIER_DETECTED;
                    _state = HandleBarrierStmState::WAIT_FOR_STOP_0;
                }
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_0:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _hcSr04->setCurrentVelocity(0);
                    _state = HandleBarrierStmState::MIDDLE_ON_LINE;
                }
                break;

            case HandleBarrierStmState::MIDDLE_ON_LINE:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    services::LineSensorService::getVehicleRotation(_lineSensor));
                // if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                // { /* ERROR!!?? */
                // }

                _state = HandleBarrierStmState::WAIT_FOR_STOP_1;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_1:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::POSITION_DISTANCE;
                }
                break;

            case HandleBarrierStmState::POSITION_DISTANCE:
                // barrier is out of tolerance?
                if ((distance < LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm ||
                     distance > LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm + LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm))
                {
                    int32_t corrPos = (distance - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm);
                    // correct postion
                    msg = DispatcherMessage(
                        DispatcherTaskId::LineFollowerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::Move,
                        corrPos);
                    if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                    { /* ERROR!!?? */
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_2;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_2:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::SEND_GRIP_COMMAND;
                }
                break;

            case HandleBarrierStmState::SEND_GRIP_COMMAND:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneGrip,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }

                _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_0;
                break;

            case HandleBarrierStmState::WAIT_FOR_GC_ACK_0:
                if (_gcAck)
                {
                    _gcAck = false;
                    _state = HandleBarrierStmState::TURN_ROBOT_0;
                }
                break;

            case HandleBarrierStmState::TURN_ROBOT_0:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    1800); // ° * 10
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_3;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_3:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::SET_BACK_ROBOT;
                }
                break;

            case HandleBarrierStmState::SET_BACK_ROBOT:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    -1 * LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm); // -1 due to backward
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_4;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_4:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::SEND_RELEASE_CMD;
                }
                break;

            case HandleBarrierStmState::SEND_RELEASE_CMD:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneRelease,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }

                _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_1;
                break;

            case HandleBarrierStmState::WAIT_FOR_GC_ACK_1:
                if (_gcAck)
                {
                    _gcAck = false;
                    _state = HandleBarrierStmState::TURN_ROBOT_1;
                }
                break;

            case HandleBarrierStmState::TURN_ROBOT_1:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    -1800); // ° * 10
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_5;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_5:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::DONE;
                }
                break;

            case HandleBarrierStmState::DONE:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    0); // 0 = LineFollowerMode
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                break;

            default:
                break;
            }
            return retVal;
        }

        void HandleBarrierStm::reset()
        {
            _state = HandleBarrierStmState::IDLE;
        }

        void HandleBarrierStm::update(uint32_t msgData)
        {
            // shouldn't be reached
        }

        void HandleBarrierStm::update(uint32_t msgData, TaskCommand cmd)
        {
            switch (cmd)
            {
            case TaskCommand::Move:
                _lastMsgData = msgData;
                if (msgData == 0) // LineFollowerMode
                {
                    _state = HandleBarrierStmState::CHECK_DISTANCE;
                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);
                }
                break;

            case TaskCommand::GcAck:
                _lastMsgData = msgData;
                _gcAck = true;
                break;

            case TaskCommand::Stop:
                _lastMsgData = msgData;
                _state = HandleBarrierStmState::IDLE;
                *_statusFlags &= ~(uint32_t)RunModeFlag::LINEFOLLOWER_BARRIER_DETECTED;
                break;
            default:
                /* ERROR shouldnt be reached */
                break;
            }
        }
    }
}