#include "HandleBarrierStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

#include "DispatcherMessage.hpp"
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
                if (distance < BRAKEDISTANCE_BARRIER_IN_MM)
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
                    _state = HandleBarrierStmState::MIDDLE_ON_LINE;
                }
                break;

            case HandleBarrierStmState::MIDDLE_ON_LINE:
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    static_cast<int32_t>(miscDevices::LineSensorService::getVehicleRotation(_lineSensor) * 10));
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }

                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_1:
                if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
                {
                    _state = HandleBarrierStmState::POSITION_DISTANCE;
                }
                break;

            case HandleBarrierStmState::POSITION_DISTANCE:
                // barrier is out of tolerance?
                if ((distance < LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm ||
                     distance > LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm + LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm))
                {
                    int32_t corrPos = (LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - distance);
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

            _hcSr04->triggerNewMeasurment();
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
                lastMsgData = msgData;
                if (msgData == 0) // LineFollowerMode
                {
                    _state = HandleBarrierStmState::CHECK_DISTANCE;
                }
                break;

            case TaskCommand::GcAck:
                lastMsgData = msgData;
                _gcAck = true;
                break;
            default:
                /* ERROR shouldnt be reached */
                break;
            }
        }
    }
}