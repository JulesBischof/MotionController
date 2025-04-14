#include "HandleBarrierStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

#include "DispatcherMessage.hpp"

namespace nMotionController
{
    HandleBarrierStm::HandleBarrierStm(uint32_t *statusFlags, HcSr04 *hcSr04, Tmc5240 *driver0, Tmc5240 *driver1, QueueHandle_t lineFollowerTaskQueue)
        : StmBase(statusFlags),
          _hcSr04(hcSr04),
          _driver0(driver0),
          _driver1(driver1),
          _lineFollowerTaskQueue(lineFollowerTaskQueue)
    {
        init();
    }

    HandleBarrierStm::HandleBarrierStm() {}

    HandleBarrierStm::~HandleBarrierStm()
    {
    }

    void HandleBarrierStm::init()
    {
        _state = HandleBarrierStmState::IDLE;
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
                msg = DispatcherMessage (
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
            // TODO

            _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_0;
            break;

        case HandleBarrierStmState::WAIT_FOR_GC_ACK_0:
            // TODO

            _state = HandleBarrierStmState::TURN_ROBOT_1;
            break;

        case HandleBarrierStmState::TURN_ROBOT_1:
            msg = DispatcherMessage (
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

            // TODO
            _state = HandleBarrierStmState::WAIT_FOR_STOP_4;
            break;

        case HandleBarrierStmState::WAIT_FOR_STOP_4:
            if (*_statusFlags & (uint32_t)RunModeFlag::MOTORS_AT_STANDSTILL)
            {
                _state = HandleBarrierStmState::SEND_RELEASE_COMMAND;
            }
            break;

        case HandleBarrierStmState::SEND_RELEASE_COMMAND:
            // TODO
            _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_1;
            break;

        case HandleBarrierStmState::WAIT_FOR_GC_ACK_1:
            // TODO
            _state = HandleBarrierStmState::TURN_ROBOT_2;
            break;

        case HandleBarrierStmState::TURN_ROBOT_2:
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
                _state = HandleBarrierStmState::BACK_TO_LINEFOLLOWERMODE;
            }
            break;

        case HandleBarrierStmState::BACK_TO_LINEFOLLOWERMODE:
            msg = DispatcherMessage(
                DispatcherTaskId::LineFollowerTask,
                DispatcherTaskId::LineFollowerTask,
                TaskCommand::Move,
                0); // 0 = LineFollowerMode
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
        lastMsgData = msgData;

        // gets called from Move command. 0 = move LineFollowerMode
        if (msgData == 0)
        {
            _state == HandleBarrierStmState::CHECK_DISTANCE;
        }
    }
}