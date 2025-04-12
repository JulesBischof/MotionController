#include "HandleBarrierStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

#include "MotionController.hpp"

namespace MotionController
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

    HandleBarrierStm::~HandleBarrierStm()
    {
    }

    void HandleBarrierStm::init()
    {
        _state = State::IDLE;
    }

    bool HandleBarrierStm::run()
    {
        bool retVal = false;

        float distance = _hcSr04->getSensorData();

        switch (_state)
        {
        case State::IDLE:
            return true;
            break;

        case State::CHECK_DISTANCE:
            if (distance < BRAKEDISTANCE_BARRIER_IN_MM)
            {
                // stop drives and start Barrier Detected state maschine
                DispatcherMessage msg(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }

                *_statusFlags |= LINEFOLLOWER_BARRIER_DETECTED;

                _state++;
            }
            break;

        case State::WAIT_FOR_STOP_0:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

        case State::MIDDLE_ON_LINE:
            break;

        case State::WAIT_FOR_STOP_1:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

            // LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_cm

        case State::POSITION_DISTANCE:
            // barrier is out of tolerance?
            if ((distance < LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm ||
                 distance > LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm + LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm))
            {
                int32_t corrPos = (LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - distance);
                // correct postion
                DispatcherMessage msg(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    corrPos / 10); // convert to cm
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
            }

            _state++;

            break;

        case State::WAIT_FOR_STOP_2:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

        case State::SEND_GRIP_COMMAND:
            // TODO

            _state++;
            break;

        case State::WAIT_FOR_GC_ACK_0:
            // TODO

            _state++;
            break;

        case State::TURN_ROBOT_1:
            DispatcherMessage msg(
                DispatcherTaskId::LineFollowerTask,
                DispatcherTaskId::LineFollowerTask,
                TaskCommand::Turn,
                1800); // ° * 10
            if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
            { /* ERROR!!?? */
            }
            _state++;
            break;

        case State::WAIT_FOR_STOP_3:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

        case State::SET_BACK_ROBOT:
            break;

        case State::WAIT_FOR_STOP_4:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

        case State::SEND_RELEASE_COMMAND:
            // TODO
            _state++;
            break;

        case State::WAIT_FOR_GC_ACK_1:
            // TODO
            _state++;
            break;

        case State::TURN_ROBOT_2:
            DispatcherMessage msg(
                DispatcherTaskId::LineFollowerTask,
                DispatcherTaskId::LineFollowerTask,
                TaskCommand::Turn,
                -1800); // ° * 10
            if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
            { /* ERROR!!?? */
            }
            _state++;
            break;

        case State::WAIT_FOR_STOP_5:
            if (*_statusFlags & MOTORS_AT_STANDSTILL)
            {
                _state++;
            }
            break;

        case State::BACK_TO_LINEFOLLOWERMODE:
            DispatcherMessage msg(
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
        _state = State::IDLE;
    }

    void HandleBarrierStm::update(uint32_t msgData)
    {
        lastMsgData = msgData;

        // gets called from Move command. 0 = move LineFollowerMode
        if (msgData == 0)
        {
            _state == State::CHECK_DISTANCE;
        }
    }
}