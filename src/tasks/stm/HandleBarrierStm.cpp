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
                                           QueueHandle_t messageDispatcherQueue)
            : StmBase(statusFlags),
              _hcSr04(hcSr04),
              _lineSensor(lineSensor),
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
            _posReached = false;
        }

        bool HandleBarrierStm::run()
        {
            bool retVal = false;

            DispatcherMessage msg;

            float distance = _hcSr04->getSensorData();

            switch (_state)
            {
            case HandleBarrierStmState::IDLE:
                _gcAck = false;
                _posReached = false;
                retVal = true;
                break;

            case HandleBarrierStmState::CHECK_DISTANCE:
                if (distance < SLOWDOWNDISTANCE_BARRIER_IN_MM)
                {
                    // slow down robot
                    msg = DispatcherMessage(
                        DispatcherTaskId::BarrierHandlerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::SlowDown,
                        0);
                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                    }
                    _state = HandleBarrierStmState::SLOW_DOWN;
                }
                break;

            case HandleBarrierStmState::SLOW_DOWN:
                if (distance < BRAKEDISTANCE_BARRIER_IN_MM)
                {
                    _hcSr04->setCurrentVelocity(V_SLOW_IN_MMPS);
                    // stop drives
                    msg = DispatcherMessage(
                        DispatcherTaskId::BarrierHandlerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::Stop,
                        0);
                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                    }

                    // send msg to RaspberryHAT
                    msg = DispatcherMessage(
                        DispatcherTaskId::BarrierHandlerTask,
                        DispatcherTaskId::RaspberryHatComTask,
                        TaskCommand::BarrierDetectedInfo,
                        0);
                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                    }
                    _state = HandleBarrierStmState::WAIT_FOR_STOP_0;
                }
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_0:
                if (_posReached)
                {
                    _hcSr04->setCurrentVelocity(0);
                    _state = HandleBarrierStmState::MIDDLE_ON_LINE;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::MIDDLE_ON_LINE:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    services::LineSensorService::getVehicleRotation(_lineSensor));
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }

                _state = HandleBarrierStmState::WAIT_FOR_STOP_1;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_1:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::POSITION_DISTANCE;
                    _posReached = false;
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
                        DispatcherTaskId::BarrierHandlerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::Move,
                        corrPos);
                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_2;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_2:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::SEND_GRIP_COMMAND;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::SEND_GRIP_COMMAND:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneGrip,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
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

                // TODO: REMOVE!!! ------- DEBUG WITHOUT GC ONLY
                vTaskDelay(1000);
                _state = HandleBarrierStmState::TURN_ROBOT_0;
                // REMOVE !!!
                break;

            case HandleBarrierStmState::TURN_ROBOT_0:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    1800); // ° * 10
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_3;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_3:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::SET_BACK_ROBOT_0;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::SET_BACK_ROBOT_0:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    -1 * LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm); // -1 due to backward
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_4;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_4:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::SEND_RELEASE_CMD;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::SEND_RELEASE_CMD:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneRelease,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_1;
                break;

            case HandleBarrierStmState::WAIT_FOR_GC_ACK_1:
                if (_gcAck)
                {
                    _gcAck = false;
                    _state = HandleBarrierStmState::SET_BACK_ROBOT_1;
                }

                // TODO: REMOVE!!! ------- DEBUG WITHOUT GC ONLY
                vTaskDelay(1000);
                _state = HandleBarrierStmState::SET_BACK_ROBOT_1;
                // REMOVE !!!
                break;

            case HandleBarrierStmState::SET_BACK_ROBOT_1:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    -1 * LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm); // -1 due to backward
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_5;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_5:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::TURN_ROBOT_1;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::TURN_ROBOT_1:
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    -1800); // ° * 10
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_6;
                break;

            case HandleBarrierStmState::WAIT_FOR_STOP_6:
                if (_posReached)
                {
                    _state = HandleBarrierStmState::DONE;
                    _posReached = false;
                }
                break;

            case HandleBarrierStmState::DONE:
                reset();
                
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    0); // 0 = LineFollowerMode
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
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
                if (msgData == 0) // LineFollowerMode
                {
                    _state = HandleBarrierStmState::CHECK_DISTANCE;
                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);
                }
                break;

            case TaskCommand::PositionReached:
                _posReached = true;
                break;

            case TaskCommand::GcAck:
                _gcAck = true;
                break;

            case TaskCommand::Stop:
                _state = HandleBarrierStmState::IDLE;
                break;
            default:
                /* ERROR shouldnt be reached */
                break;
            }
        }
    }
}