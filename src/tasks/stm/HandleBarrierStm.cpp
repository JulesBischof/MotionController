#include "HandleBarrierStm.hpp"

#include "LineFollowerTaskConfig.hpp"
#include "LineFollowerTaskStatusFlags.hpp"

#include "LoggerService.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        HandleBarrierStm::HandleBarrierStm(uint32_t *statusFlags,
                                           miscDevices::HcSr04 *hcSr04,
                                           QueueHandle_t messageDispatcherQueue)
            : StmBase(statusFlags),
              _hcSr04(hcSr04),
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

            _medianStack = miscDevices::MedianStack(LINEFOLLOWERCONFIG_MEDIANSTACK_SIZE);
        }

        bool HandleBarrierStm::run()
        {
            bool retVal = false;

            DispatcherMessage msg;

            switch (_state)
            {
            case HandleBarrierStmState::IDLE:
                _posReached = false;
                _gcAck = false;
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::CHECK_DISTANCE:
                if (_hcSr04->getSensorData() > SLOWDOWNDISTANCE_BARRIER_IN_MM)
                {
                    break; // no barrier in sight
                }

                services::LoggerService::debug("HandleBarrierStm::run() state#CHECK_DISTANCE ", "Barrier Detected, SLOW DOWN Vehicle now");

                // slow down robot
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::SlowDown,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#CHECK_DISTANCE", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::SLOWED_DOWN;
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::SLOWED_DOWN:
                if (_hcSr04->getSensorData() > BRAKEDISTANCE_BARRIER_IN_MM)
                {
                    break; // barrier still far away enough
                }

                services::LoggerService::debug("HandleBarrierStm::run() state#SLOWED_DOWN ", "Barrier Detected, STOP Vehicle now");

                // stop vehicle
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SLOWED_DOWN", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }

                // send msg to RaspberryHAT
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::RaspberryHatComTask,
                    TaskCommand::BarrierDetectedInfo,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SLOWED_DOWN", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_0;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_0:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_0 ", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_0 ", "_posReached = true");
                    _posReached = false;
                    _state = HandleBarrierStmState::MIDDLE_ON_LINE;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::MIDDLE_ON_LINE:
                // not implemented for now - cmd is part of MovePositionModeStm
                // _state = HandleBarrierStmState::WAIT_FOR_STOP_1;
                _state = HandleBarrierStmState::POSITION_DISTANCE;
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_1:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_1 ", "_posReached = %d", _posReached);
                /* NOT REACHED FOR NOW due to middle on line is not part of stm */
                _posReached = false;
                _state = HandleBarrierStmState::POSITION_DISTANCE;
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::POSITION_DISTANCE:
            {
                // take some measurments - and then take median
                while (!_medianStack.isFull())
                {
                    _hcSr04->blockForNewMeasurment();
                    _medianStack.push(_hcSr04->getSensorData());
                }
                float distance = _medianStack.getMedian();

                // barrier is out of tolerance?
                services::LoggerService::debug("HandleBarrierStm::run() state#POSITION_DISTANCE ", "checkin Barrier distance median: %f mm", distance);
                if ((distance < LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm ||
                     distance > LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm + LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm))
                {
                    int32_t corrPos = (distance - LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm);
                    services::LoggerService::debug("HandleBarrierStm::run() state#POSITION_DISTANCE ", "Barrier out of tolerance. correct by = %d mm", corrPos);
                    // move position mode
                    msg = DispatcherMessage(
                        DispatcherTaskId::BarrierHandlerTask,
                        DispatcherTaskId::LineFollowerTask,
                        TaskCommand::Move,
                        corrPos);
                    if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                    { /* ERROR!!?? */
                        services::LoggerService::fatal("HandleBarrierStm::run() state#POSITION_DISTANCE", "_messagDispatcherQueue TIMEOUT");
                        while (1)
                        {
                        }
                    }
                }
            }
                _posReached = false;
                _state = HandleBarrierStmState::WAIT_FOR_STOP_2;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_2:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_2", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state# ", "_posReached = true");
                    _posReached = false;
                    _state = HandleBarrierStmState::SEND_GRIP_COMMAND;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::SEND_GRIP_COMMAND:
                services::LoggerService::debug("HandleBarrierStm::run() state#SEND_GRIP_COMMAND", "sending GripCmd now");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneGrip,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SEND_GRIP_COMMAND", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_0;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_GC_ACK_0:
                // _gcAck = true; /* \TODO !!! ONLY FOR DEBUG REASONS*/
                if (_gcAck)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_GC_ACK_0 ", "_gcAck = true");
                    _gcAck = false;
                    _state = HandleBarrierStmState::TURN_ROBOT_0;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::TURN_ROBOT_0:
                services::LoggerService::debug("HandleBarrierStm::run() state#TURN_ROBOT_0 ", "turn Vehicle by 180° ");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    1800); // ° * 10
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#TURN_ROBOT_0", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_STOP_3;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_3:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_3 ", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_3 ", "_posReached = true");
                    _state = HandleBarrierStmState::SET_BACK_ROBOT_0;
                    _posReached = false;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::SET_BACK_ROBOT_0:
                services::LoggerService::debug("HandleBarrierStm::run() state#SET_BACK_ROBOT_0 ", "set back Vehicle now");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    static_cast<uint64_t>(-1 * LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm)); // -1 due to backward
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SET_BACK_ROBOT_0", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _posReached = false;
                _state = HandleBarrierStmState::WAIT_FOR_STOP_4;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_4:
                services::LoggerService::debug("HandleBarrierStm::run() state# ", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state# ", "_posReached = true");
                    _posReached = false;
                    _state = HandleBarrierStmState::SEND_RELEASE_CMD;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::SEND_RELEASE_CMD:
                services::LoggerService::debug("HandleBarrierStm::run() state#SEND_RELEASE_CMD ", "send CraneRelease Cmd");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::GripControllerComTask,
                    TaskCommand::CraneRelease,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SEND_RELEASE_CMD", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::WAIT_FOR_GC_ACK_1;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_GC_ACK_1:
                // _gcAck = true; /* \TODO !!! ONLY FOR DEBUG REASONS*/
                if (_gcAck)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state# ", "_gcAck = true");
                    _state = HandleBarrierStmState::SET_BACK_ROBOT_1;
                    _gcAck = false;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::SET_BACK_ROBOT_1:
                services::LoggerService::debug("HandleBarrierStm::run() state#SET_BACK_ROBOT_1 ", "set back Vehicle now");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    static_cast<uint64_t>(-1 * LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm)); // -1 due to backward
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#SET_BACK_ROBOT_1", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _posReached = false;
                _state = HandleBarrierStmState::WAIT_FOR_STOP_5;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_5:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_5 ", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_5 ", "_posReached = true");
                    _state = HandleBarrierStmState::TURN_ROBOT_1;
                    _posReached = false;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::TURN_ROBOT_1:
                services::LoggerService::debug("HandleBarrierStm::run() state#TURN_ROBOT_1 ", "turn Vehicle by -180° ");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Turn,
                    static_cast<uint64_t>(-1 * 1800)); // ° * 10
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#TURN_ROBOT_1", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _posReached = false;
                _state = HandleBarrierStmState::WAIT_FOR_STOP_6;
                taskYIELD();
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::WAIT_FOR_STOP_6:
                services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_6 ", "_posReached = %d", _posReached);
                if (_posReached)
                {
                    services::LoggerService::debug("HandleBarrierStm::run() state#WAIT_FOR_STOP_6 ", "_posReached = true");
                    _state = HandleBarrierStmState::DONE;
                    _posReached = false;
                }
                break;

                /* ---------------------------------------------------------------*/
            case HandleBarrierStmState::DONE:
                services::LoggerService::debug("HandleBarrierStm::run() state#DONE ", "Barrier Handling DONE");

                services::LoggerService::debug("HandleBarrierStm::run() state#DONE ", "send cmd LINEFOLLOWER_MODE");
                msg = DispatcherMessage(
                    DispatcherTaskId::BarrierHandlerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("HandleBarrierStm::run() state#TURN_ROBOT_1", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                _state = HandleBarrierStmState::IDLE;
                break;

                /* ---------------------------------------------------------------*/
            default:
                services::LoggerService::error("HandleBarrierStm::run()", "unknown state! ");
                break;
            }

            return retVal;
        }

        void HandleBarrierStm::reset()
        {
            _state = HandleBarrierStmState::IDLE;
            _posReached = false;
            _gcAck = false;
        }

        void HandleBarrierStm::update(uint32_t msgData)
        {
            // shouldn't be reached
        }

        void HandleBarrierStm::update(uint32_t msgData, TaskCommand cmd)
        {
            _posReached = false;
            _gcAck = false;

            switch (cmd)
            {
            case TaskCommand::Move:
                if (msgData == 0 && _state == HandleBarrierStmState::IDLE) // LineFollowerMode
                {
                    _state = HandleBarrierStmState::CHECK_DISTANCE;
                    _hcSr04->setCurrentVelocity(V_MAX_IN_MMPS);
                    services::LoggerService::debug("HandleBarrierStm::update() cmd#Move ", "set state to CheckDistance");
                }
                break;

            case TaskCommand::GcAck:
                _gcAck = true;
                break;

            case TaskCommand::PositionReached:
                services::LoggerService::debug("HandleBarrierStm::update() ", "Recieved Command: POSITION_REACHED");
                _posReached = true;
                break;

            case TaskCommand::Stop:
                _state = HandleBarrierStmState::IDLE;
                _hcSr04->setCurrentVelocity(0);
                services::LoggerService::debug("HandleBarrierStm::update() cmd#Stop ", "recieved Stop cmd");
                break;
            default:
                services::LoggerService::error("HandleBarrierStm::update() cmd#Stop ", "unknown command!?");
                /* ERROR shouldnt be reached */
                break;
            }
        }
    }
}