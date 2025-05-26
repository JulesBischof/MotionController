#include "CheckForLineStm.hpp"
#include "LoggerService.hpp"
#include "LineSensor.hpp"
#include "LineFollowerTaskConfig.hpp"

namespace MtnCtrl
{

    namespace stm
    {
        CheckForLineStm::CheckForLineStm(miscDevices::LineSensor *lineSensor, QueueHandle_t messageDispatcherQueue)
            : _lineSensor(lineSensor), _messageDispatcherQueue(messageDispatcherQueue)
        {
            init();
        }

        void CheckForLineStm::init()
        {
            _state = CheckForLineStmState::IDLE;
            _measCounter = 0;
            _stack = services::MedianStack<uint16_t>(LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS);
            _posReachedFlag = false;
        }

        bool CheckForLineStm::run()
        {
            switch (_state)
            {
            case (CheckForLineStmState::IDLE):
                break;

            case (CheckForLineStmState::CHECK_APPEARANCE):
            {
                services::LoggerService::debug("CheckForLineStm::run()", "Check appearance...");
                _lineSensor->toggleUvLed(true);

                // first check counter - and break cycle if neccessary
                if (_measCounter >= POLL_LINE_N_CHECK_CYCLES)
                {
                    // getLinePos - this thime for real
                    for (size_t i = 0; i < LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS; i++)
                    {
                        _stack.push(_lineSensor->getLinePositionAnalog());
                        vTaskDelay(pdMS_TO_TICKS(1));
                    }
                    _lineSensor->toggleUvLed(false);

                    // send median to RaspberryHat
                    DispatcherMessage msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                              DispatcherTaskId::RaspberryHatComTask,
                                                              TaskCommand::PollLineSensor,
                                                              _stack.getMedian());
                    xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY);

                    // clear buffer
                    _stack.clearBuffer();

                    // move back vehicle
                    msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                              DispatcherTaskId::LineFollowerTask,
                                                              TaskCommand::Move,
                                                              (-1 * _measCounter * POLL_LINE_MOVE_INCREMENT_MM));
                    xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY);

                    // reset stm
                    reset();
                    break;
                }

                _measCounter++;

                // per default: get line position until linesensor claimes NO LINE
                for (size_t i = 0; i < LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS; i++)
                {
                    _lineSensor->getLinePositionAnalog();
                    vTaskDelay(pdMS_TO_TICKS(5));
                }
                _lineSensor->toggleUvLed(false);

                if (_lineSensor->getStatus() != miscDevices::LINESENSOR_NO_LINE)
                {
                    services::LoggerService::debug("CheckForLineStm::run()", "Line detected! ");
                    _state = CheckForLineStmState::MOVE_ROBOT;
                }
                else
                {
                    services::LoggerService::debug("CheckForLineStm::run()", "Lost Line! ");
                    // send lost line to Raspberry Hat
                    DispatcherMessage msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                              DispatcherTaskId::RaspberryHatComTask,
                                                              TaskCommand::LostLineInfo,
                                                              0);
                    xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY);

                    // move back Robot to Node
                    if (_measCounter)
                    {
                        DispatcherMessage msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                                  DispatcherTaskId::LineFollowerTask,
                                                                  TaskCommand::Move,
                                                                  (-1 * _measCounter * POLL_LINE_MOVE_INCREMENT_MM));
                        xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY);
                    }
                    reset();
                }
            }
            break;

            case (CheckForLineStmState::MOVE_ROBOT):
            {
                services::LoggerService::debug("CheckForLineStm::run()", "Send Move Robot Command");
                DispatcherMessage msg = DispatcherMessage(DispatcherTaskId::LineFollowerTask,
                                                          DispatcherTaskId::LineFollowerTask,
                                                          TaskCommand::Move,
                                                          POLL_LINE_MOVE_INCREMENT_MM);
                xQueueSend(_messageDispatcherQueue, &msg, portMAX_DELAY);
                _state = CheckForLineStmState::WAIT_FOR_STOP;
                taskYIELD();
            }
            break;

            case (CheckForLineStmState::WAIT_FOR_STOP):
                if (_posReachedFlag)
                {
                    services::LoggerService::debug("CheckForLineStm::run()", "Received PositionReached - switch state to check appearance again");
                    _state = CheckForLineStmState::CHECK_APPEARANCE;
                }
                break;

            default:
                services::LoggerService::error("CheckForLineStm::run()", "unknown State");
                _state = CheckForLineStmState::IDLE;
                break;
            }

            return true;
        }

        void CheckForLineStm::reset()
        {
            services::LoggerService::debug("CheckForLineStm::reset()", "stm reset");
            _state = CheckForLineStmState::IDLE;
            _stack.clearBuffer();
            _measCounter = 0;
            _posReachedFlag = false;
        }

        void CheckForLineStm::update(uint32_t msgData)
        {
            // WRONG METHOD !!
            services::LoggerService::fatal(" CheckForLineStm::update(uint32_t msgData)", "wrong Overload!!!");
            while (1)
            {
            }
        }

        void CheckForLineStm::update(TaskCommand cmd, uint32_t msgData)
        {
            if (cmd == TaskCommand::PollLineSensor && _state == CheckForLineStmState::IDLE)
            {
                services::LoggerService::debug("CheckForLineStm::update()", "received PollLineSensor");
                _state = CheckForLineStmState::CHECK_APPEARANCE;
            }

            if (cmd == TaskCommand::PositionReached && _state == CheckForLineStmState::WAIT_FOR_STOP)
            {
                services::LoggerService::debug("CheckForLineStm::update()", "received PositionReached");
                _posReachedFlag = true;
            }
        }
    }
}
