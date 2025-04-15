#include "CheckSafetyButtonStm.hpp"

#include "DispatcherMessage.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        CheckSafetyButtonStm::CheckSafetyButtonStm(uint32_t *_statusFlags, miscDevices::DigitalInput *safetyButton, QueueHandle_t lineFollowerTaskQueue) : StmBase(_statusFlags),
                                                                                                                                                           _safetyButton(safetyButton),
                                                                                                                                                           _lineFollowerTaskQueue(lineFollowerTaskQueue)
        {
        }

        CheckSafetyButtonStm::CheckSafetyButtonStm()
        {
        }

        CheckSafetyButtonStm::~CheckSafetyButtonStm()
        {
        }

        void CheckSafetyButtonStm::init()
        {
            _state = CheckSafetyButtonStmState::WAIT_FOR_BUTTON;
        }

        bool CheckSafetyButtonStm::run()
        {
            bool retVal = false;
            switch (_state)
            {
            case CheckSafetyButtonStmState::WAIT_FOR_BUTTON:
                if (!_safetyButton->getValue())
                {
                    _state = CheckSafetyButtonStmState::BUTTON_PRESSED;
                    retVal = true;
                }
                break;

            case CheckSafetyButtonStmState::BUTTON_PRESSED:
                // stop drives
                *_statusFlags |= (uint32_t)RunModeFlag::SAFETY_BUTTON_PRESSED;
                DispatcherMessage msg(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                break;
            }
            return retVal;
        }

        void CheckSafetyButtonStm::reset()
        {
            _state = CheckSafetyButtonStmState::WAIT_FOR_BUTTON;
        }

        void CheckSafetyButtonStm::update(uint32_t msgData)
        {
            // nothing to do in here
        }
    }
}