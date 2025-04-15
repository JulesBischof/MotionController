#include "SendStatusFlagsStm.hpp"

#include "DispatcherMessage.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        SendStatusFlagsStm::SendStatusFlagsStm(uint32_t *_statusFlags, QueueHandle_t messageDispatcherQueue)
            : StmBase(_statusFlags), _messageDispatcherQueue(messageDispatcherQueue)
        {
            init();
        }

        SendStatusFlagsStm::SendStatusFlagsStm()
        {
        }

        SendStatusFlagsStm::~SendStatusFlagsStm()
        {
        }

        void SendStatusFlagsStm::init()
        {
            _state = SendStatusFlagsStmState::WAIT_FOR_FLAGS;
        }

        bool SendStatusFlagsStm::run()
        {
            bool retVal = false;
            DispatcherMessage msg;

            switch (_state)
            {
            case SendStatusFlagsStmState::WAIT_FOR_FLAGS:
                // wait for flags
                retVal = true;
                break;
            case SendStatusFlagsStmState::SEND_FLAGS:
                // send flags
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::RaspberryHatComTask,
                    TaskCommand::Info,
                    *_statusFlags);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }
                *_statusFlags = 0;
                _state = SendStatusFlagsStmState::WAIT_FOR_FLAGS;
                break;
            default:
                /* ERROR..? */
                break;
            }
            return false;
        }
        void SendStatusFlagsStm::reset()
        {
            _state = SendStatusFlagsStmState::WAIT_FOR_FLAGS;
        }
        void SendStatusFlagsStm::update(uint32_t msgData)
        {
            // empty
        }
    }
}