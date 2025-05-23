#include "StmBase.hpp"
#include "LineSensor.hpp"
#include "DispatcherMessage.hpp"
#include "MedianStack.hpp"
#include "FreeRTOS.h"
#include "queue.h"

namespace MtnCtrl
{
    namespace stm
    {
        enum class CheckForLineStmState : uint8_t
        {
            IDLE,
            CHECK_APPEARANCE,
            MOVE_ROBOT,
            WAIT_FOR_STOP
        };

        /// @brief statemaschine that performs some line-sweep-linedetection movement in order to determine if there is a line underneath the line sensor
        class CheckForLineStm : StmBase<CheckForLineStmState>
        {
        private:
            CheckForLineStmState _state;
            bool _posReachedFlag;

            uint32_t _measCounter;
            miscDevices::MedianStack<uint16_t> _stack;

            miscDevices::LineSensor *_lineSensor;
            QueueHandle_t _messageDispatcherQueue;

        public:
            CheckForLineStm(miscDevices::LineSensor *lineSensor, QueueHandle_t messageDispatcherQueue);

            void init();
            bool run();
            void reset();

            /// @brief not implemented overload
            /// @param msgData 
            void update(uint32_t msgData);

            /// @brief commonly prefered overload - updates state of the statemaschine dependent on several inputs such as STOP / POLL_LINE
            /// @param cmd Task Command from DispatcherMessage
            /// @param msgData Message Params
            void update(TaskCommand cmd, uint32_t msgData);
            CheckForLineStmState getState() { return _state; };
        };
    }
}