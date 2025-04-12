#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "queue.h"

namespace nMotionController
{
    enum class SendStatusFlagsStmState : uint8_t
    {
        WAIT_FOR_FLAGS,
        SEND_FLAGS,
    };

    class SendStatusFlagsStm : public StmBase<SendStatusFlagsStmState>
    {
    private:
        QueueHandle_t _messageDispatcherQueue;

    public:
        SendStatusFlagsStm(uint32_t *_statusFlags, QueueHandle_t messageDispatcherQueue);
        SendStatusFlagsStm();
        ~SendStatusFlagsStm() override;

        void init() override;
        bool run() override;
        void reset() override;
        void update(uint32_t msgData) override;

        SendStatusFlagsStmState getState() override { return _state; };
    };
}