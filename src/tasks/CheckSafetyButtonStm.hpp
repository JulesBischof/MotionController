#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "DigitalInput.hpp"

namespace nMotionController
{
    enum class CheckSafetyButtonStmState
    {
        WAIT_FOR_BUTTON,
        BUTTON_PRESSED,
    };

    class CheckSafetyButtonStm : public StmBase<CheckSafetyButtonStmState>
    {
    private:
        DigitalInput *_safetyButton;
        QueueHandle_t _lineFollowerTaskQueue;
    public:
        CheckSafetyButtonStm(uint32_t *_statusFlags, DigitalInput *safetyButton, QueueHandle_t lineFollowerTaskQueue);
        CheckSafetyButtonStm();
        ~CheckSafetyButtonStm() override;

        void init() override;
        bool run() override;
        void reset() override;
        void update(uint32_t msgData) override;

        CheckSafetyButtonStmState getState() override { return _state; };
    };
}