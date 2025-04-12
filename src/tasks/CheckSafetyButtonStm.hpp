#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "FreeRTOS.h"
#include "task.h"

#include "DigitalInput.hpp"

namespace MotionController
{
    class CheckSafetyButtonStm : public StmBase<CheckSafetyButtonStm::State>
    {
    private:
        DigitalInput *_safetyButton;
        QueueHandle_t _lineFollowerTaskQueue;

        enum class State
        {
            WAIT_FOR_BUTTON,
            BUTTON_PRESSED,
        };

    public:
        CheckSafetyButtonStm(uint32_t *_statusFlags, DigitalInput *safetyButton, QueueHandle_t lineFollowerTaskQueue);
        ~CheckSafetyButtonStm() override;

        void init() override;
        bool run() override;
        void reset() override;
        void update(uint32_t msgData) override;

        State getState() override { return _state; };
    };
}