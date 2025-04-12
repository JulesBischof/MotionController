#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

#include "FreeRTOS.h"
#include "queue.h"

namespace MotionController
{
    class LineFollowerStm : public StmBase<LineFollowerStm::State>
    {
    private:
        Tmc5240*_driver0;
        Tmc5240*_driver1;
        LineSensor *_lineSensor;

        QueueHandle_t _lineFollowerTaskQueue;

        void _followLine();
        int32_t _controllerC(int32_t e);
        void _checkLineFollowerStatus();

        uint32_t lastMsgData;

        enum class State
        {
            IDLE,
            FOLLOW_LINE,
            LOST_LINE,
            CROSSPOINT_DETECTED,
        };

    public:
        LineFollowerStm(uint32_t *_statusFlags, LineSensor *lineSensor, Tmc5240 *driver0, Tmc5240 *driver1, QueueHandle_t LineFollowerTaskQueue);
        ~LineFollowerStm() override;

        void init() override;
        bool run() override;
        void reset() override;
        void update(uint32_t msgData) override;

        State getState() override { return _state; };
    };
}