#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "Tmc5240.hpp"

#include "DispatcherMessage.hpp"

namespace nMotionController
{
    enum class MovePositionModeStmState : uint8_t
    {
        IDLE,
        POSITION_MODE,
        TURN_MODE,
        STOP_MODE,
        WAIT_FOR_STOP,
        STOPPED,
    };

    class MovePositionModeStm : public StmBase<MovePositionModeStmState>
    {
    private:
        Tmc5240 *_driver0;
        Tmc5240 *_driver1;

        uint32_t lastMsgData;

        void _movePositionMode(int32_t distance);
        void _turnRobot(int32_t angle);
        void _stopDrives();

    public:
        MovePositionModeStm(uint32_t *_statusFlags, Tmc5240 *driver0, Tmc5240 *driver1);
        MovePositionModeStm();
        ~MovePositionModeStm() override;

        void init() override;
        bool run() override;
        void reset() override;
        void update(uint32_t msgData) override;
        void update(uint32_t msgData, TaskCommand cmd);

        MovePositionModeStmState getState() override { return _state; };
    };
}