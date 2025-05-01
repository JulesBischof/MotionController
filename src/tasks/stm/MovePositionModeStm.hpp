#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"

#include "DispatcherMessage.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        enum class MovePositionModeStmState : uint8_t
        {
            IDLE,
            POSITION_MODE,
            TURN_MODE,
            STOP_MODE,
            MIDDLE_ON_LINE_MODE,
            WAIT_FOR_STOP,
            STOPPED,
        };

        class MovePositionModeStm : public StmBase<MovePositionModeStmState>
        {
        private:
            spiDevices::Tmc5240 *_driver0;
            spiDevices::Tmc5240 *_driver1;
            miscDevices::LineSensor *_lineSensor;

            QueueHandle_t _messageDispatcherQueue;

            uint32_t _lastMsgData;
            absolute_time_t _stoppedTimeStamp;

            void _movePositionMode(int32_t distance);
            void _turnRobot(int32_t angle);
            void _stopDrives();
            bool _checkDriversForStandstill();

        public:
            MovePositionModeStm(uint32_t *_statusFlags,
                                spiDevices::Tmc5240 *driver0,
                                spiDevices::Tmc5240 *driver1,
                                miscDevices::LineSensor *lineSensor,
                                QueueHandle_t messageDispatcherQueue);

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
}