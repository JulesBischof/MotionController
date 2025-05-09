#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "Tmc5240.hpp"
#include "LineSensor.hpp"
#include "DispatcherMessage.hpp"

#include "FreeRTOS.h"
#include "queue.h"


namespace MtnCtrl
{
    namespace stm
    {
        enum class LineFollowerStmState : uint8_t
        {
            IDLE,
            FOLLOW_LINE,
            LOST_LINE,
            CROSSPOINT_DETECTED,
        };

        class LineFollowerStm : public StmBase<LineFollowerStmState>
        {
        private:
            spiDevices::Tmc5240 *_driver0;
            spiDevices::Tmc5240 *_driver1;
            miscDevices::LineSensor *_lineSensor;

            absolute_time_t _stopTimeStamp;

            QueueHandle_t _lineFollowerTaskQueue,
                _messageDispatcherQueue;
            bool _slowFlag;

            void _followLine();
            int32_t _controllerC(int32_t e);
            void _checkLineFollowerStatus();

            uint32_t _lastMsgData;

        public:
            LineFollowerStm(miscDevices::LineSensor *lineSensor,
                            spiDevices::Tmc5240 *driver0,
                            spiDevices::Tmc5240 *driver1,
                            QueueHandle_t LineFollowerTaskQueue,
                            QueueHandle_t MessageDispatcherQueue);
            LineFollowerStm();
            ~LineFollowerStm() override;

            void init() override;
            bool run() override;
            void reset() override;
            void update(uint32_t msgData) override;
            void update(uint32_t msgData, TaskCommand cmd);

                LineFollowerStmState getState() override
            {
                return _state;
            };
        };
    }
}