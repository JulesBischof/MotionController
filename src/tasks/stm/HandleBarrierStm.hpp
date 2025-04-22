#pragma once

#include "StmBase.hpp"
#include "pico/stdlib.h"

#include "HcSr04.hpp"
#include "LineSensor.hpp"

#include "FreeRTOS.h"
#include "queue.h"


#include "DispatcherMessage.hpp"

namespace MtnCtrl
{
    namespace stm
    {
        enum class HandleBarrierStmState : uint8_t
        {
            IDLE,              // IDLE
            CHECK_DISTANCE,    // check distance
            WAIT_FOR_STOP_0,   // wait for robot to stop
            MIDDLE_ON_LINE,    // send driver command: middle on line
            WAIT_FOR_STOP_1,   // wait for robot to stop
            POSITION_DISTANCE, // send driver command: correct barrierdistance
            WAIT_FOR_STOP_2,   // wait for robot to stop
            SEND_GRIP_COMMAND, // send GC-command: grip barrier
            WAIT_FOR_GC_ACK_0, // wait for GC-ACK
            TURN_ROBOT_0,      // send driver command: turn robot
            WAIT_FOR_STOP_3,   // wait for robot to stop
            SET_BACK_ROBOT,    // send driver command: move back
            WAIT_FOR_STOP_4,   // wait for robot to stop
            SEND_RELEASE_CMD,  // send GC-command: release barrier
            WAIT_FOR_GC_ACK_1, // wait for GC-ACK
            TURN_ROBOT_1,      // send driver command: turn robot
            WAIT_FOR_STOP_5,   // wait for robot to stop
            DONE,              // send driver command: move robot
        };

        class HandleBarrierStm : public StmBase<HandleBarrierStmState>
        {
        private:
            miscDevices::HcSr04 *_hcSr04;
            miscDevices::LineSensor *_lineSensor;
            QueueHandle_t _lineFollowerTaskQueue, _messageDispatcherQueue;
            uint32_t lastMsgData;
            bool _gcAck;

        public:
            HandleBarrierStm(uint32_t *statusFlags,
                             miscDevices::HcSr04 *hcSr04,
                             miscDevices::LineSensor *lineSensor,
                             QueueHandle_t lineFollowerTaskQueue,
                             QueueHandle_t messageDispatcherQueue);
            HandleBarrierStm();
            ~HandleBarrierStm() override;

            void init() override;
            bool run() override;
            void reset() override;

            void update(uint32_t msgData) override;
            void update(uint32_t msgData, TaskCommand cmd);

            HandleBarrierStmState getState() override { return _state; };
        };
    }
}