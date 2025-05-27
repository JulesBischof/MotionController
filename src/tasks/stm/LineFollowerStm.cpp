#include "LineFollowerStm.hpp"
#include "LineFollowerTaskConfig.hpp"
#include "LoggerService.hpp"
#include "DispatcherMessage.hpp"
#include "pico/stdlib.h"
#include <stdio.h>

namespace MtnCtrl
{
    namespace stm
    {
        LineFollowerStm::LineFollowerStm(miscDevices::LineSensor *lineSensor,
                                         spiDevices::Tmc5240 *driver0,
                                         spiDevices::Tmc5240 *driver1,
                                         QueueHandle_t LineFollowerTaskQueue,
                                         QueueHandle_t MessageDispatcherQueue)
            : StmBase(),
              _lineSensor(lineSensor),
              _driver0(driver0),
              _driver1(driver1),
              _lineFollowerTaskQueue(LineFollowerTaskQueue),
              _messageDispatcherQueue(MessageDispatcherQueue)
        {
        }

        LineFollowerStm::LineFollowerStm()
        {
        }

        LineFollowerStm::~LineFollowerStm()
        {
        }

        void LineFollowerStm::init()
        {
            _lastMsgData = 0;
            _state = LineFollowerStmState::IDLE;
        }

        bool LineFollowerStm::run()
        {
            // now run state maschine
            bool retVal = false;

            DispatcherMessage msg;

            switch (_state)
            {
            case LineFollowerStmState::IDLE:
                retVal = true;
                break;

            case LineFollowerStmState::FOLLOW_LINE:
                _followLine();

                if (_lineSensor->getStatus() & miscDevices::LINESENSOR_CROSS_DETECTED)
                {
                    services::LoggerService::info("LineFollowerStm::run()", "Node Detected!");
                    _state = LineFollowerStmState::CROSSPOINT_DETECTED;
                }
                if (_lineSensor->getStatus() & miscDevices::LINESENSOR_NO_LINE)
                {
                    services::LoggerService::info("LineFollowerStm::run()", "Lost Line!");
                    _state = LineFollowerStmState::LOST_LINE;
                }

                break;

            case LineFollowerStmState::LOST_LINE:
                // stop motors every task
                services::LoggerService::debug("LineFollowerStm::run() state#LOST_LINE", "send Stop Cmd");
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::Broadcast,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("LineFollowerStm::run() state#LOST_LINE", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }

                // send info to RaspberryHat
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::RaspberryHatComTask,
                    TaskCommand::LostLineInfo,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("LineFollowerStm::run() state#LOST_LINE", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                break;

            case LineFollowerStmState::CROSSPOINT_DETECTED:
                // move in positionmode on top of node
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm - (LINEFOLLOWERCONFIG_DISTANCE_PER_LINE_POLLCYCLE_MM * RINGBUFFER_SIZE));
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("LineFollowerStm::run() state#CROSSPOINT_DETECTED", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }

                // send driven Distance to RaspberryHat - infoflag = driven distance now
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::PollDistance,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("LineFollowerStm::run() state#CROSSPOINT_DETECTED", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }

                // stop barrier detection
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::BarrierHandlerTask,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_messageDispatcherQueue, &msg, pdMS_TO_TICKS(1000)) != pdPASS)
                { /* ERROR!!?? */
                    services::LoggerService::fatal("LineFollowerStm::run() state#CROSSPOINT_DETECTED", "_messagDispatcherQueue TIMEOUT");
                    while (1)
                    {
                    }
                }
                reset();
                break;

            default:
                /* ERROR..? */
                break;
            }

            return retVal;
        }

        void LineFollowerStm::reset()
        {
            services::LoggerService::debug("LineFollowerStm::reset()", "reset lineFollowerStm");
            _slowFlag = false;
            _state = LineFollowerStmState::IDLE;
            _lineSensor->toggleUvLed(false);
            _lineSensor->reset();
        }

        void LineFollowerStm::update(uint32_t msgData)
        {
            services::LoggerService::fatal("LineFollwerStm::update(uint32_t msgData);", "not implemented");
            while (1)
            {
            }
        }

        void LineFollowerStm::update(uint32_t msgData, TaskCommand cmd)
        {
            _slowFlag = (cmd == TaskCommand::SlowDown) ? true : false;

            if (msgData == 0 && cmd == TaskCommand::Move)
            {
                _lineSensor->reset(); // clear buffers and counters for node detection
                _lineSensor->toggleUvLed(true);
                services::LoggerService::debug("LineFollowerStm::update()", "start FOLLOW_LINE");
                _state = LineFollowerStmState::FOLLOW_LINE;
            }
        }

        /// @brief controller for linefollowing. Config behaviour by settings values in LineFollwoerConfig.h file
        void LineFollowerStm::_followLine()
        {
            // init vars
            int32_t v1 = 0;
            int32_t v2 = 0;

            // decide for speedmode
            uint32_t vmax = (_slowFlag == true) ? LINEFOLLERCONFIG_VMAX_REGISTER_VALUE_SLOW : LINEFOLLERCONFIG_VMAX_REGISTER_VALUE;

            // get Sensor values
            int16_t y = _lineSensor->getLinePositionAnalog();

#if ENABLE_DATA_OUTPUT_LINEPOS == (1)
            printf("%d\n", y);
#endif
            // get error
            int16_t e = LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG - y;

            // calc control variable
            int32_t u = 0;
            u = _controllerC(e);

            // set Motor values (Process P)
            v1 = vmax + u;
            v2 = vmax - u;

            // set Motor values
            _driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE);
            _driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_REGISTER_VALUE);
        } // end followLine

        /// @brief Controller Method for LineFollowing
        /// @param e error as input
        /// @return returns controller parameter u as output
        int32_t LineFollowerStm::_controllerC(int32_t e)
        {
#if LINEFOLLERCONFIG_USE_P_CONTROLLER == (1)
            // P-Type Controller
            int32_t u = 0; // default 0
            u = e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP;
#endif

#if LINEFOLLERCONFIG_USE_PD_CONTROLLER == (1)
            static int32_t last_e = 0;
            int32_t u = 0; // default 0

            // getTimeDifference in s
            static absolute_time_t lastTime = 0;
            absolute_time_t now = get_absolute_time();
            float deltaT = absolute_time_diff_us(lastTime, now) / 1e6; // s

            int32_t deltaE = (e - last_e) / deltaT;
            last_e = e;

            u = LINEFOLLERCONFIG_CONTROLLERVALUE_KP * e + LINEFOLLERCONFIG_CONTROLLERVALUE_KD * deltaE;
#endif

#if LINEFOLLERCONFIG_USE_PD_CONTROLLER_MATLAB == (1)
            static int32_t last_e = 0;
            static double last_u = 0;

            double res = cz_matlab_num[0] * e + cz_matlab_num[1] * last_e - cz_matlab_dnum[1] * last_u;
            last_u = res;
            last_e = e;

            int32_t u = static_cast<int32_t>(res);
#endif
            return u;
        } // end ControllerC
    }
}