#include "LineFollowerStm.hpp"

#include "LineFollowerTaskConfig.h"
#include "LineFollowerTaskStatusFlags.hpp"

#include "DispatcherMessage.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

namespace MtnCtrl
{
    namespace stm
    {
        LineFollowerStm::LineFollowerStm(uint32_t *_statusFlags, miscDevices::LineSensor *lineSensor, spiDevices::Tmc5240 *driver0, spiDevices::Tmc5240 *driver1, QueueHandle_t LineFollowerTaskQueue)
            : StmBase(_statusFlags), _lineSensor(lineSensor), _driver0(driver0), _driver1(driver1), _lineFollowerTaskQueue(LineFollowerTaskQueue)
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
            lastMsgData = 0;
            _state = LineFollowerStmState::IDLE;
        }

        bool LineFollowerStm::run()
        {
            // make sure that the state machine is not called too often
            static TickType_t lastRunTick = 0;
            TickType_t now = xTaskGetTickCount();
            if ((now - lastRunTick) < pdMS_TO_TICKS(LINEFOLLOWERTASKCONFIG_HCSR04_POLLING_RATE_MS))
            {
                return false;
            }
            lastRunTick = now;

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
                    _state = LineFollowerStmState::CROSSPOINT_DETECTED;
                    *_statusFlags |= (uint32_t)RunModeFlag::CROSSPOINT_DETECTED;
                }
                if (_lineSensor->getStatus() & miscDevices::LINESENSOR_NO_LINE)
                {
                    _state = LineFollowerStmState::LOST_LINE;
                    *_statusFlags |= (uint32_t)RunModeFlag::LOST_LINE;
                }

                break;

            case LineFollowerStmState::LOST_LINE:
                // stop motors
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Stop,
                    0);
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
                }

                _state = LineFollowerStmState::IDLE;
                break;

            case LineFollowerStmState::CROSSPOINT_DETECTED:
                // move in positionmode on top of node
                msg = DispatcherMessage(
                    DispatcherTaskId::LineFollowerTask,
                    DispatcherTaskId::LineFollowerTask,
                    TaskCommand::Move,
                    LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm);
                if (xQueueSend(_lineFollowerTaskQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS)
                { /* ERROR!!?? */
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
            _state = LineFollowerStmState::IDLE;
        }

        void LineFollowerStm::update(uint32_t msgData)
        {
            if (msgData == 0)
            {
                _state = LineFollowerStmState::FOLLOW_LINE;
            }
        }

        /// @brief controller for linefollowing. Config behaviour by settings values in LineFollwoerConfig.h file
        void LineFollowerStm::_followLine()
        {
            // init vars
            uint32_t statusFlags = *_statusFlags;
            int32_t v1 = 0;
            int32_t v2 = 0;

            // get Sensor values
#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
            int16_t y = _lineSensor->getLinePositionDigital();
#else
            int16_t y = _lineSensor->getLinePositionAnalog();
#endif

#if ENABLE_DATA_OUTPUT_LINEPOS == (1)
            printf("%d\n", y);
#endif

            // get error
            int16_t e = 0;

#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
            e = LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL - y;
#else
            e = LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG - y;
#endif
            // calc control variable
            int32_t u = 0;
            u = _controllerC(e);

            // set Motor values (Process P)
            v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC + u;
            v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC - u;
            

            // set Motor values
            _driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
            _driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
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

#if LINEFOLLERCONFIG_USE_PD_CONTROLLER_DIGITAL == (1)
            static int32_t last_e = 0;
            // D-Controller with low Pass Filter
            int32_t u = 0; // default 0

            int32_t deltaE = (e - last_e) / LINEFOLLOWERCONFIG_POLLING_RATE_MS;
            last_e = e;

            u = LINEFOLLERCONFIG_CONTROLLERVALUE_KP * e + LINEFOLLERCONFIG_CONTROLLERVALUE_KD * deltaE;
#endif

#if LINEFOLLERCONFIG_USE_PD_CONTROLLER_Z_TRANSFORM == (1)
            static double last_e = 0;
            static double last_u = 0;

            double res = LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_A1 * e +
                         LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_A2 * last_e -
                         LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_B2 * last_u;

            int32_t u = static_cast<int32_t>(res);

            last_e = e;
            last_u = u;
#endif
            int32_t retVal = u * LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P;
            return retVal;
        } // end ControllerC
    }
}