#include "LineFollowerTask.hpp"

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"
#include "LineFollowerTaskConfig.h"

#include "TMC5240_HW_Abstraction.h"
#include "Tmc5240Config.h"
#include "Tmc5240.hpp"

#include "queues.h"

#include "DigitalInput.hpp"
#include "Tla2528.hpp"

#include <stdio.h>
#include <cmath>

/* ================================= */
/*         static Members            */
/* ================================= */

LineFollowerTask *LineFollowerTask::_instance;
TaskHandle_t LineFollowerTask::_taskHandle;
QueueHandle_t LineFollowerTask::_dispatcherQueue;
QueueHandle_t LineFollowerTask::_lineFollowerQueue;
uint32_t LineFollowerTask::_statusFlags;

Tmc5240 *LineFollowerTask::_driver0;
Tmc5240 *LineFollowerTask::_driver1;
Tla2528 *LineFollowerTask::_adc;
LineSensor *LineFollowerTask::_lineSensor;
DigitalInput *LineFollowerTask::_safetyButton;

/* ================================= */
/*           status Flags            */
/* ================================= */

constexpr uint32_t RUNMODEFLAG_T_UPPER_BITMASK(0xFFFF0000);
constexpr uint32_t RUNMODEFLAG_T_LOWER_BITMASK(0x0000FFFF);

enum RunModeFlag_t
{
    // lower 16 bits statemaschine relevant flags
    MOTOR_RUNNING = 1 << 0,
    MOTOR_POSITIONMODE = 1 << 1,
    MOTOR_POSITIONMODE_REQUEST_SEND = 1 << 2,
    MOTOR_STOPREQUEST_SEND = 1 << 3,
    RUNMODE_SLOW = 1 << 4,
    LINE_FOLLOWER_MODE = 1 << 5,
    TURN_MODE = 1 << 6,
    TURNREQUEST_SEND = 1 << 7,
    STATUSFLAGS_SEND = 1 << 8,

    // upper 16 bits events and infos
    CROSSPOINT_DETECTED = 1 << 15,
    LOST_LINE = 1 << 16,
    POSITION_REACHED = 1 << 17,
    SAFETY_BUTTON_PRESSED = 1 << 18
} RunModeFlag_t;

constexpr uint32_t STM_LINEFOLLOWER_BITSET = 0 | (MOTOR_RUNNING | LINE_FOLLOWER_MODE);
constexpr uint32_t STM_MOVE_POSITIONMODE_BITSET = 0 | (MOTOR_RUNNING | MOTOR_POSITIONMODE);
constexpr uint32_t STM_STOPMOTOR_BITSET = 0;
constexpr uint32_t STM_TURNROBOT_BITSET = 0 | (TURN_MODE | MOTOR_RUNNING);

/* ================================= */
/*           Definition              */
/* ================================= */

/// @brief creates an instance of LineFollower Task and creates Linefollower Task
/// @param dispatcherQueue queue message dispatcher
/// @param lineFollowerQueue queue line follower queue
LineFollowerTask::LineFollowerTask(QueueHandle_t dispatcherQueue, QueueHandle_t lineFollowerQueue)
{
    _dispatcherQueue = dispatcherQueue;
    _lineFollowerQueue = lineFollowerQueue;

    _initDevices();
    _statusFlags = 0;

    if (xTaskCreate(_run, LINEFOLLOWERTASK_NAME, LINEFOLLOWERCONFIG_STACKSIZE/sizeof(StackType_t), NULL, LINEFOLLOWERCONFIG_PRIORITY, &_taskHandle) != pdTRUE)
    {
        for (;;)
            ;
        /* ERROR Todo: error handling */
    }
} // end ctor

LineFollowerTask::~LineFollowerTask()
{
    if (_instance != nullptr)
    {
        // delete memory of static member objects
        delete _driver0;
        delete _driver1;
        delete _adc;
        delete _lineSensor;
        delete _safetyButton;
        delete _instance;
    }
}; // end dctor

void LineFollowerTask::_run(void *pvParameters)
{
    // Variables
    int32_t X_ACTUAL_startValueDriver0 = _driver0->getXActual();
    int32_t drivenDistanceDriver0 = 0;

    int32_t X_ACTUAL_startValueDriver1 = _driver1->getXActual();
    int32_t drivenDistanceDriver1 = 0;

    uint32_t maxDistance = 0;

    // loop forever
    for (;;)
    {
        dispatcherMessage_t message;

        if (uxQueueMessagesWaiting(_lineFollowerQueue) > 0)
        {
            xQueueReceive(_lineFollowerQueue, &message, pdMS_TO_TICKS(10));

            if (message.recieverTaskId != TASKID_LINE_FOLLOWER_TASK)
            {
                printf("LINEFOLLOWERTASK - Message contains wrong Task ID \n");
                continue;
            }

            dispatcherMessage_t response;

            switch (message.command)
            {
            case COMMAND_MOVE:
                _statusFlags &= ~POSITION_REACHED;

                X_ACTUAL_startValueDriver0 = _driver0->getXActual();
                X_ACTUAL_startValueDriver1 = _driver1->getXActual();

                // move infinit as Line Follower
                if (message.data == 0)
                {
                    maxDistance = 0; // TODO: 2m in usteps
                    _statusFlags = STM_LINEFOLLOWER_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                }

                // move in Position Mode
                if (message.data != 0)
                {
                    maxDistance = Tmc5240::meterToUStepsConversion(message.data) / 1e2; // 1e2 due to distance gets send in cm to avoid floats in protocoll
                    _statusFlags = STM_MOVE_POSITIONMODE_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                }
                break;

            case COMMAND_STOP:
                _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                break;

            case COMMAND_TURN:
                _statusFlags &= ~POSITION_REACHED;
                _statusFlags &= ~TURNREQUEST_SEND;
                _statusFlags = STM_TURNROBOT_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                break;

            case COMMAND_POLL_DISTANCE:
                // TODO
                break;

            case COMMAND_POLL_LINE_SENSOR:
                // TODO
                break;

            case COMMAND_POLL_STATUSFLAGS:
                response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                            message.senderTaskId,
                                            COMMAND_POLL_STATUSFLAGS,
                                            (uint32_t)_statusFlags);
                xQueueSend(_dispatcherQueue, &response, 0);
                break;

            default:
                // error! command not found assert() ?
                break;
            }
        } // end of message handling

        /// ------- stm check safetybutton -------
        if (!_safetyButton->getValue())
        {
            _statusFlags &= ~MOTOR_RUNNING;
            _statusFlags | SAFETY_BUTTON_PRESSED;
        }
        else
        {
            _statusFlags &= ~SAFETY_BUTTON_PRESSED;
        }

        // ------- stm check hcsr04 distance -------

        // TODO: check distance

        // ------- stm line follower -------
        if ((_statusFlags & STM_LINEFOLLOWER_BITSET) == STM_LINEFOLLOWER_BITSET)
        {
            _followLine();
            drivenDistanceDriver0 = _driver0->getXActual() - X_ACTUAL_startValueDriver0;
            drivenDistanceDriver1 = _driver1->getXActual() - X_ACTUAL_startValueDriver1;

            // maxdistance reached - send Info to RaspberryHAT
            if ((maxDistance != 0 && drivenDistanceDriver0 > maxDistance) ||
                (maxDistance != 0 && drivenDistanceDriver1 > maxDistance))
            {
                _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                _statusFlags |= POSITION_REACHED;
            }
        }

        // ------- stm move Positionmode -------
        if ((_statusFlags & STM_MOVE_POSITIONMODE_BITSET) == STM_MOVE_POSITIONMODE_BITSET)
        {
            if (!(_statusFlags & MOTOR_POSITIONMODE_REQUEST_SEND))
            {
                _statusFlags |= MOTOR_POSITIONMODE_REQUEST_SEND;
                _movePositionMode(message.data);
            }

            if (_checkForStandstill())
            {
                _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                _statusFlags |= POSITION_REACHED;
            }
        }

        /// ------- stm turn vehicle -------
        if ((_statusFlags & STM_TURNROBOT_BITSET) == STM_TURNROBOT_BITSET)
        {
            _turnRobot(message.data);

            if (_checkForStandstill())
            {
                _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_LOWER_BITMASK);
                _statusFlags |= POSITION_REACHED;
            }
        }

        /// ------- stm check and send info flags -------
        if ((_statusFlags & RUNMODEFLAG_T_UPPER_BITMASK) && !(_statusFlags & STATUSFLAGS_SEND))
        {
            dispatcherMessage_t response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                                            TASKID_RASPBERRY_HAT_COM_TASK,
                                                            COMMAND_INFO,
                                                            (uint32_t)_statusFlags);
            _statusFlags |= STATUSFLAGS_SEND;
            xQueueSend(_dispatcherQueue, &response, 0);
        }

        /// ------- stm stop drives -------
        if (!(_statusFlags & MOTOR_RUNNING))
        {
            _stopDrives();
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
} // end vLineFollowerTask

/// @brief initializes device members such as sensors and drivers
void LineFollowerTask::_initDevices()
{
    // initialize peripherals neccessary for line follower
    _driver0 = new Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
    _driver1 = new Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);

    // init adc device and line Sensor
    _adc = new Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
    _lineSensor = new LineSensor(_adc, UV_LED_GPIO);

    // int safety button
    _safetyButton = new DigitalInput(DIN_4);
}

bool LineFollowerTask::_checkForStandstill()
{
    uint8_t status_driver0 = _driver0->getStatus();
    uint8_t status_driver1 = _driver1->getStatus();

    if ((status_driver0 & TMC5240_SPI_STATUS_POSITION_REACHED_MASK) &&
        (status_driver1 & TMC5240_SPI_STATUS_POSITION_REACHED_MASK))
    {
        return true;
    }
    return false;
}

void LineFollowerTask::_movePositionMode(int32_t distance)
{
    _driver0->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _driver1->moveRelativePositionMode(distance, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
}

void LineFollowerTask::_turnRobot(int32_t angle)
{
    // check if turn signal got send already
    if (_statusFlags & TURNREQUEST_SEND)
    {
        return;
    }
    // printf("Stack high water mark: %u\n", uxTaskGetStackHighWaterMark(_taskHandle));

    // Tmc5240::degreeToUStepsConversion(angle) / 10; // divide by 10 due to unit conversion - angle gets send in (° * 10) to avoid float
    // do some math - get microsteps
    double dAngle = static_cast<double>(angle);
    double rawVal = (MICROSTEPS_PER_REVOLUTION / 3600.0f) * dAngle; // magic number - steps per revolution divided by 360°

    // int32_t nStepsDriver = 25599;
    int32_t nStepsDriver = static_cast<int32_t>(std::round(rawVal));

    // move drives in different directions
    _driver0->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _driver1->moveRelativePositionMode(nStepsDriver, LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST * 2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _statusFlags |= TURNREQUEST_SEND;
}
void LineFollowerTask::_followLine()
{
    // init vars
    bool speedMode = (_statusFlags & RUNMODE_SLOW) ? true : false;

    int32_t v1 = 0;
    int32_t v2 = 0;

    // get Sensor values
#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
    int8_t y = lineSensor.getLinePosition();
#else
    uint16_t y = _lineSensor->getLinePositionAnalog();
#endif

    // check for LineSensor events
    if (_lineSensor->getStatus() & LINESENSOR_CROSS_DETECTED)
    {
        _statusFlags |= CROSSPOINT_DETECTED;
        _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_UPPER_BITMASK);

        printf("LINESENSOR detected Crosspoint \n");
        return;
    }
    if (_lineSensor->getStatus() & LINESENSOR_NO_LINE)
    {
        _statusFlags |= LOST_LINE;
        _statusFlags = STM_STOPMOTOR_BITSET | (_statusFlags & RUNMODEFLAG_T_UPPER_BITMASK);

        printf("LINESENSOR lost Line \n");
        return;
    }

    // TODO: HCSR04 get distance

    // get error
    int8_t e = 0;

#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
    e = LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL - y;
#else
    e = LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG - y;
#endif

    // calc control variable
    int32_t u = 0;
    u = _controllerC(e);

    // set Motor values (Process P)
    if (!speedMode)
    {
        v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST + u;
        v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST - u;
    }
    else
    {
        v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW + u;
        v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW - u;
    }

    // set Motor values
    _driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
} // end followLine

int32_t LineFollowerTask::_controllerC(int8_t e)
{
#if LINEFOLLERCONFIG_USE_P_CONTROLLER == 1
    // P-Type Controller
    int32_t u = 0; // default 0
    u = e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP;
#endif
    int32_t retVal = u * LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P;
    return retVal;
} // end Control

void LineFollowerTask::_stopDrives()
{
    if (_statusFlags & MOTOR_STOPREQUEST_SEND)
    {
        return;
    }
    _driver0->moveVelocityMode(1, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _driver1->moveVelocityMode(0, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _statusFlags |= MOTOR_STOPREQUEST_SEND;

    return;
} // end stop Drives

/* ================================= */
/*              getters              */
/* ================================= */

/// @brief signleton pattern
/// @param dispatcherQueue Queue of dispatcher Task
/// @param lineFollowerQueue Queue of LineFollower Task
/// @return instance of LineFollowerTask
LineFollowerTask LineFollowerTask::getInstance(QueueHandle_t messageDispatcherQueue, QueueHandle_t lineFollowerQueue)
{
    if (_instance == nullptr)
    {
        _instance = new LineFollowerTask(messageDispatcherQueue, lineFollowerQueue);
    }
    return *_instance;
}

QueueHandle_t LineFollowerTask::getQueue()
{
    return _lineFollowerQueue;
}

TaskHandle_t LineFollowerTask::getTaskHandle()
{
    return _taskHandle;
}
