#include "LineFollowerTask.hpp"

#include "MotionControllerPinning.h"
#include "MotionControllerConfig.h"
#include "LineFollowerTaskConfig.h"

#include "queues.hpp"

#include "digitalInput.hpp"
#include "Tla2528.hpp"

#include <stdio.h>


/* ================================= */
/*         static Members            */
/* ================================= */

LineFollowerTask *LineFollowerTask::_instance = nullptr;
TaskHandle_t LineFollowerTask::_taskHandle = nullptr;
QueueHandle_t LineFollowerTask::_dispatcherQueue = nullptr;
QueueHandle_t LineFollowerTask::_lineFollowerQueue = nullptr;

/* ================================= */
/*           status Flags            */
/* ================================= */

#define RUNMODEFLAG_T_EVENTFLAGS_BITMASK (0xFFFF0000)
typedef enum RunModeFlag_t
{
    // lower 16 bits statemaschine relevant flags
    MOTOR_RUNNING = 1 << 0,
    RUNMODE_SLOW = 1 << 1,
    LINE_FOLLOWER_MODE = 1 << 2,
    TURN_MODE = 1 << 3,

    // upper 16 bits events and infos
    CROSSPOINT_DETECTED = 1 << 15,
    LOST_LINE = 1 << 16,
    MAXDISTANCE_REACHED = 1 << 17,
    SAFETY_BUTTON_PRESSED = 1 << 18
} RunModeFlag_t;

/* ================================= */
/*           Definition              */
/* ================================= */

/// @brief creates an instance of LineFollower Task and creates Linefollower Task
/// @param dispatcherQueue queue message dispatcher
/// @param lineFollowerQueue queue line follower queue
LineFollowerTask::LineFollowerTask(QueueHandle_t *dispatcherQueue)
{
    _dispatcherQueue = *dispatcherQueue;

    _lineFollowerQueue = xQueueCreate(LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS, sizeof(dispatcherMessage_t));

    _initDevices();
    _statusFlags = 0;

    xTaskCreate(_run, LINEFOLLOWERTASK_NAME, LINEFOLLOWERCONFIG_STACKSIZE, this, LINEFOLLOWERCONFIG_PRIORITY, &_taskHandle);
} // end ctor

#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERCONFIG_STACKSIZE (1024)
#define LINEFOLLOWERCONFIG_PRIORITY (1)

LineFollowerTask::~LineFollowerTask()
{
    if (_instance != nullptr)
    {
        delete _instance;
    }
}; // end dctor

void LineFollowerTask::_run(void *pvParameters)
{
    // Variables
    int32_t X_ACTUAL_startValueDriver0 = _driver0.getXActual();
    int32_t drivenDistanceDriver0 = 0;

    int32_t X_ACTUAL_startValueDriver1 = _driver1.getXActual();
    int32_t drivenDistanceDriver1 = 0;

    uint32_t maxDistance = 0;

    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(_lineFollowerQueue) > 0)
        {
            dispatcherMessage_t message;
            xQueueReceive(_lineFollowerQueue, &message, pdMS_TO_TICKS(10));

            if (message.recieverTaskId != TASKID_LINE_FOLLOWER_TASK)
            {
                printf("LINEFOLLOWERTASK - Message contains wrong Task ID \n");
                continue;
            }

            dispatcherMessage_t response;

            switch (message.command)
            {
            case COMMAND_FOLLOW_LINE:
                _statusFlags &= ~TURN_MODE;
                _statusFlags &= ~MAXDISTANCE_REACHED;
                _statusFlags &= ~RUNMODE_SLOW;
                _statusFlags &= ~LOST_LINE;
                _statusFlags &= ~CROSSPOINT_DETECTED;
                _statusFlags |= LINE_FOLLOWER_MODE;
                _statusFlags |= MOTOR_RUNNING;

                X_ACTUAL_startValueDriver0 = _driver0.getXActual();
                X_ACTUAL_startValueDriver1 = _driver1.getXActual();
                break;

            case COMMAND_STOP:
                _statusFlags &= ~MOTOR_RUNNING;
                break;

            case COMMAND_TURN:
                _statusFlags &= ~LINE_FOLLOWER_MODE;
                _statusFlags |= TURN_MODE;
                _statusFlags |= MOTOR_RUNNING;
                // TODO: TURN void turnRobot(float angle);
                break;

            case COMMAND_GET_DISTANCE:
                // TODO
                break;

            case COMMAND_GET_LINE_POSITION:
                // TODO
                break;

            case COMMAND_GET_STATUSFLAGS:
                response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                            message.senderTaskId,
                                            COMMAND_GET_STATUSFLAGS,
                                            (uint32_t)_statusFlags);
                xQueueSend(_dispatcherQueue, &response, 0);
                break;

            default:
                // error! command not found assert() ?
                break;
            }
        } // end of message handling

        /// ------- stm check safetybutton -------
        if (!_safetyButton.getValue())
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
        if ((_statusFlags & MOTOR_RUNNING) && (_statusFlags & LINE_FOLLOWER_MODE))
        {
            _followLine();

            drivenDistanceDriver0 = _driver0.getXActual();
            drivenDistanceDriver1 = _driver0.getXActual();

            // maxdistance reached - send Info to RaspberryHAT
            if (drivenDistanceDriver0 > maxDistance || drivenDistanceDriver1 > maxDistance)
            {
                _statusFlags &= ~MOTOR_RUNNING;
                _statusFlags | MAXDISTANCE_REACHED;
                dispatcherMessage_t response;
                response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                            TASKID_RASPBERRY_HAT_COM_TASK,
                                            COMMAND_SEND_WARNING, // TODO: change command to Info
                                            (uint32_t)_statusFlags);
                xQueueSend(_dispatcherQueue, &response, 0);
            }
        }

        /// ------- stm turn vehicle -------
        if ((_statusFlags & MOTOR_RUNNING) && (_statusFlags & TURN_MODE))
        {
            // TODO turnRobot(int16_t angle); -- angle * 10!
        }

        /// ------- stm check and send info flags -------
        if (_statusFlags & RUNMODEFLAG_T_EVENTFLAGS_BITMASK)
        {
            dispatcherMessage_t response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                                            TASKID_RASPBERRY_HAT_COM_TASK,
                                                            COMMAND_SEND_WARNING,
                                                            (uint32_t)_statusFlags);
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
    _driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
    _driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);

    // init adc device and line Sensor
    _adc = Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
    _lineSensor = LineSensor(&_adc, UV_LED_GPIO);

    // int safety button
    _safetyButton = DigitalInput(DIN_4);
}

void LineFollowerTask::_followLine()
{
    // init vars
    bool speedMode = (_statusFlags & RUNMODE_SLOW) ? true : false;

    int32_t v1 = 0;
    int32_t v2 = 0;

    // get Sensor values
    int8_t y = _lineSensor.getLinePosition();

#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
    int8_t y = lineSensor->getLinePosition();
#else
    uint16_t y = _lineSensor.getLinePositionAnalog();
#endif

    // check for LineSensor events
    if (_lineSensor.getStatus() & LINESENSOR_CROSS_DETECTED)
    {
        _statusFlags |= CROSSPOINT_DETECTED;
        _statusFlags & ~MOTOR_RUNNING;
        _statusFlags & ~LINE_FOLLOWER_MODE;

        printf("LINESENSOR detected Crosspoint \n");
        return;
    }
    if (_lineSensor.getStatus() & LINESENSOR_NO_LINE)
    {
        _statusFlags |= LOST_LINE;
        _statusFlags & ~MOTOR_RUNNING;
        _statusFlags & ~LINE_FOLLOWER_MODE;

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
    _driver0.moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    _driver1.moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
} // end followLine

int32_t LineFollowerTask::_controllerC(int8_t e)
{
#if LINEFOLLERCONFIG_USE_P_CONTROLLER == 1
    // P-Type Controller
    int32_t u = 0; // default 0
    u = e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP;
#endif
    return u * LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P;
} // end Control

void LineFollowerTask::_stopDrives()
{
    // check if stop command is already sent
    uint32_t vmax0 = _driver0.getVmax();
    uint32_t vmax1 = _driver1.getVmax();

    if (vmax0 != 0 || vmax1 != 0)
    {
        _driver0.moveVelocityMode(1, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        _driver1.moveVelocityMode(0, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    }

    return;
} // end stop Drives

/* ================================= */
/*              getters              */
/* ================================= */

/// @brief signleton pattern
/// @param dispatcherQueue Queue of dispatcher Task
/// @param lineFollowerQueue Queue of LineFollower Task
/// @return instance of LineFollowerTask
LineFollowerTask LineFollowerTask::getInstance(QueueHandle_t *messageDispatcherQueue)
{
    if (_instance == nullptr)
    {
        _instance = new LineFollowerTask(messageDispatcherQueue);
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
