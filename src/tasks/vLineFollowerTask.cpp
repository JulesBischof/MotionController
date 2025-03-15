#include "vLineFollowerTask.hpp"

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

#define RUNMODEFLAG_T_EVENTFLAGS_BITMASK (0xFF00)
typedef enum RunModeFlag_t
{
    // lower 8 bits statemaschine relevant flags
    MOTOR_RUNNING = 1 << 0,
    RUNMODE_SLOW = 1 << 1,
    LINE_FOLLOWER_MODE = 1 << 2,
    TURN_MODE = 1 << 3,

    // upper 8 bits events and infos
    CROSSPOINT_DETECTED = 1 << 8,
    LOST_LINE = 1 << 9,
    MAXDISTANCE_REACHED = 1 << 10,
    SAFETY_BUTTON_PRESSED = 1 << 11
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
    xTaskCreate(_run, LINEFOLLOWERTASK_NAME, LINEFOLLOWERCONFIG_STACKSIZE, this, LINEFOLLOWERCONFIG_PRIORITY, &_taskHandle);
} // end ctor

/// @brief signleton pattern
/// @param dispatcherQueue Queue of dispatcher Task
/// @param lineFollowerQueue Queue of LineFollower Task
/// @return instance of LineFollowerTask
LineFollowerTask LineFollowerTask::getInstance(QueueHandle_t *dispatcherQueue)
{
    if (_instance == nullptr)
    {
        _instance = new LineFollowerTask(dispatcherQueue);
    }
    return *_instance;
}

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
    // initialize peripherals neccessary for line follower
    Tmc5240 driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 1);
    Tmc5240 driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 1);

    // init adc device and line Sensor
    Tla2528 adc = Tla2528(I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

    // int safety button
    DigitalInput safetyButton = DigitalInput(DIN_4);

    // Flags
    uint32_t flags = 0;

    // Variables
    int32_t X_ACTUAL_startValueDriver0 = driver0.getXActual();
    int32_t drivenDistanceDriver0 = 0;

    int32_t X_ACTUAL_startValueDriver1 = driver1.getXActual();
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
                flags &= ~TURN_MODE;
                flags &= ~MAXDISTANCE_REACHED;
                flags &= ~RUNMODE_SLOW;
                flags &= ~LOST_LINE;
                flags &= ~CROSSPOINT_DETECTED;
                flags |= LINE_FOLLOWER_MODE;
                flags |= MOTOR_RUNNING;

                X_ACTUAL_startValueDriver0 = driver0.getXActual();
                X_ACTUAL_startValueDriver1 = driver1.getXActual();
                break;

            case COMMAND_STOP:
                flags &= ~MOTOR_RUNNING;
                break;

            case COMMAND_TURN:
                flags &= ~LINE_FOLLOWER_MODE;
                flags |= TURN_MODE;
                flags |= MOTOR_RUNNING;
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
                                            (uint32_t)flags);
                xQueueSend(_dispatcherQueue, &response, 0);
                break;

            default:
                // error! command not found assert() ?
                break;
            }
        } // end of message handling

        /// ------- stm check safetybutton -------
        if (!safetyButton.getValue())
        {
            flags &= ~MOTOR_RUNNING;
            flags | SAFETY_BUTTON_PRESSED;
        }
        else
        {
            flags &= ~SAFETY_BUTTON_PRESSED;
        }

        // ------- stm check hcsr04 distance -------

        // TODO: check distance

        // ------- stm line follower -------
        if ((flags & MOTOR_RUNNING) && (flags & LINE_FOLLOWER_MODE))
        {
            _followLine(&driver0,
                       &driver1,
                       &lineSensor,
                       &flags);

            drivenDistanceDriver0 = driver0.getXActual();
            drivenDistanceDriver1 = driver0.getXActual();

            // maxdistance reached - send Info to RaspberryHAT
            if (drivenDistanceDriver0 > maxDistance || drivenDistanceDriver1 > maxDistance)
            {
                flags &= ~MOTOR_RUNNING;
                flags | MAXDISTANCE_REACHED;
                dispatcherMessage_t response;
                response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                            TASKID_RASPBERRY_HAT_COM_TASK,
                                            COMMAND_SEND_WARNING, // TODO: change command to Info
                                            (uint32_t)flags);
                xQueueSend(_dispatcherQueue, &response, 0);
            }
        }

        /// ------- stm turn vehicle -------
        if ((flags & MOTOR_RUNNING) && (flags & TURN_MODE))
        {
            // TODO turnRobot(int16_t angle); -- angle * 10!
        }

        /// ------- stm check and send info flags -------
        if (flags & RUNMODEFLAG_T_EVENTFLAGS_BITMASK)
        {
            dispatcherMessage_t response = generateResponse(TASKID_LINE_FOLLOWER_TASK,
                                                            TASKID_RASPBERRY_HAT_COM_TASK,
                                                            COMMAND_SEND_WARNING,
                                                            (uint32_t)flags);
            xQueueSend(_dispatcherQueue, &response, 0);
        }

        /// ------- stm stop drives -------
        if (!(flags & MOTOR_RUNNING))
        {
            _stopDrives(&driver0, &driver1, &flags);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
} // end vLineFollowerTask

void LineFollowerTask::_followLine(Tmc5240 *Driver0, Tmc5240 *Driver1, LineSensor *lineSensor, uint32_t *flags)
{
    // init vars
    bool speedMode = (*flags & RUNMODE_SLOW) ? true : false;

    int32_t v1 = 0;
    int32_t v2 = 0;

    // get Sensor values
    int8_t y = lineSensor->getLinePosition();

#if LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR == 1
    int8_t y = lineSensor->getLinePosition();
#else
    uint16_t y = lineSensor->getLinePositionAnalog();
#endif

    // check for LineSensor events
    if (lineSensor->getStatus() & LINESENSOR_CROSS_DETECTED)
    {
        *flags |= CROSSPOINT_DETECTED;
        *flags & ~MOTOR_RUNNING;
        *flags & ~LINE_FOLLOWER_MODE;

        printf("LINESENSOR detected Crosspoint \n");
        return;
    }
    if (lineSensor->getStatus() & LINESENSOR_NO_LINE)
    {
        *flags |= LOST_LINE;
        *flags & ~MOTOR_RUNNING;
        *flags & ~LINE_FOLLOWER_MODE;

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
    Driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    Driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
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

void LineFollowerTask::_stopDrives(Tmc5240 *Driver0, Tmc5240 *Driver1, uint32_t *flags)
{
    // check if stop command is already sent
    uint32_t vmax0 = Driver0->getVmax();
    uint32_t vmax1 = Driver1->getVmax();

    if (vmax0 != 0 || vmax1 != 0)
    {
        Driver0->moveVelocityMode(1, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
        Driver1->moveVelocityMode(0, 0, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    }

    return;
} // end stop Drives

QueueHandle_t LineFollowerTask::getQueue()
{
    return _lineFollowerQueue;
}

TaskHandle_t LineFollowerTask::getTaskHandle()
{
    return _taskHandle;
}
