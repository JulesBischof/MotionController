#include "vLineFollowerTask.hpp"

typedef enum RunModeFlag_t
{
    MOTOR_RUNNING = 1 << 0,
    RUNMODE_SLOW = 1 << 1,
    MAXDISTANCE_REACHED = 1 << 2,
    LINE_FOLLOWER_MODE = 1 << 3,
    TURN_MODE = 1 << 4,
    WAITING_FOR_INSTRUCTION = 1 << 5
} RunModeFlag_t;

dispatcherMessage_t generateResponse(dispatcherTaskId_t senderTaskId, dispatcherTaskId_t recieverTaskId, taskCommand_t command, uint32_t data)
{
    dispatcherMessage_t response;
    response.senderTaskId = senderTaskId;
    response.recieverTaskId = recieverTaskId;
    response.command = command;
    response.data = data;

    return response;
}

void followLine(Tmc5240 *Driver0, Tmc5240 *Driver1, LineSensor *lineSensor, int32_t *drivenDistanceDriver0, int32_t *drivenDistanceDriver1, uint32_t *flags)
{
    // init vars
    bool runSlow = (*flags & RUNMODE_SLOW) ? true : false;

    int32_t v1 = 0;
    int32_t v2 = 0;

    // get Sensor values
    int8_t linePosition = lineSensor->getLinePosition();
    // TODO: HCSR04 get distance

    // set Controller values
#if LINEFOLLERCONFIG_USE_P_CONTROLLER == 1
    // P-Type Controller
    int8_t e = 0;     // error - default 0
    e -= linePosition; // calc error value

    if (!runSlow)
    {
        v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST + (e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP);
        v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST - (e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP);
    }
    else
    {
        v1 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW + (e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP);
        v2 = LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW - (e * LINEFOLLERCONFIG_CONTROLLERVALUE_KP);
    }

#endif

    // set Motor values
    Driver0->moveVelocityMode(1, v1, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
    Driver1->moveVelocityMode(0, v2, LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED);
}

void stopDrives(Tmc5240 *Driver0, Tmc5240 *Driver1, uint32_t *flags)
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
}

void vLineFollowerTask(void *pvParameters)
{
    // initialize peripherals neccessary for line follower
    Tmc5240 driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0, 0);
    Tmc5240 driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1, 0);

    // init adc device and line Sensor
    ArduinoAdcSlave adc = ArduinoAdcSlave(UART_INSTANCE_GRIPCONTROLLER);
    LineSensor lineSensor = LineSensor(&adc, UV_LED_GPIO);

    // get queues
    QueueHandle_t xLineFollwerQueue = getLineFollowerTaskQueue();
    QueueHandle_t xDispatcherQueue = getDispatcherTaskQueue();

    // Flags
    uint32_t flags = 0;

    // Variables
    int32_t drivenDistanceDriver0 = 0;
    int32_t drivenDistanceDriver1 = 0;

    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(xLineFollwerQueue) > 0)
        {
            dispatcherMessage_t message;
            xQueueReceive(xLineFollwerQueue, &message, pdMS_TO_TICKS(10));

            // TODO: check if message is meant for this task
            dispatcherMessage_t response;

            switch (message.command)
            {
            case COMMAND_FOLLOW_LINE:
                flags &= ~TURN_MODE;
                flags &= ~MAXDISTANCE_REACHED;
                flags &= ~RUNMODE_SLOW;
                flags |= LINE_FOLLOWER_MODE;
                flags |= MOTOR_RUNNING;
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
                response = generateResponse(LINE_FOLLOWER_TASK,
                                            message.senderTaskId, COMMAND_GET_STATUSFLAGS,
                                            (uint32_t)flags);
                xQueueSend(xDispatcherQueue, &response, 0);
                break;

            default:
                // error! command not found assert() ?
                break;
            }
        } // end of message handling

        // controller
        if (flags & !MOTOR_RUNNING)
        {
            stopDrives(&driver0, &driver1, &flags);
        }

        if ( (flags & MOTOR_RUNNING) && (flags & LINE_FOLLOWER_MODE ))
        {
            followLine(&driver0,
                       &driver1,
                       &lineSensor,
                       &drivenDistanceDriver0,
                       &drivenDistanceDriver1,
                       &flags);
        }

        if (flags & MOTOR_RUNNING & TURN_MODE)
        {
            // TODO: GET ANGLE AND TURN
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}