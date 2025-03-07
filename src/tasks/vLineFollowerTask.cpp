#include "vLineFollowerTask.hpp"

typedef enum RunModeFlag_t
{
    MOTOR_RUNNING = 1 << 0, // 0001
    LINE_FOLLOWER = 1 << 1, // 0010
    TURN_COMMAND = 1 << 2   // 0100
} RunModeFlag_t;

void vLineFollowerTask(void *pvParameters)
{
    // initialize peripherals neccessary for line follower
    Tmc5240 driver0 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_0);
    Tmc5240 driver1 = Tmc5240(TMC5240_SPI_INSTANCE, SPI_CS_DRIVER_1);

    LineSensor lineSensor = LineSensor(*I2C_INSTANCE_DEVICES, I2C_DEVICE_TLA2528_ADDRESS, UV_LED_GPIO);

    // get queues
    QueueHandle_t xLineFollwerQueue = getLineFollowerTaskQueue();
    QueueHandle_t xDispatcherQueue = getDispatcherTaskQueue();

    // set variables
    int8_t linePosition = 0;
    uint32_t distance = 0;

    // Flags
    uint8_t flags = 0;

    // loop forever
    for (;;)
    {
        if (uxQueueMessagesWaiting(xLineFollwerQueue) > 0)
        {
            dispatcherMessage_t message;
            xQueueReceive(xDispatcherQueue, &message, 0);

            // TODO: check if message is meant for this task

            switch (message.command)
            {
            case DRIVE:
                flags &= ~TURN_COMMAND;
                flags |= LINE_FOLLOWER;
                flags |= MOTOR_RUNNING;
                break;

            case STOP:
                flags &= ~MOTOR_RUNNING;
                break;

            case TURN:
                flags &= ~LINE_FOLLOWER;
                flags |= TURN_COMMAND;
                flags |= MOTOR_RUNNING;
                break;

            case GET_DISTANCE:
                // TODO
                break;

            case GET_LINE_POSITION:
                dispatcherMessage_t response;
                    response.taskId = LINE_FOLLOWER_TASK;
                    response.command = GET_LINE_POSITION;
                    response.data = (uint32_t)linePosition;
                
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
            // TODO: STOP MOTORS set back variables, send confirmation
        }

        if (flags & MOTOR_RUNNING & LINE_FOLLOWER)
        {
            // TODO: LINE FOLLOWER
        }

        if (flags & MOTOR_RUNNING & TURN_COMMAND)
        {
            // TODO: GET ANGLE AND TURN
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}