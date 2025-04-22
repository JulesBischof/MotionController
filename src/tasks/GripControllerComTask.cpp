#include "MotionController.hpp"

#include <stdio.h>
#include <cstdint>
#include <cstring>

#include "pico/stdlib.h"

#include "prain_uart/decoder.hpp"
#include "prain_uart/encoder.hpp"

#include "MotionControllerConfig.hpp"
#include "MotionControllerPinning.hpp"

#include "GripControllerComTaskConfig.hpp"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"


using namespace prain_uart;
using namespace encoder;

namespace MtnCtrl
{
    void MotionController::_gripControllerComTask()
    {
        // init Rx interrupts
        _initUartRxIsr(UART_INSTANCE_GRIPCONTROLLER, _uart1RxIrqHandler);

        // get QueueHandles
        QueueHandle_t gripControllerComQueue = getGripControllerComQueue();
        if (gripControllerComQueue == nullptr)
        {
            printf("ERROR #gripControllerComTask# NULLREFERENCE gripControllerComQueue\n");
            while (1)
            { /*  ERROR  */
            }
        }
        QueueHandle_t messageDispatcherQueue = getMessageDispatcherQueue();
        if (messageDispatcherQueue == nullptr)
        {
            printf("ERROR #gripControllerComTask# NULLREFERENCE messageDispatcherQueue\n");
            while (1)
            { /*  ERROR  */
            }
        }

        // loop forever
        for (;;)
        {

            DispatcherMessage message;
            DispatcherMessage uartMsg;

            // suspend Task until Message is recieved
            if (xQueueReceive(gripControllerComQueue, &message, portMAX_DELAY) == pdTRUE)
            {
                if (message.receiverTaskId != DispatcherTaskId::GripControllerComTask)
                {
                    printf("ERROR #_gripControllerComTask# - Message contains wrong Task ID \n");
                    continue;
                }

                frame txMsg;

                // check incoming commands - messages supposed to go out
                switch (message.command)
                {
                case (TaskCommand::CraneGrip):
                    txMsg = encode_grip(address::GRIP_CTRL);
                    break;
                case (TaskCommand::CraneRelease):
                    txMsg = encode_release(address::GRIP_CTRL);
                    break;
                case (TaskCommand::DecodeMessage):
                    uartMsg = _getCommand(UART_INSTANCE_GRIPCONTROLLER);
                    
                    if (xQueueSend(messageDispatcherQueue, &uartMsg, pdMS_TO_TICKS(100)) != pdTRUE)
                    {
                        printf("ERROR #_gripControllerComTask# WRITE TO QUEUE cmd decode msg FAILED\n");
                    }
                    break;
                default:
                    break;
                }
                // send msg to Gc
                if (txMsg.raw() != 0)
                {
                    sendUartMsg(&txMsg, UART_INSTANCE_GRIPCONTROLLER);
                }
            }
        }
    }

    void MotionController::_uart1RxIrqHandler()
    {
        /* some message arrived!! send read command to gripControllerComQueue
        buffer-read and decoding shouldn't be handled inside an isr */
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        QueueHandle_t queueHandle = getGripControllerComQueue();

        if (queueHandle != nullptr)
        {
            DispatcherMessage msg(
                DispatcherTaskId::GripControllerComTask,
                DispatcherTaskId::GripControllerComTask,
                TaskCommand::DecodeMessage,
                0);

            xQueueSendFromISR(queueHandle, &msg, &xHigherPriorityTaskWoken);
        }
        // disable uart interrupts
        uart_set_irq_enables(UART_INSTANCE_GRIPCONTROLLER, false, false);

        if (xHigherPriorityTaskWoken)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}