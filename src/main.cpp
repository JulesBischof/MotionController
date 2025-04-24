#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "AppConfig.hpp"
#include "tests.hpp"

#include "MotionController.hpp"

int main()
{
    stdio_init_all();

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 1

    MtnCtrl::MotionController motionController = MtnCtrl::MotionController();
    motionController.startTasks();

    vTaskStartScheduler();
    for (;;)
    {
        /* never reached */
    }

#endif
}

// -------- Hooks
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // Handle Stack Overflow hier
    printf("Stack Overflow in Task: %s\n", pcTaskName);
    // Optional: Hier kannst du das System stoppen, neu starten oder debuggen.
    for (;;)
        ; // Endlosschleife, falls erforderlich
}
