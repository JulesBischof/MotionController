#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"
#include "AppConfig.h"
#include "tests.h"

#include "MotionController.hpp"

int main()
{
    stdio_init_all();

#if APP_MODE == 0
    testApp();
#endif

#if APP_MODE == 1

    nMotionController::MotionController motionController = nMotionController::MotionController();
    motionController.startScheduler();

    // never reached
    while (1)
    {
    };
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
