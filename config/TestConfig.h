#ifndef TESTCONFIG_H
#define TESTCONFIG_H

/* ==============================================================

                        select Tests to run

        make sure to toggle "Device Tests" in AppConfig.h
        
============================================================== */




// ------------- stepper devices ------------------
#define TEST_TMC5240 (0)
#define TEST_TMC5260_VELOCITY_MODE (0)
#define TEST_TMC5260_POSITION_MODE (0)

// --------------- i2c devices ------------------

#define BROADCAST_I2C_DEVICES (0)

#define TEST_ICM42670 (0)

#define TEST_TLA2528 (0)

// --------------- misc devices ------------------
#define TEST_LINESENSOR (0)
#define TEST_LINISENSOR_USING_TLA2528 (0)
#define TEST_LINESENSOR_USING_ARDUINO (0)
#define TEST_LINESENSOR_ANALOGMODE (0)

#define TEST_HCSR04 (1)

// ------------------ Tasks ---------------------

// ## LineFollowerTask ##
#define TEST_VLINEFOLLOWERTASK (0)
#define TEST_SENDMESSAGE_VIA_DISPATCHER (0)
#define TEST_FOLLOW_LINE (0)
#define TEST_TURN (0)
#define TEST_POSITIONMODE (0)

// ------------------ Misc ---------------------
#define TEST_UART_IRQ_CONFIGURATION (0)

#endif // TESTCONFIG_H