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

#define TEST_HCSR04 (0)

#define TEST_ARDUINO_ADC (0)

// ------------------ Tasks ---------------------

// ## LineFollowerTask ##
#define TEST_VLINEFOLLOWERTASK (1)


#endif // TESTCONFIG_H