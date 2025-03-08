#ifndef TESTCONFIG_H
#define TESTCONFIG_H

/* ==============================================================

                        select Tests to run

        make sure to toggle "Device Tests" in AppConfig.h
        
============================================================== */


#define BROADCAST_I2C_DEVICES (1)

#define TEST_TMC5240 (0)

#define TEST_ICM42670 (0)

#define TEST_TLA2528 (0)

#define TEST_LINESENSOR (0)

#define TEST_HCSR04 (0)

#endif // TESTCONFIG_H