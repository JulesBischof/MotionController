#ifndef ICM42670PCONFIG_H
#define ICM42670PCONFIG_H

#include "Icm42670p_HW_Abstraction.h"

/* ==============================================================

        Basic Initialization Gyroscope - ref Datasheet

============================================================== */

/* PWR_MGMT0
    - Accelerometer LP mode uses Wake Up oscillator clock
    - the RC oscillator is powered on even if Accel and Gyro are powered off
    - Places gyroscope in Low Noise (LN) Mode
    - Turns accelerometer off
*/
#define PWR_MGMT0_INITVALUE ( (0b0 << PWR_MGMT0_ACCEL_LP_CLK_SEL_POS) | (0b1 << PWR_MGMT0_IDLE_POS) | (0b11 << PWR_MGMT0_GYRO_MODE_POS) | (0b00 << PWR_MGMT0_ACCEL_MODE_POS) )

/* GYRO_CONFIG0
    - +-2000 °/s
    - ODR 200Hz
*/
#define CONFIG0_INITVALUE ( (0b00 << GYRO_CONFIG0_GYRO_UI_FS_SEL_POS) | (0b1000 << GYRO_CONFIG0_GYRO_ODR_POS) )

/* GYRO_CONFIG1
    - Gyro low pass Filter bandwith 16Hz
    - disable average filter accelerometer
    - disable accel low pass filter
*/
#define CONFIG1_INITVALUE ((0b111 << GYRO_CONFIG1_GYRO_UI_FILT_BW_POS) )

/* ==============================================================

        Unit Conversion - Scale Factor - ref Datasheet

============================================================== */

/*
depends on GYRO_UI_FS_SEL - Bits 6:5 - GYRO_CONFIG_0 ### INITIALIZATION ###

GYRO_UI_FS_SEL = 0 -> 16.4 LSB/(º/s)    (FullScale Select +-2000dps)
GYRO_UI_FS_SEL = 1 -> 32.8 LSB/(º/s)    (FullScale Select +-1000dps)
GYRO_UI_FS_SEL = 2 -> 65.5 LSB/(º/s)    (FullScale Select +-500 dps)
GYRO_UI_FS_SEL = 3 -> 131 LSB/(º/s)     (FullScale Select +-250 dps)
*/

#define LSB_TO_DPS_SCALE_FACTOR (16.4f)

/* ==============================================================

                         misc settings

============================================================== */

// figured out by tests
#define STATIC_OFFSET (6)
#define SENSITIVITY_TRESHOLD (20)
#define POLLING_RATE_MS (50)

#endif // ICM42670PCONFIG_H