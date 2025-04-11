#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (10 * 1024) // 10kB
#define LINEFOLLOWERTASK_PRIORITY (4)

#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (10)

#define LINEFOLLOWERCONFIG_POLLING_RATE_MS (30)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (0)
#define LINEFOLLERCONFIG_USE_PD_CONTROLLER (1)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (1)

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1000) // ~1000 digital
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

// P
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (13) // ~30 analog

// D
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KD (0.7f)
// #define LINEFOLLERCONFIG_CONTROLLERVALUE_LP_TAU
// #define LINEFOLLERCONFIG_CONTROLLERVALUE_LP_ALPHA (LINEFOLLERCONFIG_CONTROLLERVALUE_LP_TAU / (LINEFOLLERCONFIG_CONTROLLERVALUE_LP_TAU + LINEFOLLOWERCONFIG_POLLING_RATE_MS))
// #define LINEFOLLERCONFIG_CONTROLLERVALUE_LP_BETA (LINEFOLLERCONFIG_CONTROLLERVALUE_KD / (LINEFOLLERCONFIG_CONTROLLERVALUE_LP_TAU + LINEFOLLOWERCONFIG_POLLING_RATE_MS))

/* ==============================================================

                set VMAX / AMAX / MaxDistance / PowerManagement

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (2 * 75000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (50000)

#define LINEFOLLOWERCONFIG_MAXDISTANCE (200)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (4000)

#define LINEFOLLOWERCONFIG_MOTORCURRENT_LINEFOLLOWER_PERCENTAGE (30)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE (50)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE (100)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE (100)

/* ==============================================================

                    Physical dimensions

============================================================== */
#define LINEFOLLOWERCONFIG_AXIS_WIDTH_mm (283)                 // 272 - using CAD
#define LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm (111)

#define BRAKEDISTANCE_BARRIER_IN_MM (118.f)

#endif // LINEFOLLOWERTASKCONFIG_H