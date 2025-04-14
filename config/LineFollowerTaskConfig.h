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

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0) // 0 = analog, 1 = digital

#define ENABLE_DATA_OUTPUT_LINEPOS (1) 

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1) // ~1 analog / ~1000 digital
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

// P
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (35) // ~30 - 35 analog //~ 13 digital

// D
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KD (0.7.f) // ~0.7 analog //~ 0.7 digital

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

#define LINEFOLLOWERCONFIG_LINESENSOR_CELLDISTANCE_mm (9)

// Barrier Handling
#define BRAKEDISTANCE_BARRIER_IN_MM (103.f)
#define LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm (15)
#define LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm (5)
#define LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_cm (25.f)


#endif // LINEFOLLOWERTASKCONFIG_H