#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (8192) // 8kb
#define LINEFOLLOWERTASK_PRIORITY (2)

#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (10)

#define LINEFOLLOWERCONFIG_POLLING_RATE_MS (50)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (1)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (1)

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1000) // ~1000 digital
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (10) // ~30 analog

/* ==============================================================

                set VMAX / AMAX / MaxDistance

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (2 * 50000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (50000)

#define LINEFOLLOWERCONFIG_MAXDISTANCE (200)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (5000)


/* ==============================================================

                    Physical dimensions

============================================================== */
#define LINEFOLLOWERCONFIG_AXIS_WIDTH_m (272 / 1e3) // TODO: CHECK
#define LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_cm (11) // TODO: CHECK

#endif // LINEFOLLOWERTASKCONFIG_H