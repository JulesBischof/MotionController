#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (4096)
#define LINEFOLLOWERTASK_PRIORITY (1)

#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (100)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (1)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0)

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (30)

/* ==============================================================

                set VMAX / AMAX / MaxDistance

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (50000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (20000)

#define LINEFOLLOWERCONFIG_MAXDISTANCE (200)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (5000)

/* ==============================================================

                    Physical dimensions

============================================================== */
#define LINEFOLLOWERCONFIG_AXIS_WIDTH_m (30 / 1e2) // TODO: CHECK
#define LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_m (20 / 1e2) // TODO: CHECK

#endif // LINEFOLLOWERTASKCONFIG_H