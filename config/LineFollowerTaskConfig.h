#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERCONFIG_STACKSIZE (1024)
#define LINEFOLLOWERCONFIG_PRIORITY (1)


/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (1)
#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0)

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1000)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (4) 

/* ==============================================================

                set VMAX / AMAX / MaxDistance

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (50000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (20000)

#define LINEFOLLOWERCONFIG_MAXDISTANCE (5)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (5000)

#endif // LINEFOLLOWERTASKCONFIG_H