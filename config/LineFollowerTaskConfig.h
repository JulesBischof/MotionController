#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (1)

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1000)
#define LINEFOLLERCONFIG_VALUE_X (0)

#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (2) 

/* ==============================================================

                set VMAX / AMAX / MaxDistance

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (50000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (25000)

#define LINEFOLLOWERCONFIG_MAXDISTANCE (5)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (5000)

#endif // LINEFOLLOWERTASKCONFIG_H