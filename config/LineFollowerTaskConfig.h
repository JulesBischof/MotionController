#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (1)

/* ==============================================================

                   set Controller Parameter

============================================================== */

#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (5000)

/* ==============================================================

                         set VMAX / AMAX

============================================================== */

#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST (100000)
#define LINEFOLLERCONFIG_VMAX_STEPSPERSEC_SLOW (100000)

#define LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED (5000)

#endif // LINEFOLLOWERTASKCONFIG_H