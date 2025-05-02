#ifndef LINEFOLLOWERTASKCONFIG_H
#define LINEFOLLOWERTASKCONFIG_H

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (10 * 1024) // 10kB
#define LINEFOLLOWERTASK_PRIORITY (1)

#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (100)

#define LINEFOLLOWERCONFIG_POLLING_RATE_MS (10)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (0)
#define LINEFOLLERCONFIG_USE_PD_CONTROLLER (1)

#define LINEFOLLERCONFIG_USE_PD_CONTROLLER_Z_TRANSFORM (0)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0) // 0 = analog, 1 = digital

#define ENABLE_DATA_OUTPUT_LINEPOS (0) 

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1) // ~1 analog / ~1000 digital
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

// P
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (32) // ~30 - 35 analog //~ 13 digital

// D
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KD (1.5f) //~ 0.7 digital

// PD - Matlab Approximation 
// G(z) = (A1 * z + A2) / (1 * z + B2)
#define LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_A1 (321.2f) // numerator
#define LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_A2 (-274.2f) // numerator
#define LINEFOLLOWERCONFIG_CONROLLERVALUE_MATLAB_Z_TRANSFORM_B2 (0.9157f) // denominator

/* ==============================================================

                set VMAX / AMAX / MaxDistance / PowerManagement

============================================================== */

#define LINEFOLLERCONFIG_VMAX_REGISTER_VALUE (150000)
#define LINEFOLLERCONFIG_VMAX_REGISTER_VALUE_SLOW (50000)

#define LINEFOLLERCONFIG_AMAX_REGISTER_VALUE (0.7 * 6500) // 6500

#define LINEFOLLOWERCONFIG_MOTORCURRENT_LINEFOLLOWER_PERCENTAGE (30)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE (50)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE (100)
#define LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE (100)

/* ==============================================================

                    Physical dimensions

============================================================== */
#define LINEFOLLOWERCONFIG_AXIS_WIDTH_mm (253)                 // 272 - using CAD
#define LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm (111.f)
#define LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM (80)
#define LINEFOLLOWERCONFIG_LINESENSOR_CELLDISTANCE_mm (9)

// Barrier Handling
#define SLOWDOWNDISTANCE_BARRIER_IN_MM (260.f) // 110
#define BRAKEDISTANCE_BARRIER_IN_MM (100.f)
#define LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm (28)
#define LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm (2)
#define LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm (365)
#define LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm (50)

#define LINEFOLLOWERCONFIG_WAIT_MS_UNTIL_POSITION_CORRECTION (500)
#define LINEFOLLOWERCONFIG_WAIT_MS_180_TURN (1000)

#endif // LINEFOLLOWERTASKCONFIG_H