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

#define LINEFOLLERCONFIG_USE_PD_CONTROLLER_MATLAB (0)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0) // 0 = analog, 1 = digital

#define ENABLE_DATA_OUTPUT_LINEPOS (1) 

/* ==============================================================

                   set Controller Parameter

============================================================== */
#define LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P (1) // ~1 analog / ~1000 digital
#define LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL (0)
#define LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG (3500)

// P
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KP (32) // ~32 analog //~ 13 digital

// D
#define LINEFOLLERCONFIG_CONTROLLERVALUE_KD (3.2f) //~ 0.7 digital ~1.5f Analog

// PD - Matlab Approximation
constexpr double pd_alpha = 0.2308;
constexpr double pd_beta = 196.9;

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
#define LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm (360)
#define LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm (50)

#define LINEFOLLOWERCONFIG_WAIT_MS_UNTIL_POSITION_CORRECTION (500)
#define LINEFOLLOWERCONFIG_WAIT_MS_180_TURN (1000)

constexpr uint8_t LINEFOLLOWERCONFIG_MEDIANSTACK_SIZE = 11;

#endif // LINEFOLLOWERTASKCONFIG_H