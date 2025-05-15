#pragma once

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (20 * 1024) // 20kB
#define LINEFOLLOWERTASK_PRIORITY (1)
#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (100)
#define LINEFOLLOWERCONFIG_POLLING_RATE_MS (10)

#define LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS (30) // n of linepolls to get a median value

// DEBUG

#define ENABLE_DATA_OUTPUT_LINEPOS (0)
#define TRIGGER_LINESENSOR_SAMPLES (0)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (0)
#define LINEFOLLERCONFIG_USE_PD_CONTROLLER (0)

#define LINEFOLLERCONFIG_USE_PD_CONTROLLER_MATLAB (1)

#define LINEFOLLOWERCONFIG_USE_DIGITAL_LINESENSOR (0) // 0 = analog, 1 = digital

/* ==============================================================

                   set Controller Parameter

============================================================== */
constexpr int32_t LINEFOLLERCONFIG_CONVERSION_CONSTANT_C_TO_P = (1); // ~1 analog / ~1000 digital
constexpr int32_t LINEFOLLOWERCONFIG_CONTROLVALUE_DIGITAL = (0);
constexpr int32_t LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG = (3500);

// P
constexpr float LINEFOLLERCONFIG_CONTROLLERVALUE_KP(32); // ~32 analog //~ 13 digital

// D
constexpr float LINEFOLLERCONFIG_CONTROLLERVALUE_KD = (3.2f); //~ 0.7 digital ~1.5f Analog

// PD - Matlab Approximation
constexpr double cz_matlab_num[2] = {59.85, -58.67};
constexpr double cz_matlab_dnum[2] = {1, -0.8519};

/* ==============================================================

                set VMAX / AMAX / MaxDistance / PowerManagement

============================================================== */

/* ---  AMAX and VMAX Management   ---*/
constexpr int32_t LINEFOLLERCONFIG_VMAX_REGISTER_VALUE = (150000);
constexpr int32_t LINEFOLLERCONFIG_VMAX_REGISTER_VALUE_SLOW = (50000);

constexpr int32_t LINEFOLLERCONFIG_AMAX_REGISTER_VALUE = (4500); // 6500

/* --- straight velocity params ------ */
constexpr float LINEFOLLERCONFIG_VMAX_STRAIGHT_BACKWARDS_PERCANTAGE = (0.5);
constexpr float LINEFOLLERCONFIG_AMAX_STRAIGHT_BACKWARDS_PERCANTAGE = (0.2);

constexpr int32_t LINEFOLLOWERCONFIG_BACKWARDS_VMAX = LINEFOLLERCONFIG_VMAX_STRAIGHT_BACKWARDS_PERCANTAGE * LINEFOLLERCONFIG_VMAX_REGISTER_VALUE;
constexpr int32_t LINEFOLLOWERCONFIG_BACKWARDS_AMAX = LINEFOLLERCONFIG_AMAX_STRAIGHT_BACKWARDS_PERCANTAGE * LINEFOLLERCONFIG_AMAX_REGISTER_VALUE;

/* ------ turn velocity params ------ */
constexpr float LINEFOLLERCONFIG_VMAX_TURN_PERCANTAGE = (1);
constexpr float LINEFOLLERCONFIG_AMAX_TURN_PERCANTAGE = (0.3);

constexpr int32_t LINEFOLLOWERCONFIG_TURN_VMAX = LINEFOLLERCONFIG_VMAX_TURN_PERCANTAGE * LINEFOLLERCONFIG_VMAX_REGISTER_VALUE;
constexpr int32_t LINEFOLLOWERCONFIG_TURN_AMAX = LINEFOLLERCONFIG_AMAX_TURN_PERCANTAGE * LINEFOLLERCONFIG_AMAX_REGISTER_VALUE;

/* ------   power Management    ------*/
constexpr int32_t LINEFOLLOWERCONFIG_MOTORCURRENT_LINEFOLLOWER_PERCENTAGE = (30);
constexpr int32_t LINEFOLLOWERCONFIG_MOTORCURRENT_POSITIONMODE_PERCENTAGE = (50);
constexpr int32_t LINEFOLLOWERCONFIG_MOTORCURRENT_TURN_PERCENTAGE = (100);
constexpr int32_t LINEFOLLOWERCONFIG_MOTORCURRENT_STOP_PERCENTAGE = (100);

/* ==============================================================

                    Physical dimensions

============================================================== */
constexpr float LINEFOLLOWERCONFIG_AXIS_WIDTH_mm(240); // 272 - using CAD
constexpr float LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm(111.f);
constexpr float LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM(80);
constexpr float LINEFOLLOWERCONFIG_LINESENSOR_CELLDISTANCE_mm(9);

// Barrier Handling
constexpr float SLOWDOWNDISTANCE_BARRIER_IN_MM = (270.f); // 110
constexpr float BRAKEDISTANCE_BARRIER_IN_MM = (100.f);
constexpr uint8_t LINEFOLLOWERCONFIG_MEDIANSTACK_SIZE = 11;

constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm = (28);
constexpr float LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm = (2);
constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm = (350);
constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm = (50);
