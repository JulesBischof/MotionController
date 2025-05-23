#pragma once

#include <cmath>
#include "Tmc5240Config.hpp"
#include "FreeRTOS.h"

/* ==============================================================

                    general Settings

============================================================== */
#define LINEFOLLOWERTASK_NAME ("vLineFollowerTask")
#define LINEFOLLOWERTASK_STACKSIZE (20 * 1024) // 20kB
#define LINEFOLLOWERTASK_PRIORITY (1)
#define LINEFOLLOWERCONFIG_QUEUESIZE_N_ELEMENTS (100)
#define LINEFOLLOWERCONFIG_POLLING_RATE_MS (10)


#define USE_CHECKFORLINE_STM (1)

#define POLL_LINE_N_CHECK_CYCLES (4)
#define POLL_LINE_MOVE_INCREMENT_MM (10)
#define LINEFOLLOWERCONFIG_NUMBER_OF_LINEPOLLS (30) // n of linepolls to get a median value


// DEBUG

#define ENABLE_DATA_OUTPUT_LINEPOS (0)

#define TRIGGER_LINEPOS_SAMPLES (0)

#define TRIGGER_LINESENSOR_SAMPLES (0)

/* ==============================================================

                    select controllertype

============================================================== */

#define LINEFOLLERCONFIG_USE_P_CONTROLLER (0)
#define LINEFOLLERCONFIG_USE_PD_CONTROLLER (0)

#define LINEFOLLERCONFIG_USE_PD_CONTROLLER_MATLAB (1)

/* ==============================================================

                   set Controller Parameter

============================================================== */
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

constexpr float LINEFOLLERCONFIG_VMAX_STRAIGHT_FORWARD_PERCANTAGE = (1.0);
constexpr float LINEFOLLERCONFIG_AMAX_STRAIGHT_FORWARD_PERCANTAGE = (0.7);

constexpr int32_t LINEFOLLOWERCONFIG_BACKWARDS_VMAX = LINEFOLLERCONFIG_VMAX_STRAIGHT_BACKWARDS_PERCANTAGE * LINEFOLLERCONFIG_VMAX_REGISTER_VALUE;
constexpr int32_t LINEFOLLOWERCONFIG_BACKWARDS_AMAX = LINEFOLLERCONFIG_AMAX_STRAIGHT_BACKWARDS_PERCANTAGE * LINEFOLLERCONFIG_AMAX_REGISTER_VALUE;

constexpr int32_t LINEFOLLOWERCONFIG_FORWARDS_VMAX = LINEFOLLERCONFIG_VMAX_STRAIGHT_FORWARD_PERCANTAGE * LINEFOLLERCONFIG_VMAX_REGISTER_VALUE;
constexpr int32_t LINEFOLLOWERCONFIG_FORWARDS_AMAX = LINEFOLLERCONFIG_AMAX_STRAIGHT_FORWARD_PERCANTAGE * LINEFOLLERCONFIG_AMAX_REGISTER_VALUE;

/* ------ turn velocity params ------ */
constexpr float LINEFOLLERCONFIG_VMAX_TURN_PERCANTAGE = (1);
constexpr float LINEFOLLERCONFIG_AMAX_TURN_PERCANTAGE = (0.25);

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
constexpr float LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM(80); // 80
constexpr float LINEFOLLOWERCONFIG_LINESENSOR_CELLDISTANCE_mm(9);

/* ================================= */
/*            consts                 */
/* ================================= */

constexpr float V_MAX_ROTATIONS_PER_SEC = (LINEFOLLOWERCONFIG_FORWARDS_VMAX / (16777216.f / TMC5240CLOCKFREQUENCY) / MICROSTEPS_PER_REVOLUTION);
constexpr float V_MAX_IN_MMPS = ((LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM)*M_PI * V_MAX_ROTATIONS_PER_SEC); // mm per second

constexpr float V_SLOW_ROTATIONS_PER_SEC = (LINEFOLLERCONFIG_VMAX_REGISTER_VALUE_SLOW / (16777216.f / TMC5240CLOCKFREQUENCY) / MICROSTEPS_PER_REVOLUTION);
constexpr float V_SLOW_IN_MMPS = ((LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM)*M_PI * V_SLOW_ROTATIONS_PER_SEC); // mm per second

constexpr float ROTATIONS_PER_SEC_SQUARED = (LINEFOLLOWERCONFIG_FORWARDS_AMAX / (131072.f / TMC5240CLOCKFREQUENCY));
constexpr float A_MAX_IN_MMPSS = (LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM * M_PI * ROTATIONS_PER_SEC_SQUARED) / MICROSTEPS_PER_REVOLUTION; // mm per s^2

constexpr TickType_t TIME_UNTIL_STANDSTILL_IN_MS = 1000 * (V_MAX_IN_MMPS / A_MAX_IN_MMPSS);
constexpr double DISTANCE_UNTIL_STANDSTILL_IN_MM = (V_MAX_IN_MMPS * V_MAX_IN_MMPS) / (2 * A_MAX_IN_MMPSS);

// Barrier Handling
constexpr float SLOWDOWNDISTANCE_BARRIER_IN_MM = (290.f); // 110
constexpr float BRAKEDISTANCE_BARRIER_IN_MM = DISTANCE_UNTIL_STANDSTILL_IN_MM; // 100
constexpr uint8_t LINEFOLLOWERCONFIG_MEDIANSTACK_SIZE = 11;

constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_mm = (28);
constexpr float LINEFOLLOWERCONFIG_BARRIER_GRIP_DISTANCE_TOLAREANCE_mm = (2);
constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_mm = (350);
constexpr int32_t LINEFOLLOWERCONFIG_BARRIER_SET_BACK_DISTANCE_AFTER_TURN_BACK_mm = (50);