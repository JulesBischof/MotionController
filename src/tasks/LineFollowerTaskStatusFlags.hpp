#pragma once

#include "Tmc5240Config.h"
#include "LineFollowerTaskConfig.h"
#include <cmath>


/* ================================= */
/*            consts                 */
/* ================================= */
constexpr float V_MAX_IN_MMPS = (STEPPERCONFIG_WHEEL_DIAMETER_MM * M_PI * LINEFOLLERCONFIG_VMAX_STEPSPERSEC_FAST) / MICROSTEPS_PER_REVOLUTION; // mm per second
constexpr float A_MAX_IN_MMPSS = (STEPPERCONFIG_WHEEL_DIAMETER_MM * M_PI * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED) / MICROSTEPS_PER_REVOLUTION; // mm per s^2

/* ================================= */
/*           status Flags            */
/* ================================= */

constexpr uint32_t RUNMODEFLAG_T_UPPER_BITMASK(0xFFFF0000);
constexpr uint32_t RUNMODEFLAG_T_LOWER_BITMASK(0x0000FFFF);

enum RunModeFlag
{
    // lower 16 bits statemaschine relevant flags
    MOTOR_RUNNING = 1 << 0,
    MOTOR_POSITIONMODE = 1 << 1,
    MOTOR_POSITIONMODE_REQUEST_SEND = 1 << 2,
    MOTOR_STOPREQUEST_SEND = 1 << 3,
    RUNMODE_SLOW = 1 << 4,
    LINE_FOLLOWER_MODE = 1 << 5,
    TURN_MODE = 1 << 6,
    TURNREQUEST_SEND = 1 << 7,
    STATUSFLAGS_SEND = 1 << 8,
    LINEFOLLOWER_FIND_LINE = 1 << 11,
    MOTORS_AT_STANDSTILL = 1 << 12,

    // upper 16 bits events and infos
    CROSSPOINT_DETECTED = 1 << 16,
    LOST_LINE = 1 << 17,
    POSITION_REACHED = 1 << 18,
    SAFETY_BUTTON_PRESSED = 1 << 19,
    LINEFOLLOWER_ERROR = 1 << 20,
    LINEFOLLOWER_BARRIER_DETECTED = 1 << 21,

    MOTION_STOPPED = 1 << 30,           // TODO set
    LINEFOLLOWER_COMMAND_ACK = 1 << 31, // TODO set

} RunModeFlag;

/* TODO Flags 4 Errors - upper 16 RunModeFlags get send as Info, not as Error */

constexpr uint32_t STM_LINEFOLLOWER_BITSET = 0 | (MOTOR_RUNNING | LINE_FOLLOWER_MODE);
constexpr uint32_t STM_MOVE_POSITIONMODE_BITSET = 0 | (MOTOR_RUNNING | MOTOR_POSITIONMODE);
constexpr uint32_t STM_STOPMOTOR_BITSET = 0;
constexpr uint32_t STM_TURNROBOT_BITSET = 0 | (TURN_MODE | MOTOR_RUNNING);
constexpr uint32_t STM_FIND_LINE_BITSET = 0 | (LINEFOLLOWER_FIND_LINE | MOTOR_RUNNING);