#pragma once

#include "pico/stdlib.h"

#include "Tmc5240Config.hpp"
#include "LineFollowerTaskConfig.hpp"
#include <cmath>

#include "FreeRTOS.h"

namespace MtnCtrl
{
    /* ================================= */
    /*           status Flags            */
    /* ================================= */

    constexpr uint32_t RUNMODEFLAG_T_UPPER_BITMASK(0xFFFF0000);
    constexpr uint32_t RUNMODEFLAG_T_LOWER_BITMASK(0x0000FFFF);

    enum class RunModeFlag : uint32_t
    {
        // lower 16 bits statemaschine relevant flags
        MOTORS_AT_STANDSTILL = 1 << 0,

        // upper 16 bits events and infos for raspberry hat
        CROSSPOINT_DETECTED = 1 << 16,
        LOST_LINE = 1 << 17,
        POSITION_REACHED = 1 << 18,
        SAFETY_BUTTON_PRESSED = 1 << 19,
        LINEFOLLOWER_ERROR = 1 << 20,
        LINEFOLLOWER_BARRIER_DETECTED = 1 << 21,
    };

    /* ================================= */
    /*            consts                 */
    /* ================================= */

    constexpr float ROTATIONS_PER_SEC = (LINEFOLLERCONFIG_VMAX_REGISTER_VALUE / (16777216.f / TMC5240CLOCKFREQUENCY) / MICROSTEPS_PER_REVOLUTION);
    constexpr float V_MAX_IN_MMPS = ( (LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM)* M_PI * ROTATIONS_PER_SEC); // mm per second

    constexpr float ROTATIONS_PER_SEC_SQUARED = (LINEFOLLERCONFIG_AMAX_REGISTER_VALUE / (131072.f / TMC5240CLOCKFREQUENCY));
    constexpr float A_MAX_IN_MMPSS = (LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM * M_PI * ROTATIONS_PER_SEC_SQUARED) / MICROSTEPS_PER_REVOLUTION; // mm per s^2

    constexpr TickType_t TIME_UNTIL_STANDSTILL_IN_MS = 1000 *(V_MAX_IN_MMPS / A_MAX_IN_MMPSS);
}