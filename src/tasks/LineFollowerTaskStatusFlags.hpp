#pragma once

#include "pico/stdlib.h"

#include "Tmc5240Config.h"
#include "LineFollowerTaskConfig.h"
#include <cmath>

namespace MtnCtrl
{
    /* ================================= */
    /*            consts                 */
    /* ================================= */
    constexpr float V_MAX_IN_MMPS = (STEPPERCONFIG_WHEEL_DIAMETER_MM * M_PI * LINEFOLLERCONFIG_VMAX_STEPSPERSEC) / MICROSTEPS_PER_REVOLUTION;    // mm per second
    constexpr float A_MAX_IN_MMPSS = (STEPPERCONFIG_WHEEL_DIAMETER_MM * M_PI * LINEFOLLERCONFIG_AMAX_STEPSPERSECSQUARED) / MICROSTEPS_PER_REVOLUTION; // mm per s^2

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

}