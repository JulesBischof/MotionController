#include "StepperService.hpp"

#include "LineFollowerTaskConfig.h"
#include "Tmc5240Config.h"
#include <cmath>
#include "pico/stdlib.h"

namespace spiDevices
{
    constexpr float MICROSTEPSPERREVOLUTION = STEPPERCONFIG_NR_FULLSTEPS_PER_TURN * STEPPERCONFIG_MICROSTEPPING;
    constexpr float WHEELCIRCUMFENCE_MM = LINEFOLLOWERCONFIG_WHEEL_DIAMETER_MM * M_PI;

    int32_t StepperService::convertDistanceMmToMicrosteps(float distance)
    {
        float res = (distance / WHEELCIRCUMFENCE_MM) * MICROSTEPSPERREVOLUTION;

        int32_t retVal = static_cast<int32_t>(std::round(res));
        return retVal;
    }

    int32_t StepperService::convertMicrostepsToCentimeter(uint32_t uSteps)
    {
        float res = (uSteps * MICROSTEPSPERREVOLUTION) * WHEELCIRCUMFENCE_MM;

        int32_t retVal = static_cast<int32_t>(std::round(res));
        return retVal;
    }

    int32_t StepperService::convertDegreeToMicrosteps(int32_t degrees)
    {
        float usps = (degrees / 360.0f) * MICROSTEPS_PER_REVOLUTION;
        return static_cast<int32_t>(round(usps));
    }

    float StepperService::convertDeltaDrivenDistanceToDegree(int32_t uStepsDifference)
    {
        float dsMeters = convertMicrostepsToCentimeter(uStepsDifference) * 1e2;

        float res = (dsMeters * 180.0f) / (WHEELCIRCUMFENCE_MM);

        return res;
    }

}