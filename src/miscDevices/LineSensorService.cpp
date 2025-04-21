#include "LineSensorService.hpp"

#include "LineSensor.hpp"
#include "LineFollowerTaskConfig.h"

#include <cmath>

namespace miscDevices
{
    float LineSensorService::getVehicleRotation(LineSensor *lineSensor)
    {
        uint32_t linePosition = lineSensor->getLinePositionAnalog();

        float s_rad = (LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG - linePosition) * LINESENSOR_UNITCONVERSION_SENSORVALUE_TO_MM;
        float angle = (s_rad / LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm) / (180 / M_PI);

        return angle;
    }
}