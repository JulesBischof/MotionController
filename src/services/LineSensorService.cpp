#include "LineSensorService.hpp"

#include "LineFollowerTaskConfig.hpp"

#include <cmath>

namespace services
{
    int32_t LineSensorService::getVehicleRotation(miscDevices::LineSensor *lineSensor)
    {
        int32_t linePosition = lineSensor->getLinePositionAnalog();

        float s = (linePosition - LINEFOLLOWERCONFIG_CONTROLVALUE_ANALOG) / LINESENSOR_UNITCONVERSION_SENSORVALUE_TO_MM;
        float angle = static_cast<float>((s / (LINEFOLLOWERCONFIG_DISTANCE_LINESENSOR_TO_AXIS_mm*2) ) * (180 / M_PI));

        int32_t retVal = static_cast<int32_t>(angle * 10);// ° * 10 due to prain_uart
        return retVal; 
    }
}