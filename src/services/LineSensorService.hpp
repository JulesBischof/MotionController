#pragma once

#include "LineSensor.hpp"

namespace services
{
    class LineSensorService
    {
        public:
            /// @brief determines the actual rotation of the vahicle based on the line position
            /// @param lineSensor current instance of the lineSensor
            /// @return rotation in 10 * degree
            static int32_t getVehicleRotation(miscDevices::LineSensor *lineSensor);
    };
}