#pragma once

namespace miscDevices
{
    class LineSensorService
    {
        public:
            /// @brief determines the actual rotation of the vahicle based on the line position
            /// @param lineSensor current instance of the lineSensor
            /// @return rotation in degree
            static float getVehicleRotation(LineSensor *lineSensor);
    };
}