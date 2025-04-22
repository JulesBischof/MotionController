#pragma once

namespace spiDevices
{
    /// @brief service class providing methods to convert units & Co
    class StepperService
    {
    public:
        /// @brief convert mm to usteps
        /// @param distance distance in [mm]
        /// @return microsteps
        static int32_t convertMillimeterToMicrosteps(float meter);

        /// @brief convert ustep/s in cm
        /// @param uSteps microsteps
        /// @return Distance in Centimeter
        static int32_t convertMicrostepsToMillimeter(uint32_t uSteps);

        /// @brief convert degree to microsteps
        /// @param degrees float degrees
        /// @return value of Microsteps
        static int32_t convertDegreeToMicrosteps(int32_t degrees);

        /// @brief calcs rotatinal angle given by driven distances of 2 wheels on one axis
        /// @param uStepsDifference step difference in Microsteps!
        /// @return angle
        static float convertDeltaDrivenDistanceToDegree(int32_t uStepsDifference);
    };
}