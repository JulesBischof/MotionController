#pragma once

#include "Tmc5240.hpp"

namespace MtnCtrl
{
    /// @brief keeps track on how far Robot drove since last measurment
    class MovementTracker
    {
        private:
            int32_t _rotationXActualStartDriver0, _rotationXActualStartDriver1;
            int32_t _distanceXActualStartDriver0, _distanceXActualStartDriver1;

            spiDevices::Tmc5240 *_driver0, *_driver1;

            uint32_t _offsetMm;

        public:
            /// @brief creates in Instance of the MovementTracker class
            /// @param driver0 StepperDriver  0 # tmc5240 expected
            /// @param driver1 StepperDriver  1 # tmc5240 expected
            MovementTracker(spiDevices::Tmc5240 *driver0, spiDevices::Tmc5240 *driver1);
            ~MovementTracker();

            /// @brief initializes the movement tracker
            void init();

            /// @brief resets the rotation counter to 0
            void resetRotation();

            /// @brief resets the distance counter to 0
            void resetDistance();

            /// @brief returns the rotation of the vehicle in degree
            /// @return rotation in degree * 10
            int32_t getRotation();

            /// @brief returns the distance driven in mm
            /// @return distance in mm
            int32_t getDistance();

            /// @brief adds an Offset to the driven Distanze.
            /// might be e.g. due to Barrier-Handling
            void addOffsetMillimeters(uint16_t offset);
    };
}