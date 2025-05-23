#pragma once

#include "pico/stdlib.h"
#include "time.h"

namespace miscDevices
{
    /// @brief AdaptiveLowPassFilter filters sensor data by combining prediction and correction phases.
    ///
    /// This filter is designed to handle noisy distance measurements (e.g. from ultrasonic sensor hcsr04)
    /// and adapt its behavior depending on the motion state (stationary or moving).
    class AdaptiveLowPassFilter
    {
    private:
        /// @brief Performs the prediction step based on previous velocity and integration.
        void _prediction();

        /// @brief Performs the correction step based on new sensor input.
        /// @param value The new measured sensor value to correct the state with.
        void _correction(float value);

        float _alphaStandStill;
        float _alphaMoving;
        float _beta;
        float _gamma;
        float _antiWindup;

        float _cv;
        float _v;
        float _y_k;
        float _ki;
        float _ci;
        float _dxIntegral;

        absolute_time_t _lastPrediction; 
        absolute_time_t _lastCorrection; 

        float _barrierDetected; 

    public:
        /// @brief Default constructor.
        AdaptiveLowPassFilter();

        /// @brief Parameterized constructor for detailed filter configuration.
        ///
        /// @param alphaStandStill Filter factor when stationary.
        /// @param alphaMoving Filter factor when moving.
        /// @param beta Gain adaptation factor for velocity.
        /// @param gamma Gain adaptation factor for integration.
        /// @param ki Integral gain.
        /// @param antiWindup Anti-windup limit.
        /// @param initDistance Initial sensor reading to initialize state.
        AdaptiveLowPassFilter(float alphaStandStill, float alphaMoving, float beta, float gamma, float ki, float antiWindup, float initDistance);

        /// @brief Destructor.
        ~AdaptiveLowPassFilter();

        /// @brief Retrieves the filtered distance value.
        ///
        /// Internally performs a prediction step before returning the state.
        /// @return The current filtered sensor value.
        float getData();

        /// @brief Updates the filter with a new raw sensor measurement.
        ///
        /// If the value exceeds a distance threshold, it's considered invalid.
        /// Otherwise, prediction and correction steps are applied.
        /// @param value New sensor input value.
        void update(float value);

        /// @brief Sets the current velocity used in prediction.
        ///
        /// @param velocity Velocity in appropriate units (e.g., m/s).
        void setVelocity(float velocity);
    };
}