#pragma once

/**
 * @brief Kalman filter for distance and velocity fusion (HC-SR04)
 *
 * Created with assistance from ChatGPT (OpenAI), April 2025.
 * Modifications, integration, and structure provided by the project developer.
 */

namespace miscDevices
{
    class HcSr04KalmanFilter
    {
    public:
        HcSr04KalmanFilter();
        HcSr04KalmanFilter(float init_d, float init_v, float init_dt, float q_d, float q_v, float r_d, float r_v);
        ~HcSr04KalmanFilter();

        void update(float z_distance, float z_velocity, float dt);
        float getDistancePredicted(float dt);

        void setVelocity(float speedMps);
        void setDt(float dt);

    private:
        void _predictionPhase();
        void _correctionPhase(float z[2]);

        float _distance = 0;
        float _velocity = 0;
        float _dt = 0;

        float F[2][2]; // State transition matrix
        float H[2][2]; // Observation matrix
        float Q[2][2]; // Process noise covariance
        float R[2][2]; // Measurement noise covariance
        float P[2][2]; // Estimation error covariance
    };
}