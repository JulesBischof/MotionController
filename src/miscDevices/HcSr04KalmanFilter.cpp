#include "HcSr04KalmanFilter.hpp"

namespace miscDevices
{
    HcSr04KalmanFilter::HcSr04KalmanFilter()
    {
        /* default ctor */
    }

    HcSr04KalmanFilter::HcSr04KalmanFilter(float init_d, float init_v, float init_dt, float q_d, float q_v, float r_d, float r_v)
    {
        _dt = init_dt;
        _velocity = init_v;
        _distance = init_d;

        // State transition matrix
        F[0][0] = 1;
        F[0][1] = -_dt;
        F[1][0] = 0;
        F[1][1] = 1;

        // Observation matrix (identity)
        H[0][0] = 1;
        H[0][1] = 0;
        H[1][0] = 0;
        H[1][1] = 1;

        // Process noise covariance
        Q[0][0] = q_d;
        Q[0][1] = 0;
        Q[1][0] = 0;
        Q[1][1] = q_v;

        // Measurement noise covariance
        R[0][0] = r_d;
        R[0][1] = 0;
        R[1][0] = 0;
        R[1][1] = r_v;

        // Initial error covariance
        P[0][0] = 1;
        P[0][1] = 0;
        P[1][0] = 0;
        P[1][1] = 1;
    }

    HcSr04KalmanFilter::~HcSr04KalmanFilter() {}

    void HcSr04KalmanFilter::setVelocity(float speedMps)
    {
        _velocity = speedMps;
    }

    void HcSr04KalmanFilter::setDt(float dt)
    {
        _dt = dt;
        F[0][1] = -_dt;
    }

    void HcSr04KalmanFilter::_predictionPhase()
    {
        float d_pred = F[0][0] * _distance + F[0][1] * _velocity;
        float v_pred = F[1][0] * _distance + F[1][1] * _velocity;

        _distance = d_pred;
        _velocity = v_pred;

        // Update error covariance
        float P00 = F[0][0] * P[0][0] + F[0][1] * P[1][0];
        float P01 = F[0][0] * P[0][1] + F[0][1] * P[1][1];
        float P10 = F[1][0] * P[0][0] + F[1][1] * P[1][0];
        float P11 = F[1][0] * P[0][1] + F[1][1] * P[1][1];

        P[0][0] = P00 + Q[0][0];
        P[0][1] = P01 + Q[0][1];
        P[1][0] = P10 + Q[1][0];
        P[1][1] = P11 + Q[1][1];
    }

    void HcSr04KalmanFilter::_correctionPhase(float z[2])
    {
        float y[2];
        y[0] = z[0] - _distance;
        y[1] = z[1] - _velocity;

        float S[2][2];
        S[0][0] = P[0][0] + R[0][0];
        S[0][1] = P[0][1] + R[0][1];
        S[1][0] = P[1][0] + R[1][0];
        S[1][1] = P[1][1] + R[1][1];

        float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        float S_inv[2][2] = {
            {S[1][1] / det, -S[0][1] / det},
            {-S[1][0] / det, S[0][0] / det}};

        float K[2][2];
        K[0][0] = P[0][0] * S_inv[0][0] + P[0][1] * S_inv[1][0];
        K[0][1] = P[0][0] * S_inv[0][1] + P[0][1] * S_inv[1][1];
        K[1][0] = P[1][0] * S_inv[0][0] + P[1][1] * S_inv[1][0];
        K[1][1] = P[1][0] * S_inv[0][1] + P[1][1] * S_inv[1][1];

        _distance += K[0][0] * y[0] + K[0][1] * y[1];
        _velocity += K[1][0] * y[0] + K[1][1] * y[1];

        float I_KH[2][2] = {
            {1 - K[0][0], -K[0][1]},
            {-K[1][0], 1 - K[1][1]}};

        float newP[2][2];
        newP[0][0] = I_KH[0][0] * P[0][0] + I_KH[0][1] * P[1][0];
        newP[0][1] = I_KH[0][0] * P[0][1] + I_KH[0][1] * P[1][1];
        newP[1][0] = I_KH[1][0] * P[0][0] + I_KH[1][1] * P[1][0];
        newP[1][1] = I_KH[1][0] * P[0][1] + I_KH[1][1] * P[1][1];

        P[0][0] = newP[0][0];
        P[0][1] = newP[0][1];
        P[1][0] = newP[1][0];
        P[1][1] = newP[1][1];
    }

    void HcSr04KalmanFilter::update(float z_distance, float z_velocity, float dt)
    {
        setDt(dt);
        _predictionPhase();

        float z[2] = {z_distance, z_velocity};
        _correctionPhase(z);
    }

    float HcSr04KalmanFilter::getDistancePredicted(float dt)
    {
        setDt(dt);
        _predictionPhase();
        return _distance;
    }
}
