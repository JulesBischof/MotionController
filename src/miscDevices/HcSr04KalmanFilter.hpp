#ifndef HCSR04KALMANFILTER_H
#define HCSR04KALMANFILTER_H

#include "pico/stdlib.h"

namespace miscDevices
{
    /// @brief Object that represents a KalmannFilter for an HcSr04 Distance Sensor
    class HcSr04KalmanFilter
    {
    private:
        float _distance, _velocity, _dt;

        float F[2][2], H[2], Q[2][2], R, P[2][2], K[2];

        void _predictionPhase();
        void _correctionPhase(float z);

    public:
        /// @brief default ctor
        HcSr04KalmanFilter();

        /// @brief ctor for HcSr04 Kalmann Filter
        /// @param init_d initial distance
        /// @param init_v initial velocity in mm/sec
        /// @param init_dt initial timedifference between measurments
        /// @param q_d processnoise - distance measurment
        /// @param q_v processnoide - velocity
        /// @param r covariance sensor
        HcSr04KalmanFilter(float init_d, float init_v, float init_dt, float q_d, float q_v, float r);
        ~HcSr04KalmanFilter();

        /// @brief sets a new velocity to kalmann filter
        /// @param speedMmps // speed in mm per second
        void setVelocity(float speedMmps);

        /// @brief sets a new timedifference between two measurments
        /// @param dt timedifference in seconds
        void setDt(float dt);

        /// @brief updates the Kalmann Filter with a new value
        /// @param z new sensor data
        /// @param dt time increment since last measurment in ms
        void update(float z, float dt);

        /// @brief get the current value the Filter is able to print
        /// @param dt time increment since last measurment in ms
        /// @return current distance in mm
        float getDistancePredicted(float dt);

        /// @brief get the current value the Filter is able to print
        /// @return current distance in mm
        float getDistance() { return _distance; }
    };

}
#endif // HCSR04KALMANFILTER