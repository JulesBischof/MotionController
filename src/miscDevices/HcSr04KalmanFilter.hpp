#ifndef HCSR04KALMANFILTER_H
#define HCSR04KALMANFILTER_H

#include "pico/stdlib.h"

class HcSr04KalmanFilter
{
private:
    float _distance, _velocity, _dt;

    float F[2][2], H[2], Q[2][2], R, P[2][2], K[2];

    void _predictionPhase();
    void _correctionPhase(float z);

public:
    HcSr04KalmanFilter();
    HcSr04KalmanFilter(float init_d, float init_v, float init_dt, float q_d, float q_v, float r);
    ~HcSr04KalmanFilter();

    void setVelocity(float speedMps);
    void setDt(float dt);

    void update(float z, float dt);
    float getDistance();
};

#endif // HCSR04KALMANFILTER