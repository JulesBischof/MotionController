#ifndef HCSR04KALMANFILTER_H
#define HCSR04KALMANFILTER_H

#include "pico/stdlib.h"

class HcSr04KalmanFilter
{
private:
    double _distance, _velocity, _dt;

    double F[2][2], H[2], Q[2][2], R, P[2][2], K[2];

    void _predictionPhase();
    void _correctionPhase(double z);

public:
    HcSr04KalmanFilter();
    HcSr04KalmanFilter(double init_d, double init_v, double init_dt, double q_d, double q_v, double r);
    ~HcSr04KalmanFilter();

    void setVelocity(double speedMps);
    void setDt(double dt);

    void update(double z, double dt);
    float getDistance();
};

#endif // HCSR04KALMANFILTER