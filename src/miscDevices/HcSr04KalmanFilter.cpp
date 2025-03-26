#include "HcSr04KalmanFilter.hpp"

HcSr04KalmanFilter::HcSr04KalmanFilter()
{
    /* default ctor */
}

HcSr04KalmanFilter::HcSr04KalmanFilter(double init_d, double init_v, double init_dt, double q_d, double q_v, double r)
{
    _dt = init_dt;
    _velocity = init_v;
    _distance = init_d;

    // Initialize state transition matrix
    F[0][0] = 1;
    F[0][1] = -1 * _dt;
    F[1][0] = 0;
    F[1][1] = 1;

    // Initialize observation matrix
    H[0] = 1;
    H[1] = 0;

    // Initialize process noise covariance
    Q[0][0] = q_d;
    Q[0][1] = 0;
    Q[1][0] = 0;
    Q[1][1] = q_v;

    // Measurement noise variance
    R = r;

    // Initialize error covariance matrix
    P[0][0] = 1; // Initial value for position uncertainty
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 1; // Initial value for velocity uncertainty
}

HcSr04KalmanFilter::~HcSr04KalmanFilter()
{
    /* not implemented yet */
}

void HcSr04KalmanFilter::_predictionPhase()
{
    double d_pred = F[0][0] * _distance + F[0][1] * _velocity;
    double v_pred = F[1][0] * _distance + F[1][1] * _velocity;

    _distance = d_pred;
    _velocity = v_pred;

    // update covariance Matrix
    double P00 = F[0][0] * P[0][0] + F[0][1] * P[1][0];
    double P01 = F[0][0] * P[0][1] + F[0][1] * P[1][1];
    double P10 = F[1][0] * P[0][0] + F[1][1] * P[1][0];
    double P11 = F[1][0] * P[0][1] + F[1][1] * P[1][1];

    P[0][0] = P00 + Q[0][0];
    P[0][1] = P01 + Q[0][1];
    P[1][0] = P10 + Q[1][0];
    P[1][1] = P11 + Q[1][1];
}

void HcSr04KalmanFilter::_correctionPhase(double z)
{
    // calc innovation
    double y = z - (H[0] * _distance + H[1] * _velocity);

    // innovation covariance
    double S = H[0] * P[0][0] + H[1] * P[1][0] + R;

    // kalman-gain
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // state correction
    _distance = _distance + K[0] * y;
    _velocity = _velocity + K[1] * y;

    // covariance correction
    double P00 = P[0][0] - K[0] * H[0] * P[0][0];
    double P01 = P[0][1] - K[0] * H[0] * P[0][1];
    double P10 = P[1][0] - K[1] * H[1] * P[1][0];
    double P11 = P[1][1] - K[1] * H[1] * P[1][1];

    P[0][0] = P00;
    P[0][1] = P01;
    P[1][0] = P10;
    P[1][1] = P11;
}

void HcSr04KalmanFilter::setVelocity(double speedMps)
{
    this->_velocity = speedMps;
}
void HcSr04KalmanFilter::setDt(double dt)
{
    this->F[0][1] = -1 * dt;
}

void HcSr04KalmanFilter::update(double z, double dt)
{
    setDt(dt);
    _predictionPhase();
    _correctionPhase(z);
}

float HcSr04KalmanFilter::getDistance()
{
    return _distance;
}
