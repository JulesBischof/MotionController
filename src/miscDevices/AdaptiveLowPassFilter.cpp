#include "AdaptiveLowPassFilter.hpp"

#include "HcSr04Config.hpp"
#include "LoggerService.hpp"

namespace miscDevices
{
    AdaptiveLowPassFilter::AdaptiveLowPassFilter() { /* default ctor */ };
    AdaptiveLowPassFilter::AdaptiveLowPassFilter(float alphaStandStill, float alphaMoving, float beta, float gamma, float ki, float antiWindup, float initDistance)
        : _alphaStandStill(alphaStandStill), _alphaMoving(alphaMoving), _beta(beta), _gamma(gamma), _ki(ki), _antiWindup(antiWindup), _y_k(initDistance)
    {
        _lastPrediction = get_absolute_time();
        _lastCorrection = get_absolute_time();

        _ci = 1;
        _cv = 1;
        _dxIntegral = 0;
        _barrierDetected = false;
    }

    AdaptiveLowPassFilter::~AdaptiveLowPassFilter()
    {
        /* empty yet */
    }

    float AdaptiveLowPassFilter::getData()
    {
        _prediction();
        return _y_k;
    }

    void AdaptiveLowPassFilter::update(float value)
    {
        // no barrier in sight - ignore
        if (value > HCSR04CONFIG_DISTANCE_TRESHHOLD)
        {
            // services::LoggerService::debug("AdaptiveLowPassFilter::_correction()", "value higher than treshhold - no prediction neccessary");
            _y_k = HCSR04CONFIG_DISTANCE_TRESHHOLD;
            _barrierDetected = false;
            return;
        }
        _barrierDetected = true;
        _prediction();
        _correction(value);
    }

    void AdaptiveLowPassFilter::setVelocity(float velocity)
    {
        _v = velocity;
    }

    void AdaptiveLowPassFilter::_prediction()
    {
        // no barrier in sight - ignore
        if (!_barrierDetected)
            return;

        // convert timedifference us -> s
        float dt = static_cast<float>((absolute_time_diff_us(_lastPrediction, get_absolute_time()) / 1e6));
        _lastPrediction = get_absolute_time();

        // predict - ci & cv are the adaptive prediction constants
        float y_k = y_k - ((_v * _cv) * dt) + _ki * _ci * _dxIntegral;
        // services::LoggerService::debug("AdaptiveLowPassFilter::_prediction", "y_k = %f", _y_k);

        // no velocity? roll off integrations
        if(_v == 0)
        {
            _dxIntegral = _dxIntegral * 0.5;
        }
    }

    void AdaptiveLowPassFilter::_correction(float x_k)
    {
        // choose filter const
        float alpha = _alphaStandStill;
        if (_v != 0)
        {
            alpha = _alphaMoving;
        }

        // lowPass Filter
        _y_k = alpha * x_k + (1 - alpha) * _y_k;

        // update adaptive gains
        float dx = _y_k - x_k;
        _cv = dx / (dx + _beta);
        _ci = dx / (dx + _gamma);

        // integrate
        float dt = static_cast<float>((absolute_time_diff_us(_lastCorrection, get_absolute_time()) / 1e6)); // 1e6 due to us -> s
        _lastCorrection = get_absolute_time();

        _dxIntegral = _dxIntegral * dt;

        if (_dxIntegral > _antiWindup)
        {
            _dxIntegral = _antiWindup;
        }
        else if (_dxIntegral < (-1 * _antiWindup))
        {
            _dxIntegral = -1 * _antiWindup;
        }

        // services::LoggerService::debug("AdaptiveLowPassFilter::_correction", "x_k = %f, _v = %f, _cv = %f, _ci = %f", x_k, _v, _cv, _ci);
    }
}