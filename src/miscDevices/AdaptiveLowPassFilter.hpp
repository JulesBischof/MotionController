# pragma once

#include "pico/stdlib.h"
#include "time.h"

namespace miscDevices
{
    class AdaptiveLowPassFilter
    {
        private:
            void _prediction();
            void _correction(float value);

            float _alphaStandStill, _alphaMoving, _beta, _gamma, _antiWindup;
            float _cv, _v, _y_k, _ki, _ci, _dxIntegral;

            absolute_time_t _lastPrediction, _lastCorrection;

            float _barrierDetected;

        public:
            AdaptiveLowPassFilter();
            AdaptiveLowPassFilter(float alphaStandStill, float alphaMoving, float beta, float gamma, float ki, float antiWindup, float initDistance);
            ~AdaptiveLowPassFilter();

            float getData();
            void update(float value);
            void setVelocity(float velocity);
    };
}