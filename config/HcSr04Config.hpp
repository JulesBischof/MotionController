#pragma once

/* ==============================================================

              Settings HCSR04 Ultrasonic Sensor

============================================================== */

/* --------- select filtering ---------*/
#define HCSR04CONFIG_USE_KALMAN_FILTER (1)

#define HCSR04CONFIG_USE_LOW_PASS_IIR (0)
#define HCSR04CONFIG_LOW_PASS_IIR_ALPHA (0.1f)

#define HCSR04CONFIG_USE_RAW_VALUES (0)

/* ==============================================================

            HcSr04 / Distance Measurment settings

============================================================== */
#define HCSR04CONFIG_POLLING_RATE_MS (60)
#define HCSR04CONFIG_DISTANCE_TRESHHOLD (500)

/* ==============================================================

                KalmanFilter Settings

============================================================== */
constexpr float SPEEDOFSOUND = 343.2f;
constexpr float CONVERSION_FACTOR = 2000.f;

constexpr float INITIAL_DISTANCE = HCSR04CONFIG_DISTANCE_TRESHHOLD;
constexpr float INITIAL_VELOCITY = 0;
constexpr float INITIAL_DT = 0.1;

constexpr float FILTER_QD = 1e-5;    // position
constexpr float FILTER_QV = 1e-9;    // velocity - stepper motors are very percise
constexpr float FILTER_RD = 3.8474f; // measurment noise HC-SR04 - Excel calculations
constexpr float FILTER_RV = 1e-4;    // measurment noise velocity: very percise, acceleration not part of model though

/* ==============================================================

                            Debug

============================================================== */
#define HCSR04CONFIG_PRINTF_RAW_DATA (1)
#define HCSR04CONFIG_PRINTF_FILTEROUTPUT_DATA (0)