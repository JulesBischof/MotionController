#pragma once

/* ==============================================================

              Settings HCSR04 Ultrasonic Sensor

============================================================== */

/* --------- select filtering ---------*/
#define HCSR04CONFIG_USE_KALMAN_FILTER (1)

#define HCSR04CONFIG_USE_LOW_PASS_IIR (1)
#define HCSR04CONFIG_LOW_PASS_IIR_ALPHA (0.1f)

#define HCSR04CONFIG_USE_RAW_VALUES (1)