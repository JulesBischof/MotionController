#ifndef LINESENSORCONFIG_H
#define LINESENSORCONFIG_H

/* ==============================================================

                Basic Settings Line Sensor

============================================================== */

constexpr uint8_t NUMBER_OF_CELLS = 8;

constexpr uint8_t LINECOUNTER_CROSS_DETECTED = 6;

constexpr uint8_t LINESENSOR_CONFIG_CALIBRATION_NUMBER_OF_MEASURMENTS = 50;

#define LINESENSOR_CONFIG_PRINTF_CELLVALUES (0)

/* ==============================================================

                    Analog Mode Settings

============================================================== */
constexpr uint16_t LINEPOSITION_ANALOG_MAXIMUN = 7000;
constexpr uint16_t LINEPOSITION_ANALOG_SCALEFACTOR = 1000;

/* --- FLOOR PrenRoom --- */
// #define LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES {24094, 40433, 29936, 11522, 15735, 37041, 33824, 48082}
// #define LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES {48323, 55756, 52206, 44933, 46960, 53968, 56284, 56774}

/* --- FLOOR  --- */
constexpr uint16_t LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES[] = {22284, 40128, 29570, 9748, 14508, 37696, 34277, 48914};
constexpr uint16_t LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES[] = {46988, 54638, 50685, 43371, 49098, 54728, 55278, 57354};

constexpr uint16_t LINESENSOR_NORMALIZE_REFERENCE_HIGH = 1000;
constexpr uint16_t LINESENSOR_MIDDLE_POSITION((LINESENSOR_NORMALIZE_REFERENCE_HIGH * (NUMBER_OF_CELLS - 1)) / 2);

/* --- TRESHHOLDS ---*/
constexpr uint16_t LINESENSOR_LINE_DETECTED_NORMLIZED(0.7 * LINESENSOR_NORMALIZE_REFERENCE_HIGH);
constexpr uint8_t NOLINECOUNTER_MAXVALUE = 10;

constexpr float LINESENSOR_UNITCONVERSION_SENSORVALUE_TO_MM = 40; // 2000 = 50mm

#endif