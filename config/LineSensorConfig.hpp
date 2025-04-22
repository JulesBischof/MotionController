#ifndef LINESENSORCONFIG_H
#define LINESENSORCONFIG_H

/* ==============================================================

                Basic Settings Line Sensor

============================================================== */

#define NUMBER_OF_CELLS (8)

#define ADC_RAW_LINE_TRESHHOLD (30000)

#define LINECOUNTER_CROSS_DETECTED (4)

#define LINECOUNTER_MAX_VALUE (3)

/* ==============================================================

                    Analog Mode Settings

============================================================== */
#define LINEPOSITION_ANALOG_MAXIMUN (8000)
#define LINEPOSITION_ANALOG_SCALEFACTOR (1000)

#define LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES {44834, 28530, 28044, 17340, 18708, 19010, 12514, 13808}
#define LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES {55518, 51438, 50516, 45626, 48096, 46678, 46582, 49748}

#define LINESENSOR_NORMALIZE_REFERENCE_HIGH (1000)
#define LINESENSOR_MIDDLE_POSITION ((LINESENSOR_NORMALIZE_REFERENCE_HIGH * (NUMBER_OF_CELLS - 1)) >> 2)

#define LINESENSOR_LINE_DETECTED_NORMLIZED ( LINESENSOR_NORMALIZE_REFERENCE_HIGH / 2 )

#define LINESENSOR_UNITCONVERSION_SENSORVALUE_TO_MM (40) // 2000 = 50mm

#endif