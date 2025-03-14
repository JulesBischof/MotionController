#ifndef LINESENSORCONFIG_H
#define LINESENSORCONFIG_H

/* ==============================================================

                Basic Settings Line Sensor

============================================================== */

#define NUMBER_OF_CELLS (8)

#define ADC_RAW_LINE_TRESHHOLD (35000)

#define LINECOUNTER_CROSS_DETECTED (4)

/* ==============================================================

                    Analog Mode Settings

============================================================== */
#define LINEPOSITION_ANALOG_MAXIMUN (8000)
#define LINEPOSITION_ANALOG_SCALEFACTOR (1000)

#define LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES {50, 50, 50, 50, 50, 50, 50, 50} 
#define LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES {5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000}

#define LINESENSOR_NORMALIZE_REFERENCE_HIGH (1000)
#define LINESENSOR_MIDDLE_POSITION ((LINESENSOR_NORMALIZE_REFERENCE_HIGH * (NUMBER_OF_CELLS - 1)) >> 2)

#define LINESENSOR_LINE_DETECTED_NORMLIZED ( LINESENSOR_NORMALIZE_REFERENCE_HIGH / 2 )

#endif