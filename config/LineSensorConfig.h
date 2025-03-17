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

#define LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES {7880, 8194, 5108, 8192, 5104, 8192, 4357, 4096} 
#define LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES {57186, 56692, 56084, 54546, 56378, 56010, 57440, 57618}

#define LINESENSOR_NORMALIZE_REFERENCE_HIGH (1000)
#define LINESENSOR_MIDDLE_POSITION ((LINESENSOR_NORMALIZE_REFERENCE_HIGH * (NUMBER_OF_CELLS - 1)) >> 2)

#define LINESENSOR_LINE_DETECTED_NORMLIZED ( LINESENSOR_NORMALIZE_REFERENCE_HIGH / 2 )

#endif