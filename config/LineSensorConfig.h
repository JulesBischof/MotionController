#ifndef LINESENSORCONFIG_H
#define LINESENSORCONFIG_H

/* ==============================================================

                Basic Settings Line Sensor

============================================================== */

#define NUMBER_OF_CELLS (8)

#define ADC_TRESHHOLD (30000)

#define LINECOUNTER_CROSS_DETECTED (4)

/* ==============================================================

                    Analog Mode Settings

============================================================== */
#define LINEPOSITION_ANALOG_MAXIMUN (8000)
#define LINEPOSITION_ANALOG_SCALEFACTOR (1000)

#define LINESENSOR_DEFAULT_CALIBRATION_LOW_VALUES {50, 50, 50, 50, 50, 50, 50, 50} 
#define LINESENSOR_DEFAULT_CALIBRATION_HIGH_VALUES {5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000} 

#endif