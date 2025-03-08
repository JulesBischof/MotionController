// for Stepper 42BYGH40-1.8-23A

/*
I_rms = 1.7 A / Phase
I_peak = 1.7A * 1.41 = 2.397 A
U_rated = 3.4V
*/

/* ####################################################################

                        BASIC CURRENT SETTINGS

    #################################################################### */

#ifndef MOTORSETTINGS
#define MOTORSETTINGS

/* ====================================================================
    SET CURRENTRANGE IN DRV_CONF (AND IREF) TO FIT MAXIMUM MOTOR CURRENT
   ==================================================================== */
#define CURRENT_RANGE 2 // 3 A peak, not rms! (1.7 A * 1.41 = 2.4A)

/* ====================================================================
    SET GLOBALSCALER AS REQUIRED TO REACH MAXIMUM MOTOR CURRENT AT I_RUN = 31; percantage, finetune motor current
   ==================================================================== */
#define GLOBSCALER 255 // max = 255!

/* ====================================================================
    SET IRUN AS DESIRED UP TO 31, IHOLD 70% OF IRUN OR LOWER
    ==================================================================== */
#define IRUN 31 // 31 = 1.6A_rms; 19 = 1A_rms  // percentage of full scale current while motor in run -> (IRUN / 32)%
#define IHOLD 3 // standstill current

/* ====================================================================
    SET IRUNDELAY TO 1 TO 15 FOR REDUCED CURRENT PEAK AT MOTOR START UP
    ==================================================================== */
#define IRUNDELAY 0 // Delay in IRUNDELAY * 512 clocks

/* ====================================================================
    SET IHOLDDELAY TO 1 TO 15 FOR SMOOTH STANDSTILL CURRENT DECAY
    ==================================================================== */
#define IHOLDDELAY 7 // Delay per current reduction step in multiple of 2^18 clocks

/* ====================================================================
    SET TPOWERDOWN UP TO 255 FOR DELAYED STANDSTILL CURRENT REDUCTION
    ==================================================================== */
#define TPOWERDOWN 10 // sets the delay time after stand still (stst) of the motor to motor current power down. Time range is about 0 to 4 seconds

/*
Set I_ref to 16k (R2 = HIGH, R3 = LOW)
*/

/* ####################################################################

                        SPREAD CYCLE SETTINGS

    #################################################################### */
#define GCONF 0 // Stealth Chop disable, Spread Cycle active

/* ====================================================================
    ENABLE CHOPPER USING BASIC CONFIG
    ==================================================================== */

#define TOFF 4
#define TBL 2
#define HSTRT 6
#define HEND 3

/* ####################################################################

                        UNIT CONVERSION

    #################################################################### */

#define STEPPERCONFIG_NR_FULLSTEPS_PER_TURN 200
#define STEPPERCONFIG_MICROSTEPPING 16

#define STEPPERCONFIG_WHEEL_DIAMETER_MM 800

#define STEPPERCONFIG_WHEELDISTANCE_AXIS_MM 800

#endif