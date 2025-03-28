#include "Tmc5240.hpp"

#include <cmath>

#include "TMC5240_HW_Abstraction.h"
#include "Tmc5240Config.h"

#include "pico/stdlib.h"
#include <stdio.h>

constexpr float MICROSTEPSPERREVOLUTION = STEPPERCONFIG_NR_FULLSTEPS_PER_TURN * STEPPERCONFIG_MICROSTEPPING;
constexpr float WHEELCIRCUMFENCE = (STEPPERCONFIG_WHEEL_DIAMETER_MM / 1e3) * M_PI;

/* ==================================
      Constructor / Deconstructor
   ================================== */

/// @brief creates instance of Trinamics TMC5240 stepper driver
/// @param spiInstance spi instance - refer pico c/c++ sdk
/// @param csPin chip select pin
Tmc5240::Tmc5240(spi_inst_t *spiInstance, uint8_t csPin, bool stdDir) : SpiBase(spiInstance, csPin)
{
    _stdDir = stdDir;

    printf("Startup TMC5240 CS_GPIO #%d ... \n", csPin);
    _initDevice();
    _checkDevice();

    this->moveVelocityMode(0, 0, 5000); // initially Stop running Motors
}

Tmc5240::~Tmc5240()
{
    // DECONSTRUCTOR not implemented yet
}

/// @brief default constructor Tmc5240 MotorDrives
Tmc5240::Tmc5240() {}

/* ==================================
            Init members
   ================================== */

/// @brief initializes device
void Tmc5240::_initDevice()
{
    printf("resetting TMC5240 Device... \n");
    clearGSTAT();

    printf("start init Current settings TMC5240 ... \n");
    _initCurrentSetting();

    printf("init SpreadCycle Mode ... \n");
    _initSpreadCycle();

    printf("initialisation TMC5240 DONE! \n");
}

/// @brief checks, if the device is still reachable
void Tmc5240::_checkDevice()
{
    uint8_t version = _spiReadBitField(TMC5240_INP_OUT, TMC5240_VERSION_MASK, TMC5240_VERSION_SHIFT);
    printf("TMC5240 Version %d initialized! \n", version);

    bool drvEnn = _spiReadBitField(TMC5240_INP_OUT, TMC5240_DRV_ENN_MASK, TMC5240_DRV_ENN_SHIFT);
    printf("DRV_ENN State = %x \n", drvEnn);

    uint32_t gStat = getGSTAT();
    printf("GSTAT val = %x", gStat);
}

/// @brief inits Current settings out of StepperConfig.h File
void Tmc5240::_initCurrentSetting()
{
    uint32_t DRV_CONF_val = _spiReadReg(TMC5240_DRV_CONF);
    DRV_CONF_val |= CURRENT_RANGE;
    _spiWriteReg(TMC5240_DRV_CONF, DRV_CONF_val);

    _spiWriteReg(TMC5240_GLOBAL_SCALER, GLOBSCALER);

    uint32_t IHOLD_IRUN_val = _spiReadReg(TMC5240_IHOLD_IRUN);
    IHOLD_IRUN_val |= IHOLD << TMC5240_IHOLD_SHIFT;
    IHOLD_IRUN_val |= IRUN << TMC5240_IRUN_SHIFT;
    IHOLD_IRUN_val |= IHOLDDELAY << TMC5240_IHOLDDELAY_SHIFT;
    IHOLD_IRUN_val |= IRUNDELAY << TMC5240_IRUNDELAY_SHIFT;
    _spiWriteReg(TMC5240_IHOLD_IRUN, IHOLD_IRUN_val);

    _spiWriteReg(TMC5240_TPOWERDOWN, TPOWERDOWN);

    uint32_t drv_conf_val = _spiReadReg(TMC5240_DRV_CONF);
    uint32_t globscalerval = _spiReadReg(TMC5240_GLOBAL_SCALER);
    uint32_t ihold_irun_val = _spiReadReg(TMC5240_IHOLD_IRUN);

    printf("Current Settings Register Values \n --- GLOBSCALER ... 0x%x \n --- DRV_CONF ... 0x%x \n --- IHOLD_IRUN ... %x \n", globscalerval, drv_conf_val, ihold_irun_val);
}

/// @brief initializes stepper drive in SpreadCycle Mode - ref Datasheet
void Tmc5240::_initSpreadCycle()
{
    uint32_t GCONF_val = _spiReadReg(TMC5240_GCONF);
    GCONF_val |= GCONF << TMC5240_EN_PWM_MODE_SHIFT;
    _spiWriteReg(TMC5240_GCONF, GCONF_val);

    uint32_t CHOPCONF_val = _spiReadReg(TMC5240_CHOPCONF);
    CHOPCONF_val |= TOFF << TMC5240_TOFF_SHIFT;
    CHOPCONF_val |= HSTRT << TMC5240_SMALL_HYSTERESIS_SHIFT;
    CHOPCONF_val |= HEND << TMC5240_HEND_OFFSET_SHIFT;
    _spiWriteReg(TMC5240_CHOPCONF, CHOPCONF_val);

    return;
}

/* ==================================
         control targetdevice
   ================================== */

/// @brief moves Motor to specific location using position mode
/// @param xTargetVal ABSOLUTE position! in steps
/// @param vmax maximum velocity
/// @param amax maxmimum acceleration
void Tmc5240::moveAbsolutePositionMode(int32_t xTargetVal, uint32_t vmax, uint32_t amax)
{
    // activate position mode
    _spiWriteReg(TMC5240_RAMPMODE, TMC5240_MODE_POSITION);

    // set vmax, amax, dmax
    _spiWriteReg(TMC5240_VMAX, vmax);
    _spiWriteReg(TMC5240_AMAX, amax);
    _spiWriteReg(TMC5240_DMAX, amax);
    _spiWriteReg(TMC5240_VSTART, 0);
    _spiWriteReg(TMC5240_VSTOP, 101);

    // set Target position
    _spiWriteReg(TMC5240_XTARGET, xTargetVal);
}

/// @brief moves Motor to specific location using position mode
/// @param xTargetVal RELATIVE position! in steps
/// @param vmax maximum velocity
/// @param amax maxmimum acceleration
void Tmc5240::moveRelativePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax, bool dir)
{
    // activate position mode
    _spiWriteReg(TMC5240_RAMPMODE, TMC5240_MODE_POSITION);

    // set vmax, amax, dmax
    _spiWriteReg(TMC5240_VMAX, vmax);
    _spiWriteReg(TMC5240_AMAX, amax);
    _spiWriteReg(TMC5240_DMAX, amax);
    _spiWriteReg(TMC5240_VSTART, 0);
    _spiWriteReg(TMC5240_VSTOP, 101);

    // get XACTUAL - Register value
    int32_t xActualVal = getXActual();

    // set new target position
    int32_t xTargetVal = 0;
    if (dir)
    {
        xTargetVal = xActualVal + targexPos;
    }
    else
    {
        xTargetVal = xActualVal - targexPos;
    }
    // Write XTarget Val
    _spiWriteReg(TMC5240_XTARGET, xTargetVal);

    return;
}

/// @brief moves motor in velocity mode
/// @param direction direction the motor has to turn
/// @param vmax maximum velocity TODO: unit???
/// @param amax maximum acceleration TODO: unit???
void Tmc5240::moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax)
{
    // flip direction if std direction ain't matching
    if (!_stdDir)
        direction = !direction;

    _spiWriteReg(TMC5240_RAMPMODE, direction ? TMC5240_MODE_VELPOS : TMC5240_MODE_VELNEG);

    _spiWriteReg(TMC5240_VMAX, vmax);
    _spiWriteReg(TMC5240_AMAX, amax);

    uint32_t vmax_val = _spiReadReg(TMC5240_VMAX);
    uint32_t amax_val = _spiReadReg(TMC5240_AMAX);
    printf("vmax set to: %d ; amax set to: %d \n", vmax_val, amax_val);

    return;
}
/* ==================================
          getters & setters
   ================================== */

/// @brief performs a dummy write cycle in order to get the devices current spi-statusflags
/// @return statusflags
uint8_t Tmc5240::getCurrentStatusFlag()
{
    // trigger dummy Register Read
    getGSTAT();

    // get status flags
    return _spiStatus;
}

/// @brief sets motor direction
/// @param direction true/false
void Tmc5240::setShaftDirection(bool direction)
{
    uint32_t GCONF_val = _spiReadReg(TMC5240_GCONF);
    GCONF_val |= direction << TMC5240_SHAFT_SHIFT;
    _spiWriteReg(TMC5240_GCONF, GCONF_val);

    return;
}

/// @brief reads out xActual register
/// @return actual steps
int32_t Tmc5240::getXActual()
{
    int32_t retVal = _spiReadReg(TMC5240_XACTUAL);
    return retVal;
}

int32_t Tmc5240::getVmax()
{
    return _spiReadReg(TMC5240_VMAX);
}

/// @brief disables Driver
/// @param val false for disable! true to set Toff out of config header
void Tmc5240::toggleToff(bool val)
{
    uint32_t chopConfValue = _spiReadReg(TMC5240_CHOPCONF);

    // clear old Toff
    chopConfValue &= ~TMC5240_TOFF_MASK;
    if (val)
        chopConfValue |= TOFF;
    // no else - otherwise Toff = 0 -> driver disable!

    _spiWriteReg(TMC5240_CHOPCONF, chopConfValue);
}

uint32_t Tmc5240::getGSTAT()
{
    uint32_t gstatVal = _spiReadReg(TMC5240_GSTAT);
    return gstatVal;
}

void Tmc5240::clearGSTAT()
{
    _spiWriteReg(TMC5240_GSTAT, 0x1F);
}

/* ==================================
         some static helpers
   ================================== */

/// @brief convert m/s or m/s^2 to ustep/s or ustep/s^2
/// @param mps meter mer second value
/// @return microsteps
int32_t Tmc5240::convertDistanceToMicrosteps(float meters)
{
    float res = (meters / WHEELCIRCUMFENCE) * MICROSTEPSPERREVOLUTION;

    int32_t retVal = static_cast<int32_t>(std::round(res));
    return retVal;
}

/// @brief convert ustep/s in cm
/// @param uSteps microsteps
/// @return Distance in Centimeter
int32_t Tmc5240::convertMicrostepsToCentimeter(uint32_t uSteps)
{
    float res = (uSteps * MICROSTEPSPERREVOLUTION) * WHEELCIRCUMFENCE;

    int32_t retVal = static_cast<int32_t>(std::round(res));
    return retVal;
}

/// @brief convert degree to microsteps
/// @param degrees float degrees
/// @return value of Microsteps
int32_t Tmc5240::convertDegreeToMicrosteps(int32_t degrees)
{
    float usps = (degrees / 360.0f) * MICROSTEPS_PER_REVOLUTION;
    return static_cast<int32_t>(round(usps));
}

/// @brief calcs rotatinal angle given by driven distances of 2 wheels on one axis
/// @param uStepsDifference step difference in Microsteps!
/// @return angle
float Tmc5240::convertDeltaDrivenDistanceToDegree(int32_t uStepsDifference)
{
    float dsMeters = convertMicrostepsToCentimeter(uStepsDifference) * 1e2;

    float res = (dsMeters * 180.0f) / (WHEELCIRCUMFENCE);

    return res;
}
