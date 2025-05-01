#include "Tmc5240.hpp"

#include "TMC5240_HW_Abstraction.h"
#include "Tmc5240Config.hpp"

#include "pico/stdlib.h"
#include "LoggerService.hpp"

namespace spiDevices
{
    /* ==================================
           Constructor / Deconstructor
        ================================== */

    Tmc5240::Tmc5240(spi_inst_t *spiInstance, uint8_t csPin, bool stdDir) : SpiBase(spiInstance, csPin)
    {
        _stdDir = stdDir;

        services::LoggerService::debug("Tmc5240::ctor", "Startup Driver using CS_GPIO #%d ... \n", csPin);

        _initDevice();
        _checkDevice();

        this->moveVelocityMode(0, 0, 5000); // initially Stop running Motors

        _velocityMode = false;
    }

    Tmc5240::~Tmc5240()
    {
        // deconstructor not impemented yet
    }

    Tmc5240::Tmc5240() {}

    /* ==================================
                Init members
       ================================== */

    void Tmc5240::_initDevice()
    {
        services::LoggerService::debug("Tmc5240::_initDevice", "resetting Device");
        clearGSTAT();

        services::LoggerService::debug("Tmc5240::_initDevice", "init current settings");
        _initCurrentSetting();

        services::LoggerService::debug("Tmc5240::_initDevice", "init SpredCycle");
        _initSpreadCycle();

        services::LoggerService::debug("Tmc5240::_initDevice", "done");
    }

    void Tmc5240::_checkDevice()
    {
        uint8_t version = _spiReadBitField(TMC5240_INP_OUT, TMC5240_VERSION_MASK, TMC5240_VERSION_SHIFT);
        services::LoggerService::debug("Tmc5240::_checkDevice", "Version %d", version);

        bool drvEnn = _spiReadBitField(TMC5240_INP_OUT, TMC5240_DRV_ENN_MASK, TMC5240_DRV_ENN_SHIFT);
        services::LoggerService::debug("Tmc5240::_checkDevice", "DRV_ENN %d", drvEnn);

        uint32_t gStat = getGSTAT();
        services::LoggerService::debug("Tmc5240::_checkDevice", "GSTAT %d", gStat);

        services::LoggerService::debug("Tmc5240::_checkDevice", "check done");
    }

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

        services::LoggerService::debug("Tmc5240::_initCurrentSetting",
                                       "Current Settings Register Values \n --- GLOBSCALER ... 0x%x \n --- DRV_CONF ... 0x%x \n --- IHOLD_IRUN ... %x \n",
                                       globscalerval, drv_conf_val, ihold_irun_val);
    }

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

    void Tmc5240::_setIRun(uint32_t iRun)
    {
        uint32_t IHOLD_IRUN_val = _spiReadReg(TMC5240_IHOLD_IRUN);
        IHOLD_IRUN_val &= ~TMC5240_IHOLD_MASK;
        IHOLD_IRUN_val |= iRun << TMC5240_IRUN_SHIFT;
        _spiWriteReg(TMC5240_IHOLD_IRUN, IHOLD_IRUN_val);

        return;
    }

    /* ==================================
             control targetdevice
       ================================== */

    void Tmc5240::moveAbsolutePositionMode(int32_t xTargetVal, uint32_t vmax, uint32_t amax)
    {
        _velocityMode = false;
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

    void Tmc5240::moveRelativePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax, bool dir)
    {
        _velocityMode = false;
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

    void Tmc5240::moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax)
    {
        _velocityMode = true;
        // flip direction if std direction ain't matching
        if (!_stdDir)
            direction = !direction;

        _spiWriteReg(TMC5240_RAMPMODE, direction ? TMC5240_MODE_VELPOS : TMC5240_MODE_VELNEG);

        _spiWriteReg(TMC5240_VMAX, vmax);
        _spiWriteReg(TMC5240_AMAX, amax);

        uint32_t vmax_val = _spiReadReg(TMC5240_VMAX);
        uint32_t amax_val = _spiReadReg(TMC5240_AMAX);

        return;
    }
    /* ==================================
              getters & setters
       ================================== */

    uint8_t Tmc5240::getCurrentStatusFlag()
    {
        // trigger dummy Register Read
        getGSTAT();

        // get status flags
        return _spiStatus;
    }

    void Tmc5240::setShaftDirection(bool direction)
    {
        uint32_t GCONF_val = _spiReadReg(TMC5240_GCONF);
        GCONF_val |= direction << TMC5240_SHAFT_SHIFT;
        _spiWriteReg(TMC5240_GCONF, GCONF_val);

        return;
    }

    int32_t Tmc5240::getXActual()
    {
        int32_t retVal = _spiReadReg(TMC5240_XACTUAL);
        return retVal;
    }

    int32_t Tmc5240::getXTarget()
    {
        int32_t retVal = _spiReadReg(TMC5240_XTARGET);
        return retVal;
    }

    int32_t Tmc5240::getVActual()
    {
        int32_t retVal = _spiReadReg(TMC5240_VACTUAL);
        return retVal;
    }

    int32_t Tmc5240::getVmax()
    {
        return _spiReadReg(TMC5240_VMAX);
    }

    void Tmc5240::setRunCurrent(uint8_t percent)
    {
        if (percent > 100)
        {
            percent = 100;
        }

        uint8_t value = (percent * 31 + 50) / 100; // round to nearest integer
        if (value > 31)
        {
            value = 31;
        }
        _setIRun(value);
    }

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

    bool Tmc5240::checkForStandstill()
    {
        bool standstillFlag = false;

        if (_velocityMode)
        {
            int32_t vActual = getVActual();
            standstillFlag = (vActual == 0);
        }
        else
        { // position Mode
            int32_t xActual = getXActual();
            int32_t xTarget = getXTarget();

            standstillFlag = (xActual == xTarget);
        }
        return standstillFlag;
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
}