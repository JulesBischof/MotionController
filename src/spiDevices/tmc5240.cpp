#include "tmc5240.hpp"

Tmc5240::Tmc5240(spi_inst_t *spiInstance, uint8_t csPin) : SpiBase(spiInstance, csPin)
{
}

Tmc5240::~Tmc5240()
{
    // not implemented yet
}

void Tmc5240::_initDevice()
{
    _initCurrentSetting();
    _initSpreadCycle();
}

void Tmc5240::_checkDevice()
{
    uint8_t version = _spiReadBitField(TMC5240_INP_OUT, TMC5240_VERSION_MASK, TMC5240_VERSION_SHIFT);
    printf("TMC5240 Version %d initialized! \n", version);

    bool drvEnn = _spiReadBitField(TMC5240_INP_OUT, TMC5240_DRV_ENN_MASK, TMC5240_DRV_ENN_SHIFT);
    printf("DRV_ENN State = %d \n", drvEnn);
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

    printf("Current Settings Register Values \n --- GLOBSCALER ... 0x%x \n --- DRV_CONF ... 0x%x \n --- IHOLD_IRUN ... %x \n", globscalerval, drv_conf_val, ihold_irun_val);
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

void Tmc5240::setShaftDirection(bool direction)
{
    uint32_t GCONF_val = _spiReadReg(TMC5240_GCONF);
    GCONF_val |= direction << TMC5240_SHAFT_SHIFT;
    _spiWriteReg(TMC5240_GCONF, GCONF_val);

    return;
}

void Tmc5240::moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax)
{
    _spiWriteReg(TMC5240_RAMPMODE, direction ? TMC5240_MODE_VELPOS : TMC5240_MODE_VELNEG);

    _spiWriteReg(TMC5240_VMAX, vmax);
    _spiWriteReg(TMC5240_AMAX, amax);

    uint32_t vmax_val = _spiReadReg(TMC5240_VMAX);
    uint32_t amax_val = _spiReadReg(TMC5240_AMAX);
    printf("vmax set to: %d ; amax set to: %d \n", vmax_val, amax_val);

    return;
}