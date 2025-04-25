#include "Icm42670p.hpp"
#include "Icm42670pConfig.hpp"

#include "LoggerService.hpp"

namespace i2cDevices
{
    /* ==================================
          constructor / deconstructor
       ================================== */

    /// @brief creates instance of ICM42670p Gyroscope
    /// @param i2cInstance i2cInstance ref c/c++ sdk raspberry pico
    /// @param i2cAddress i2c address
    Icm42670p::Icm42670p(i2c_inst_t *i2cInstance, uint8_t i2cAddress) : I2cBase(i2cInstance, i2cAddress)
    {
        _initDevice();
        _checkDevice();
    }

    /// @brief default constructor
    Icm42670p::Icm42670p() {}

    /// @brief deconstructor not implemented yet
    Icm42670p::~Icm42670p()
    {
    }

    /* ==================================
              init Members
       ================================== */

    /// @brief basic settings for Device
    void Icm42670p::_initDevice()
    {
        uint8_t address = 0;

        address = (uint8_t)(PWR_MGMT0 & 0xFF);
        services::LoggerService::debug("Icm42670p::_initDevice()", "PWR_MGMT0 register (addr %Xh) value: (%Xh)", address, PWR_MGMT0_INITVALUE);

        if (!i2cWriteReg(address, PWR_MGMT0_INITVALUE))
        {
            services::LoggerService::fatal("Icm42670p::_initDevice()", "failed to Write PWR_MGMT0");
        }

        address = (uint8_t)(GYRO_CONFIG0 & 0xFF);
        services::LoggerService::debug("Icm42670p::_initDevice()", "GYRO_CONFIG0 register (addr %Xh) value: (%Xh)\n", address, CONFIG0_INITVALUE);

        if (!i2cWriteReg(address, CONFIG0_INITVALUE))
        {
            services::LoggerService::fatal("Icm42670p::_initDevice()", "failed to Write GYRO_CONFIG0");
        }

        address = (uint8_t)(GYRO_CONFIG1 & 0xFF);
        services::LoggerService::debug("Icm42670p::_initDevice()", "GYRO_CONFIG1 register (addr %Xh) value: (%Xh)\n", address, CONFIG1_INITVALUE);

        if (!i2cWriteReg(address, CONFIG1_INITVALUE))
        {
            services::LoggerService::fatal("Icm42670p::_initDevice()", "failed to Write GYRO_CONFIG1");
        }
        services::LoggerService::info("Icm42670p::_initDevice()", "init done!");
    }

    /// @brief sends example message to device
    void Icm42670p::_checkDevice()
    {
        services::LoggerService::debug("Icm42670p::_checkDevice()", "check initialisation values");

        uint8_t address = 0;
        uint8_t buffer[3] = {0};

        address = (uint8_t)(PWR_MGMT0);
        i2cReadReg(address, &buffer[0], 1);

        address = (uint8_t)(GYRO_CONFIG0);
        i2cReadReg(address, &buffer[1], 1);

        address = (uint8_t)(GYRO_CONFIG1);
        i2cReadReg(address, &buffer[2], 1);

        services::LoggerService::debug("Icm42670p::_checkDevice()", "Initialization Register values: \n - PWR_MGMT0: %Xh \n - GYRO_CONFIG0: %Xh \n - GYRO_CONFIG1: %Xh \n", buffer[0], buffer[1], buffer[2]);
        return;
    }

    /* ==================================
             some helpers
       ================================== */

    /// @brief returns current angular change in °/s
    /// @return signed int - angular change in °/s
    float Icm42670p::ConvertLsbToDps(int16_t rawVal)
    {
        float val = (float)rawVal / LSB_TO_DPS_SCALE_FACTOR;
        return val;
    }

    /* ==================================
              getters & setters
       ================================== */

    /// @brief calculates absolut angle in degree, given by how many steps both drivers did
    /// @param xActualDriver0 xActual Value driver 0
    /// @param xActualDriver1 xActual Value driver 1
    /// @return angle in °
    float Icm42670p::getAngle(int32_t xActualDriver0, int32_t xActualDriver1)
    {
        float retval = (xActualDriver0 / xActualDriver1) * (180 / 3.14);
        return 0.0f;
    }

    /// @brief reads raw value of z-Axis Gyro
    /// @return Z-Gyro-Data 16bit unsigned!
    int16_t Icm42670p::getRawDataZaxis()
    {
        uint8_t rawVal[2] = {0};
        uint8_t address_0 = (uint8_t)(GYRO_DATA_Z0 & 0xFF);
        uint8_t address_1 = (uint8_t)(GYRO_DATA_Z1 & 0xFF);

        i2cReadReg(address_0, &rawVal[0], 1);
        i2cReadReg(address_1, &rawVal[1], 1);

        int16_t retVal = 0;

        if (this->_i2cStatus == I2C_STATUSOK)
            retVal = (int16_t)(rawVal[1] << 8 | rawVal[0]);

        return retVal;
    }

    float Icm42670p::getTemperature()
    {
        uint8_t temp_data[2] = {0};
        uint8_t address_0 = (uint8_t)(TEMP_DATA0 & 0xFF);
        uint8_t address_1 = (uint8_t)(TEMP_DATA1 & 0xFF);

        i2cReadReg(address_1, &temp_data[1], 1);
        i2cReadReg(address_0, &temp_data[0], 1);

        int16_t rawVal = (int16_t)(temp_data[1] << 8 | temp_data[0]);
        float temp_celsius = 25;

        if (this->_i2cStatus == I2C_STATUSOK)
            temp_celsius = (rawVal / 132.48f) + 25.0f; // unit conversion ref datasheet

        return temp_celsius;
    }

}