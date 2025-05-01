#ifndef TMC5240_H
#define TMC5240_H

#include "SpiBase.hpp"

namespace spiDevices
{

    class Tmc5240 : public SpiBase
    {
    private:
        bool _stdDir;
        bool _velocityMode;

        /// @brief inits Current settings out of StepperConfig.h File
        void _initCurrentSetting();

        /// @brief initializes stepper drive in SpreadCycle Mode - ref Datasheet
        void _initSpreadCycle();

        /// @brief writes IRIN value to IHOLD_RUN Bitfield
        /// @param iRun Value from 0 ... 31. 31 = Fullscale current
        void _setIRun(uint32_t ihold);

    protected:
        void _initDevice() override;

        /// @brief checks on Register Configuration Values for Debug Reasons
        void _checkDevice() override;

    public:
        /// @brief default ctor
        Tmc5240();

        /// @brief creates instance of Trinamics TMC5240 stepper driver
        /// @param spiInstance spi instance - refer pico c/c++ sdk
        /// @param csPin chip select pin
        Tmc5240(spi_inst_t *spiInstance, uint8_t csPin, bool stdDir);
        ~Tmc5240();

        /// @brief returns statusflags out of the last transmission
        /// @return 8bits statusflags - refDatasheet
        uint8_t getLastStatusFlag() { return this->_spiStatus; };

        /// @brief performs a dummy write cycle in order to get the devices current spi-statusflags
        /// @return statusflags
        uint8_t getCurrentStatusFlag();

        /// @brief toggles Shaft Direction in Config Register
        /// @param direction 1 or 0
        void setShaftDirection(bool direction);

        /// @brief moves motor in velocity mode
        /// @param direction direction the motor has to turn
        /// @param vmax maximum velocity TODO: unit???
        /// @param amax maximum acceleration TODO: unit???
        void moveVelocityMode(bool direction, uint32_t vmax, uint32_t amax);

        /// @brief moves Motor to specific location using position mode
        /// @param xTargetVal ABSOLUTE position! in usteps
        /// @param vmax maximum velocity
        /// @param amax maxmimum acceleration
        void moveAbsolutePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax);

        /// @brief moves Motor to specific location using position mode
        /// @param xTargetVal RELATIVE position! in steps
        /// @param vmax maximum velocity
        /// @param amax maxmimum acceleration
        void moveRelativePositionMode(int32_t targexPos, uint32_t vmax, uint32_t amax, bool dir);

        /// @brief reads out xActual register
        /// @return actual usteps sind device Startup
        int32_t getXActual();

        int32_t getXTarget();

        int32_t getVActual();

        uint32_t getGSTAT();
        void clearGSTAT();

        int32_t getVmax();

        /// @brief sets IRUN based on a percentage value
        /// @param percent percantage value from 0 ... 100
        void setRunCurrent(uint8_t percentage);

        /// @brief disables Driver
        /// @param val false for disable! true to set Toff out of config header
        void toggleToff(bool val);

        /// @brief checks StatusFlgs if standstill status flags are set
        /// @return true if driver is in standstill
        bool checkForStandstill();
    };

}
#endif // TMC5240_H