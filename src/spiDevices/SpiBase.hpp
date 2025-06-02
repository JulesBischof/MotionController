#ifndef SPIBASE_H
#define SPIBASE_H

#include "hardware/spi.h"
#include "FreeRTOS.h"
#include "semphr.h"

// I2C ERRORCODES
#define STATUSOK 0;
#define TIMEOUT 1;

namespace spiDevices
{
    /// @brief BaseClass for SPI-Devices. Reentrent
    class SpiBase
    {
    private:

    protected:
        spi_inst_t *_spiInstance;
        uint8_t _csPin;
        uint8_t _spiStatus;

        virtual void _initDevice() = 0;
        virtual void _checkDevice() = 0;

        /// @brief inits the CS-Pin
        /// @param csPin chip select pin
        void _initCsGpio(uint8_t csPin);

        /// @brief basic read method
        /// @param reg register to read
        /// @return register value 32 bit
        uint32_t _spiReadReg(uint8_t reg);

        /// @brief reads of a specific bit field, if mask and shift ist provided
        /// @param reg register address
        /// @param mask register mask fpr specific bit field
        /// @param shift necessary bitshift - refer datasheet
        /// @return Register BitField Value
        uint32_t _spiReadBitField(uint8_t reg, uint32_t mask, uint8_t shift);

        /// @brief writes to a specific register
        /// @param reg register address
        /// @param data data to send
        /// @return true if transmition complete
        bool _spiWriteReg(uint8_t reg, uint32_t data);

    public:
        /// @brief basic ctor
        SpiBase();

        /// @brief spi baseclass constructor
        /// @param spiInstance spi instance - refer rp2040 datasheet
        /// @param csPin chip-select pin of spi device
        SpiBase(spi_inst_t *spiInstance, uint8_t csPin);
        
        /// @brief dctor not implemented yet
        ~SpiBase();

        /// @brief initializes spi-channel
        /// @param sdiPin serial data in / MISO
        /// @param sdoPin serial data out / MOSI
        /// @param sclkPin serial Clock
        /// @param baudrateKhz Baudrate in kHz
        /// @param spiInstance spi instance - ref rpi-pico c/c++ sdk (spi0/spi1)
        static void spiInit(uint8_t sdiPin, uint8_t sdoPin, uint8_t sclkPin, uint16_t baudrateKhz, spi_inst_t *spiInstance);

        /// @brief getter for spiBase status value
        /// @return 8 status Bits
        uint8_t getStatus() { return this->_spiStatus; };
    };

}
#endif
