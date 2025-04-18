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

    class SpiBase
    {
    private:
        static SemaphoreHandle_t _spiMutex;
        static bool _spiMutexInititalized;

    protected:
        spi_inst_t *_spiInstance;
        uint8_t _csPin;
        uint8_t _spiStatus;

        virtual void _initDevice() = 0;
        virtual void _checkDevice() = 0;
        void _initCsGpio(uint8_t csPin);

        uint32_t _spiReadReg(uint8_t reg);
        uint32_t _spiReadBitField(uint8_t reg, uint32_t mask, uint8_t shift);
        bool _spiWriteReg(uint8_t reg, uint32_t data);

    public:
        SpiBase();
        SpiBase(spi_inst_t *spiInstance, uint8_t csPin);
        ~SpiBase();

        static void spiInit(uint8_t sdiPin, uint8_t sdoPin, uint8_t sclkPin, uint16_t baudrateKhz, spi_inst_t *spiInstance);

        uint8_t getStatus() { return this->_spiStatus; };
    };

}
#endif
