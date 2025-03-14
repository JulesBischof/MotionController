#include "SpiBase.hpp"

#include "pico/stdlib.h"
#include <stdio.h>

// init static class members
SemaphoreHandle_t SpiBase::_spiMutex = NULL;
bool SpiBase::_spiMutexInititalized = false;

/// @brief spi baseclass constructor
/// @param spiInstance spi instance - refer rp2040 datasheet
/// @param csPin chip-select pin of spi device
SpiBase::SpiBase(spi_inst_t *spiInstance, uint8_t csPin)
    : _spiInstance(spiInstance), _csPin(csPin)
{
    taskENTER_CRITICAL();
    if (!this->_spiMutexInititalized)
    {
        _spiMutex = xSemaphoreCreateMutex();
        _spiMutexInititalized = true;
    }
    _initCsGpio(csPin);
    taskEXIT_CRITICAL();
}

/// @brief deconstructor - not implemented yet
SpiBase::~SpiBase()
{
    // no deconstructor
}

/// @brief initializes spi-channel
/// @param sdiPin Serial data in / MISO
/// @param sdoPin Serial data out / MOSI
/// @param sclkPin Serial Clock 
/// @param baudrateKhz Baudrate in kHz
/// @param spiInstance spi instance - ref rpi-pico c/c++ sdk (spi0/spi1)
void SpiBase::spiInit(uint8_t sdiPin, uint8_t sdoPin, uint8_t sclkPin, uint16_t baudrateKhz, spi_inst_t *spiInstance)
{
    spi_init(spiInstance, 1000 * baudrateKhz);
    
    gpio_set_function(sdiPin, GPIO_FUNC_SPI);
    gpio_set_function(sdoPin, GPIO_FUNC_SPI);
    gpio_set_function(sclkPin, GPIO_FUNC_SPI);
}

void SpiBase::_initCsGpio(uint8_t csPin)
{
    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, 1);
}

/// @brief basic read method
/// @param reg register to read
/// @return register value 32 bit
uint32_t SpiBase::_spiReadReg(uint8_t reg)
{
    uint8_t tx_buffer[5] = {reg & 0x7F, 0, 0, 0, 0}; // Read command - MSB = 0
    uint8_t rx_buffer[5] = {0};

    xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(100));

    // push address - take a look into datasheet, pipeline structure
    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    xSemaphoreGive(_spiMutex);

    // wait some time 
    vTaskDelay(pdMS_TO_TICKS(2));

    xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(100));

    // get actual data - take a look into datasheet, pipeline structure
    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    xSemaphoreGive(_spiMutex);

    this->_spiStatus = (rx_buffer[0]);

    // reconstruct register value
    uint32_t result = (rx_buffer[1] << 24) |
                      (rx_buffer[2] << 16) |
                      (rx_buffer[3] << 8) |
                      (rx_buffer[4]);
    return result;
}

/// @brief reads of a specific bit field, if mask and shift ist provided
/// @param reg register address
/// @param mask register mask fpr specific bit field
/// @param shift necessary bitshift - refer datasheet
/// @return 
uint32_t SpiBase::_spiReadBitField(uint8_t reg, uint32_t mask, uint8_t shift)
{
    // get Register
    uint32_t reg_value = _spiReadReg(reg);
    // mask Bitfield
    uint32_t bitfield = (reg_value & mask) >> shift;
    return bitfield;
}

/// @brief writes to a specific register
/// @param reg register address
/// @param data data to send
/// @return true if transmition complete
bool SpiBase::_spiWriteReg(uint8_t reg, uint32_t data)
{
    uint8_t tx_buffer[5] = {(reg | 0x80) & 0xFF, // write - command: MSB = 1
                            (data >> 24) & 0xFF,
                            (data >> 16) & 0xFF,
                            (data >> 8) & 0xFF,
                            data & 0xFF};
    uint8_t rx_buffer[5] = {0};

    xSemaphoreTake(_spiMutex, pdMS_TO_TICKS(100));

    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    xSemaphoreGive(_spiMutex);

    // get status flags
    this->_spiStatus = (rx_buffer[0]);

    return true;
}
