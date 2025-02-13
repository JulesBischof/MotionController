#include "spiBase.hpp"

/// @brief spi baseclass constructor
/// @param spiInstance spi instance - refer rp2040 datasheet
/// @param csPin chip-select pin of spi device
spiBase::spiBase(spi_inst_t *spiInstance, uint8_t csPin)
{
    this->_csPin = csPin;
    this->_spiInstance = spiInstance;

    this->_initDevice();
    this->_checkDevice();
}

/// @brief deconstructor - not implemented yet
spiBase::~spiBase()
{
    // no deconstructor
}

/// @brief basic read method
/// @param reg register to read
/// @return register value 32 bit
uint32_t spiBase::spiReadReg(uint8_t reg)
{
    uint8_t tx_buffer[5] = {reg & 0x7F, 0, 0, 0, 0}; // Read command - MSB = 0
    uint8_t rx_buffer[5] = {0};

    // disables all Interrupts and Scheduler activity from FreeRTOS
    taskENTER_CRITICAL();

    // push address - take a look into datasheet, pipeline structure
    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    // returns to TimeSlicing Behaviour
    taskEXIT_CRITICAL();

    // wait some time
    vTaskDelay(pdMS_TO_TICKS(10));

    // disables all Interrupts and Scheduler activity from FreeRTOS
    taskENTER_CRITICAL();

    // get actual data - take a look into datasheet, pipeline structure
    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    // returns to TimeSlicing Behaviour
    taskEXIT_CRITICAL();

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
uint32_t spiBase::spiReadBitField(uint8_t reg, uint32_t mask, uint8_t shift)
{
    // get Register
    uint32_t reg_value = spiReadReg(reg);
    // mask Bitfield
    uint32_t bitfield = (reg_value & mask) >> shift;
    return bitfield;
}

/// @brief writes to a specific register
/// @param reg register address
/// @param data data to send
/// @return true if transmition complete
bool spiBase::spiWriteReg(uint8_t reg, uint32_t data)
{
    uint8_t tx_buffer[5] = {(reg | 0x80) & 0xFF, // write - command: MSB = 1
                            (data >> 24) & 0xFF,
                            (data >> 16) & 0xFF,
                            (data >> 8) & 0xFF,
                            data & 0xFF};
    uint8_t rx_buffer[5] = {0};

    // disables all Interrupts and Scheduler activity from FreeRTOS
    taskENTER_CRITICAL();

    gpio_put(_csPin, 0); // pull down CS
    sleep_us(1);
    spi_write_read_blocking(_spiInstance, tx_buffer, rx_buffer, 5);
    gpio_put(_csPin, 1); // pull up CS

    // returns to TimeSlicing Behaviour
    taskEXIT_CRITICAL();

    // get status flags
    this->_spiStatus = (rx_buffer[0]);

    return true;
}
