/**
 * @brief  Interface to SPI master implementations
 *
 * General usage instructions...
 *
 * 1. Call spi_init() to initialise an SPI interface
 * 2. Call spi_add_device() for each device on that interface
 * 3. spi_transfer() can be used for up to 64K bit transfers
 * 4. spi_transfer_data() may or may not be implemented by SPI
 *    hardware (the bit-bashed driver does provide it).
 *    It doesn't drive nCS and can be used to string together
 *    multiple transfers or work with devices that need special
 *    handling of the chip-select process.
 * 5. Drivers for specific devices can be found in drivers/spi.
 *    Drivers typically call spi_add_device() for you.
 * 6. The bit-bashed driver supports transfers which are not a
 *    round number of bytes e.g. 11 bits is ok.  The hardware
 *    drivers typically only support multiple byte transfers
 *    but most standard parts only work in bytes.
 *
 * Copyright Debug Innovations Ltd. 2016-2022
 */

#ifndef HAL_SPI_H
#define HAL_SPI_H

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// SPI pins comprising an SPI interface
struct spi_pins
{
    uint16_t  sclk_gpio;
    uint16_t  mosi_gpio;
    uint16_t  miso_gpio;
    uint8_t   mode;
};
typedef struct spi_pins  spi_pins_t;


// SPI device
// This identifies a specific device on a specific SPI interface
struct spi_device
{
    spi_pins_t  *pins;        // SPI interface the device is on
    void        *dev_state;   // Device state pointer for drivers
    uint16_t    ncs_gpio;     // nCS line for this device
};
typedef struct spi_device  spi_device_t;


/*
 * Initialise SPI interface, call this first
 *
 * In the bit-bashed SPI driver and some hardware SPI drivers, the pins to use are
 * movable and chosen by their GPIO numbers.  On SPI hardware with fixed pins, the
 * sclk_gpio parameter defines the SPI peripheral block number starting at zero.
 *
 * @param[out]  pins         Interface control structure
 * @param[in]   sclk_gpio    GPIO pin to use for SCLK, see above
 * @param[in]   mosi_gpio    GPIO pin to use for MOSI
 * @param[in]   miso_gpio    GPIO pin to use for MISO
 * @param[in]   sclk_Hz      Required SCLK clock frequency (ignored by bit-bashed SPI)
 * @param[in]   mode         SPI mode to use, Bit 1:CPOL, Bit 0:CPHA
 */
void spi_init(spi_pins_t *pins, uint16_t sclk_gpio, uint16_t mosi_gpio, uint16_t miso_gpio, uint32_t sclk_Hz, uint8_t mode);


/*
 * Adds a device to an SPI interface by building a device structure
 *
 * @param[out]  device       Device to read/write.  This struct is populated by
 *                           the following parameters and should be used in calls
 *                           to the other functions.
 * @param[in]   pins         SPI interface the device is connected to
 * @param[in]   ncs_gpio     GPIO pin to use for this device's nCS
 */
void spi_add_device(spi_device_t *device, spi_pins_t *pins, uint16_t ncs_gpio);


/*
 * @brief Transfer data to/from an SPI device (for non-standard devices)
 *        This function does not drive nCS. The device must be selected by the caller.
 *        This function can be called multiple times to string together transfers without
 *        deselecting nCS to support >64K bit transfers or control the timing of nCS
 *        for devices that go to sleep or have to be polled to check they are ready
 *        before starting the transfer.  For 'normal' devices, use spi_transfer().
 *        Data is transferred in the order MSB->LSB of byte 0, then MSB->LSB of byte 1 etc.
 * @param[in]   device       Device to read/write
 * @param[in]   txdata       Data to send to the slave (see above for bit order)
 * @param[out]  rxdata       Data received from the slave (see above for bit order), NULL to discard
 * @param[in]   num_bits     Number of bits transferred (clocks), max. 65535 bits (8191 bytes)
 */
void spi_transfer_data(spi_device_t *device, uint8_t *txdata, uint8_t *rxdata, uint16_t num_bits);


/*
 * @brief Transfer data to/from an SPI device (for standard devices)
 *        This function selects the device, transfers the data then deselects the device.
 *        Data is transferred in the order MSB->LSB of byte 0, then MSB->LSB of byte 1 etc.
 * @param[in]   device       Device to read/write
 * @param[in]   txdata       Data to send to the slave (see above for bit order)
 * @param[out]  rxdata       Data received from the slave (see above for bit order), NULL to discard
 * @param[in]   num_bits     Number of bits transferred (clocks), max. 65535 bits (8191 bytes)
 */
void spi_transfer(spi_device_t *device, uint8_t *txdata, uint8_t *rxdata, uint16_t num_bits);


#ifdef __cplusplus
}
#endif

#endif

