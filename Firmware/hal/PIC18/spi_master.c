/**
 * @brief  SPI master using the new PIC18 SPI peripheral
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83
 *          26K83
 *
 * There are hardware restrictions on which pins can be used for the SPI
 * signals (see datasheet).  If you want to run SPI on pins that are not
 * usable by the hardware SPI, you'll have to use the bit-bashed SPI driver.
 *
 * However, the chip select pins are driven by software so there is no limit
 * to the number of devices on a channel or which pins can be used for the
 * chip selects.
 *
 * You can transfer up to 16376 bits (2047 bytes) in one transfer but this
 * driver only supports round numbers of bytes - 8, 16, 256 bits are all ok
 * but if you need to transfer 11 bits, you have to use the bit-bashed driver.
 *
 * Copyright Debug Innovations Ltd. 2022
 */

// TODO: Performance is poor - gap between TX bytes ~4us, should be able to pre-load the FIFO
// TODO: Support 64K transfers as per the hal_spi.h header, run the memtest (4K transfers)

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "hal_chip.h"
#include "hal_gpio.h"
#include "hal_spi.h"


// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];


#if defined(_18F25K83) || defined(_18F26K83)

#define PPS_OUT_SCLK_SETTING  0x1E
#define PPS_OUT_MOSI_SETTING  0x1F
#define PPS_OUT_CS_SETTING    0x20   // Not used, here for completeness

#elif defined(_18F26Q43) || defined(_18F27Q43)

#error "Not supported"

#endif


/*
 * Initialise SPI interface, call this first
 *
 * On this chip, the pins to use are movable and chosen by their GPIO numbers
 * but there are restrictions on which pins can be chosen (see datasheet)
 *
 * @param[out]  pins         Interface control structure
 * @param[in]   sclk_gpio    GPIO pin to use for SCLK, see above
 * @param[in]   mosi_gpio    GPIO pin to use for MOSI
 * @param[in]   miso_gpio    GPIO pin to use for MISO
 * @param[in]   sclk_Hz      Required SCLK clock frequency (ignored by bit-bashed SPI)
 * @param[in]   mode         SPI mode to use, Bit 1:CPOL, Bit 0:CPHA
 */
void spi_init(spi_pins_t *pins, uint16_t sclk_gpio, uint16_t mosi_gpio, uint16_t miso_gpio, uint32_t sclk_Hz, uint8_t mode)
{
    uint32_t  cpu_clk_Hz;
    uint16_t  port;
    uint8_t   pin, clk_div;

    if ((pins == NULL) || (sclk_Hz == 0))
        return;

    // Populate the control structure
    // The GPIO numbers are not used in transfer calls but we set up
    // the whole structure to aid debugging
    pins->sclk_gpio = sclk_gpio;
    pins->mosi_gpio = mosi_gpio;
    pins->miso_gpio = miso_gpio;
    pins->mode = mode;

    // Turn on the SPI peripheral block clocks
    PMD5bits.SPI1MD = 0;

    // Disable SPI to set it up
    SPI1CON0 = 0;

    // Configure the pins
    if (sclk_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure SCLK initially as a low GPIO output
        gpio_configure_pin(sclk_gpio, PIN_DIGITAL_OUT | PIN_SET_LOW);

        port = sclk_gpio & 0x0F00;
        pin = sclk_gpio & 7;
        if (port == 0x0100)
        {
            // Port B PPS
            *pps_outb_regs[pin] = PPS_OUT_SCLK_SETTING;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            *pps_outc_regs[pin] = PPS_OUT_SCLK_SETTING;
        }
    }
    if (mosi_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure MOSI initially as a low GPIO output
        gpio_configure_pin(mosi_gpio, PIN_DIGITAL_OUT | PIN_SET_LOW);

        port = mosi_gpio & 0x0F00;
        pin = mosi_gpio & 7;
        if (port == 0x0100)
        {
            // Port B PPS
            *pps_outb_regs[pin] = PPS_OUT_MOSI_SETTING;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            *pps_outc_regs[pin] = PPS_OUT_MOSI_SETTING;
        }
    }
    if (miso_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure MISO initially as a GPIO input
        gpio_configure_pin(miso_gpio, PIN_DIGITAL_IN | PIN_PULL_UP);

        port = miso_gpio & 0x0F00;
        pin = miso_gpio & 7;
        if (port == 0x0100)
        {
            // Port B PPS
            SPI1SDIPPS = 0x08 | pin;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            SPI1SDIPPS = 0x10 | pin;
        }
    }

    // Calculate the clock divider
    cpu_clk_Hz = chip_get_cpu_clock();
    clk_div = (uint8_t)((cpu_clk_Hz >> 1) / sclk_Hz);
    if (clk_div != 0)
        clk_div--;

    // Use master mode, 8 bits, MSB first with the required clock rate
    SPI1TWIDTH = 0;
    SPI1CON0 = 3;

    // Set the mode
    mode = (uint8_t)(((mode & 2) << 4) | (((mode ^ 1) & 1) << 6));
    SPI1CON1 = 0x84 | mode;

    SPI1CON2 = 3;
    SPI1CLK = 0;
    SPI1BAUD = clk_div;

    // Enable
    SPI1CON0 = 0x83;
}


/*
 * Adds a device to an SPI interface by building a device structure
 *
 * @param[out]  device       Device to read/write.  This struct is populated by
 *                           the following parameters and should be used in calls
 *                           to the other functions.
 * @param[in]   pins         SPI interface the device is connected to
 * @param[in]   ncs_gpio     GPIO pin to use for this device's nCS
 */
void spi_add_device(spi_device_t *device, spi_pins_t *pins, uint16_t ncs_gpio)
{
    // Initialise the device struct
    device->pins = pins;
    device->ncs_gpio = ncs_gpio;
    device->dev_state = NULL;

    // Configure the nCS pin with the slave deselected
    gpio_configure_pin(ncs_gpio, PIN_DIGITAL_OUT | PIN_SET_HIGH);
}


/*
 * @brief Transfer data to/from an SPI device (for non-standard devices)
 *        This function does not drive nCS. The device must be selected by the caller.
 *        This function can be called multiple times to string together transfers without
 *        deselecting nCS to support >16K bit transfers or control the timing of nCS
 *        for devices that go to sleep or have to be polled to check they are ready
 *        before starting the transfer.  For 'normal' devices, use spi_transfer().
 *        Data is transferred in the order MSB->LSB of byte 0, then MSB->LSB of byte 1 etc.
 * @param[in]   device       Device to read/write
 * @param[in]   txdata       Data to send to the slave (see above for bit order)
 * @param[out]  rxdata       Data received from the slave (see above for bit order), NULL to discard
 * @param[in]   num_bits     Number of bits transferred (clocks), max. 16376 bits (2047 bytes)
 */
void spi_transfer_data(spi_device_t *device, uint8_t *txdata, uint8_t *rxdata, uint16_t num_bits)
{
    uint16_t  tx, rx, num_bytes;

    // This driver doesn't support >16K bit transfers
    if (num_bits > 16376)
        return;

    // We only support transfers in multiples of 8 bits
    num_bytes = (uint16_t)(num_bits >> 3);

    if (num_bytes == 0)
        return;

    SPI1TCNT = num_bytes;

    // The TX only and TX/RX cases are dealt with separately so they can be optimised
    if (rxdata == NULL)
    {
        // Transmit only
        SPI1CON2 = 2;
        tx = 0;

        while (tx < num_bytes)
        {
            // If there is space in the TX buffer, fill it
            if (SPI1STATUSbits.TXBE)
                SPI1TXB = txdata[tx++];
        }

        // Wait for the transfer to complete
        // We have to do this because nCS has to remain low during the whole transfer
        while (SPI1CON2 & 0x80)
            ;
    }
    else
    {
        // Full-duplex case
        // Drain any lingering RX data
        (void)SPI1RXB;

        SPI1CON2 = 3;
        tx = 0;
        rx = 0;

        // Wait for space in the TX buffer
        while (SPI1STATUSbits.TXBE == 0)
            ;

        // Send the first byte
        SPI1TXB = txdata[tx++];

        while (rx < num_bytes)
        {
            // Wait for data in the RX buffer
            while (SPI1RXIF == 0)
                ;

            // Grab the RX data and send the next TX data.
            // Once the RX data arrives, the TX buffer must be empty but
            // it isn't safe to send more data until we read the RX data
            // in case an interrupt goes off and we can't read it in time.
            rxdata[rx++] = SPI1RXB;
            if (rx < num_bytes)
                SPI1TXB = txdata[tx++];
        }
    }
}


/*
 * @brief Transfer data to/from an SPI device (for standard devices)
 *        This function selects the device, transfers the data then deselects the device.
 *        Data is transferred in the order MSB->LSB of byte 0, then MSB->LSB of byte 1 etc.
 * @param[in]   device       Device to read/write
 * @param[in]   txdata       Data to send to the slave (see above for bit order)
 * @param[out]  rxdata       Data received from the slave (see above for bit order), NULL to discard
 * @param[in]   num_bits     Number of bits transferred (clocks), max. 16376 bits (2047 bytes)
 */
void spi_transfer(spi_device_t *device, uint8_t *txdata, uint8_t *rxdata, uint16_t num_bits)
{
    // This driver doesn't support >16K bit transfers
    if (num_bits > 16376)
        return;

    // Select slave device
    gpio_set(device->ncs_gpio, LOW);

    // Transfer the data
    spi_transfer_data(device, txdata, rxdata, num_bits);

    // Deselect slave device
    gpio_set(device->ncs_gpio, HIGH);
}

