/**
 * @brief  SPI slave using the new PIC18 SPI peripheral
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83
 *          26K83
 *
 * There are hardware restrictions on which pins can be used for the SPI
 * signals (see datasheet).
 *
 * The application interface to this driver presents as a register bank of up
 * to 128 (byte) registers accessed through a callback function which is called
 * from interrupt context and needs to be fast enough to avoid interfering with
 * reception of bytes at the clock rate of the SPI master.  The first address
 * to read/write is sent in the first byte...
 *
 * Master WRITES:                Direction
 *   1  0x80 + start reg addr    MST->SLV
 *   2  Byte 1 XOR 0x55          MST->SLV
 *   3  data[addr]               MST->SLV
 *   4  data[addr + 1]           MST->SLV
 *   5  etc...                   MST->SLV
 *
 * The master sends the start address in the first byte together with the
 * m.s. bit set to indicate that this is a write request, then the master sends
 * the first byte XOR'ed with 0x55 - if the first two bytes don't match, this
 * driver doesn't call the callback so no register write occurs.  Byte 3 onwards
 * cause a callback for each byte with the register address incremented for each
 * byte.  It is up to the callback function to decide the meaning of the data,
 * whether it can be written to the register bank and the effect of the write.
 * MISO data is random and should be ignored by the master.
 * For register writes, the master clocks n + 2 bytes to write n registers.
 *
 * Master READS:                 Direction
 *   1  0x00 + start reg addr    MST->SLV
 *   2  Byte 1 XOR 0x55          MST->SLV
 *   3  0x00 (Padding)           MST->SLV
 *   4  data[addr]               SLV->MST
 *   5  data[addr] XOR 0x55      SLV->MST
 *   6  data[addr + 1]           SLV->MST
 *   7  data[addr + 1] XOR 0x55  SLV->MST
 *   8  etc...                   SLV->MST
 *
 * The master sends the start address in the first byte with the m.s. bit clear
 * to indicate that this is a read request, then the master sends the first byte
 * XOR'ed with 0x55 - if the first two bytes don't match, this driver doesn't
 * call the callback and MISO data is random.  From the 3rd byte onwards, MOSI
 * data is ignored by this driver.  Byte 3 is a padding byte which allows enough
 * time for the slave to load the first register value for transmission to the
 * master in byte 4.  Byte 4 onwards cause a callback for each byte with the
 * register address incremented for each byte.  Each data byte is sent to the
 * master followed by the same byte XOR'ed with 0x55 so the master can check for
 * reception errors.  Because the slave has to pre-load each byte prior to the
 * master clocks arriving for that byte, the final byte loads the final + 1
 * byte data and this is transmitted at the start of the next transfer - this
 * is just an artefact of the protocol and should be ignored by the master.
 * For register reads, the master clocks 2n + 3 bytes to read n registers.
 *
 * Read-sensitive registers:
 * If the application wants any registers to be read-sensitive, they must be
 * preceded by an unused register.  This is because a read pre-loads the next
 * register by the end of the transfer so the read-sensitive register would
 * also be read.
 *
 * Timing:
 * Although the driver is interrupt driven, the maximum data rate is quite low
 * and depends on how many other interrupts are active in the application as a
 * whole.  The SPI slave hardware can be clocked at many MHz but the time it
 * takes for the interrupt to process each byte means the PIC can't keep up,
 * so a delay is required between each byte or the SPI clock needs to be slow
 * enough that it takes that amount of time to send the following byte.  There
 * also needs to be a gap between packets to allow the nSS high then low
 * transitions to be recognised in the correct order.  100us seems to be enough
 * though it will also be affected by other interrupt activity.
 *
 * Error handling:
 * This driver detects reception errors in the critical first byte by checking
 * the second byte after XOR'ing with 0x55.  The purpose is to detect single
 * bit shifts due to missing/extra clock edges.  If we simply inverted the 2nd
 * byte, patterns such as a read of register 0x33 (00110011 11001100) would
 * become (00011001 11100110) after a bit shift which are a match by inversion.
 * By inverting every other bit, the l.s. bit of byte 1 is inverted and the
 * m.s. bit of byte 2 is not, so single bit shifts are detected as errors.
 * For writes, the rest of the data is not checked so the data written may be
 * incorrect but the register written is likely to be the right one.
 * The application should consider the effect of incorrect register values
 * e.g. by reading them back or having a checksum, trying to detect illegal
 * values in the callback and considering the effect on the wider system.
 * For reads, the data returned to the master is sent twice - first the
 * register value then the same value XOR'ed with 0x55 for the same reason.
 * It is up to the application to take appropriate action on a read error.
 *
 * Copyright Debug Innovations Ltd. 2022
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "hal_gpio.h"
#include "spi_slave.h"


// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];


#if defined(_18F25K83) || defined(_18F26K83)

#define PPS_OUT_MISO_SETTING  0x1F

#elif defined(_18F26Q43) || defined(_18F27Q43)

#error "Not supported"

#endif


// These 3 variables are only used in the RX data interrupt
// and don't need to be volatile
static uint8_t             byte0, reg_addr, tx_data;

// This is used in both interrupt routines
static volatile uint8_t    byte_num = 0;

static spi_slave_reg_fn_t  reg_callback = NULL;
static bool                error, write_transfer;
static uint16_t            nss_pin = GPIO_NOT_CONNECTED;


static void reset_fifos(void)
{
    // Clear the TX/RX data FIFOs
    SPI1STATUS = 4;

    // Pre-load the transmit FIFO with two zero bytes
    SPI1TCNT = 2;
    SPI1TXB = 0;
    SPI1TXB = 0;
    SPI1STATUS = 0;
}


// SPI event handler (non-data)
static void __interrupt(irq(IRQ_SPI1)) spi_event_isr(void)
{
    // Reset the transfer state
    byte_num = 0;

    if (RXOIF || TXUIF)
    {
        // An error occurred, wait for nSS to go high
        if (nss_pin != GPIO_NOT_CONNECTED)
        {
            while (gpio_get(nss_pin) == LOW)
                SPI1TXB = 0;
        }
        reset_fifos();
        RXOIF = 0;
        TXUIF = 0;
    }
    else if (SOSIF)
    {
        // nSS has just gone low
        // To make sure we don't lose sync with the master,
        // we pre-load another two bytes of zeros into the
        // TX buffer which will fill it
        SPI1TXB = 0;
        SPI1TXB = 0;

        // Clear the interrupt flag
        SOSIF = 0;
    }
}


// SPI RX data handler
static void __interrupt(irq(IRQ_SPI1RX)) spi_rx_data_isr(void)
{
    uint8_t  data;

    // New RX data
    // Reading the FIFO clears the interrupt
    data = SPI1RXB;

    if (reg_callback != NULL)
    {
        switch (byte_num)
        {
            case 0:
                byte0 = data;
                // Read or write request?
                write_transfer = ((byte0 & 0x80) != 0);
                break;

            case 1:
                // Check the XOR'ed request
                error = ((data ^ 0x55) != byte0);

                // Decode the first byte
                reg_addr = byte0 & 0x7F;

                if (!error && !write_transfer)
                {
                    // For reads, pre-load the transmit reg now
                    reg_callback(reg_addr++, &tx_data, false);
                }
                break;

            default:
                if (!error)
                {
                    if (write_transfer)
                        reg_callback(reg_addr++, &data, true);
                    else
                    {
                        if (byte_num & 1)
                            reg_callback(reg_addr++, &tx_data, false);
                        else
                            tx_data ^= 0x55;
                    }
                }
                break;
        }
        byte_num++;
    }

    SPI1TXB = tx_data;
}


/*
 * Initialise SPI slave interface, call this first
 *
 * @param[in]   sclk_gpio    GPIO pin to use for SCLK, see above
 * @param[in]   mosi_gpio    GPIO pin to use for MOSI
 * @param[in]   miso_gpio    GPIO pin to use for MISO
 * @param[in]   nss_gpio     GPIO pin to use for nSS
 * @param[in]   mode         SPI mode to use, Bit 1:CPOL, Bit 0:CPHA
 * @param[in]   ttl_inputs   True to use TTL levels on the input signals
 * @param[in]   reg_cb       Callback function for register reads/writes
 */
void spi_slave_init(uint16_t sclk_gpio, uint16_t mosi_gpio, uint16_t miso_gpio, uint16_t nss_gpio,
                    uint8_t mode, bool ttl_inputs, spi_slave_reg_fn_t reg_cb)
{
    uint16_t  port;
    uint8_t   pin, input_setting;

    if (reg_cb == NULL)
        return;

    reg_callback = reg_cb;
    byte_num = 0;

    // Turn on the SPI peripheral block clocks
    PMD5bits.SPI1MD = 0;

    // Disable SPI to set it up
    SPI1CON0 = 0;
    SPI1INTE = 0;
    SPI1INTF = 0;

    input_setting = PIN_DIGITAL_IN | (ttl_inputs ? PIN_TTL_INPUT_LEVEL : 0);

    // Configure the pins
    if (sclk_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure SCLK initially as a GPIO input
        gpio_configure_pin(sclk_gpio, input_setting);

        port = sclk_gpio & 0x0F00;
        pin = sclk_gpio & 7;
        if (port == 0x0100)
        {
            // Port B PPS
            SPI1SCKPPS = 0x08 | pin;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            SPI1SCKPPS = 0x10 | pin;
        }
    }
    if (mosi_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure MOSI initially as a GPIO input
        gpio_configure_pin(mosi_gpio, input_setting);

        port = mosi_gpio & 0x0F00;
        pin = mosi_gpio & 7;
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
    if (miso_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure MISO initially as a low GPIO output
        gpio_configure_pin(miso_gpio, PIN_DIGITAL_OUT | PIN_SET_LOW);

        port = miso_gpio & 0x0F00;
        pin = miso_gpio & 7;
        if (port == 0x0100)
        {
            // Port B PPS
            *pps_outb_regs[pin] = PPS_OUT_MISO_SETTING;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            *pps_outc_regs[pin] = PPS_OUT_MISO_SETTING;
        }
    }
    if (nss_gpio != GPIO_NOT_CONNECTED)
    {
        // Configure nSS initially as a GPIO input
        gpio_configure_pin(nss_gpio, input_setting | PIN_PULL_UP);

        port = nss_gpio & 0x0F00;
        pin = nss_gpio & 7;
        if (port == 0x0000)
        {
            // Port A PPS
            SPI1SSPPS = pin;
        }
        else if (port == 0x0200)
        {
            // Port C PPS
            SPI1SSPPS = 0x10 | pin;
        }

        // Keep a record for the error interrupt
        nss_pin = nss_gpio;
    }

    // Use slave mode, 8 bits, MSB first
    SPI1TWIDTH = 0;
    SPI1CON0 = 1;

    // Set the mode
    mode = (uint8_t)(((mode & 2) << 4) | (((mode ^ 1) & 1) << 6));
    SPI1CON1 = 0x04 | mode;

    SPI1CON2 = 3;

    // Enable
    SPI1CON0 = 0x81;

    reset_fifos();

    // Enable the start of packet (nSS low) interrupt
    // and error interrupts
    SOSIE = 1;
    RXOIE = 1;
    TXUIE = 1;
    SPI1IE = 1;

    // Enable the RX data interrupt
    SPI1RXIE = 1;
}

