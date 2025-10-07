/**
 * @brief  I2C master using the new PIC18 I2C peripheral
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83 (I2C1 only)
 *          26K83 (I2C1 only)
 *          26Q43
 *          27Q43
 *
 * Copyright Debug Innovations Ltd. 2021-2022
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "hal_chip.h"
#include "hal_time.h"
#include "hal_gpio.h"
#include "hal_i2c.h"


#define I2C_WRITE    0
#define I2C_READ     1

// Number of times we re-send the device address before giving up.
// Each retry takes ~125us at 400kHz SCL so this corresponds to
// a 10ms limit.  EEPROMs that stop ACK'ing when they are busy
// writing a sector take a max. time of ~5ms.
#define RETRY_LIMIT  80


// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];


#if defined(_18F25K83) || defined(_18F26K83)

// Settings for I2C1
#define PPS_OUT_SCL_SETTING  0x21
#define PPS_OUT_SDA_SETTING  0x22

#elif defined(_18F26Q43) || defined(_18F27Q43)

#define PPS_OUT_SCL_SETTING  0x37
#define PPS_OUT_SDA_SETTING  0x38

#endif


/*
 * Initialise I2C interface, call this first
 *
 * @param[out]  pins         Interface control structure
 * @param[in]   scl_gpio     GPIO pin to use for SCL
 * @param[in]   sda_gpio     GPIO pin to use for SDA
 * @param[in]   scl_Hz       Required SCL clock frequency
 *
 * @return true if successful
 */
bool i2c_init(i2c_pins_t *pins, uint16_t scl_gpio, uint16_t sda_gpio, uint32_t scl_Hz)
{
    uint32_t  cpu_clk_Hz, clk_div;
    uint16_t  port;
    uint8_t   pin, prescale;

    if (pins == NULL)
        return false;

    pins->scl_gpio = scl_gpio;
    pins->sda_gpio = sda_gpio;
    pins->params = 0;

    // Configure SDA and SCL as open drain outputs
    gpio_configure_pin(scl_gpio, PIN_DIGITAL_OUT | PIN_SET_HIGH | PIN_OPEN_DRAIN);
    gpio_configure_pin(sda_gpio, PIN_DIGITAL_OUT | PIN_SET_HIGH | PIN_OPEN_DRAIN);

    // Allow the signals to settle
    timer_delay_us(100);

    // Both pins should have pullups
    if (!((gpio_get(scl_gpio) == HIGH) && (gpio_get(sda_gpio) == HIGH)))
        return false;

    // Turn on the I2C peripheral block clocks
    I2C1MD = 0;

    // This chip has movable pin assignments
    port = scl_gpio & 0x0F00;
    pin = scl_gpio & 7;
    if (port == 0x0100)
    {
        // Port B PPS
        I2C1SCLPPS = 0x08 | pin;
        *pps_outb_regs[pin] = PPS_OUT_SCL_SETTING;
    }
    else if (port == 0x0200)
    {
        // Port C PPS
        I2C1SCLPPS = 0x10 | pin;
        *pps_outc_regs[pin] = PPS_OUT_SCL_SETTING;
    }

    port = sda_gpio & 0x0F00;
    pin = sda_gpio & 7;
    if (port == 0x0100)
    {
        // Port B PPS
        I2C1SDAPPS = 0x08 | pin;
        *pps_outb_regs[pin] = PPS_OUT_SDA_SETTING;
    }
    else if (port == 0x0200)
    {
        // Port C PPS
        I2C1SDAPPS = 0x10 | pin;
        *pps_outc_regs[pin] = PPS_OUT_SDA_SETTING;
    }

    // Put I2C in master mode
    I2C1CON0 = 0x84;
    I2C1CON1 = 0x80;
    I2C1CON2 = 0x10;

    // We use Timer 6 to generate the I2C clock
    TMR6MD = 0;

    // Input clock is 5 x SCL clock rate
    cpu_clk_Hz = chip_get_cpu_clock();
    clk_div = cpu_clk_Hz / (scl_Hz * 5);

    // If the requested rate is higher than (CPU clock / 5),
    // just run as fast as we can
    if (clk_div == 0)
        clk_div = 1;

    // If the required clock divider is a multiple of 4, we
    // can run the timer from Fosc/4 and save some power
    if ((clk_div & 3) == 0)
    {
        // Clock Timer 6 from Fosc/4
        T6CLKCON = 1;
        clk_div >>= 2;
    }
    else
        // Clock Timer 6 from Fosc
        T6CLKCON = 2;

    // Calculate the optimum pre-scale value
    for (prescale = 0; prescale < 8; prescale++)
    {
        if (clk_div & 1)
            break;

        clk_div >>= 1;
    }

    if (prescale == 8)
        prescale = 7;

    clk_div--;
    if (clk_div > 255)
        clk_div = 255;

    T6CON = 0x80 | (uint8_t)(prescale << 4);
    T6HLT = 0;
    T6PR = (uint8_t)clk_div;

    // Set the I2C clock source to Timer 6 output
#if defined(_18F25K83) || defined(_18F26K83)
    I2C1CLK = 0x08;
#else
    I2C1CLK = 0x09;
#endif

    // Clear the buffers
    I2C1STAT1bits.CLRBF = 1;

    return true;
}


// Waits until the bus is free
// Returns true if the bus becomes free within 10ms
static bool wait_until_ready(void)
{
    uint8_t  tries;

    tries = 0;

    while (I2C1STAT0bits.BFRE == 0)
    {
        timer_delay_us(100);

        // 10ms max wait
        if (++tries > 100)
            return false;
    }

    return true;
}


// Sends the initial part of the transfer containing the slave address.
// This I2C hardware is incapable of detecting an ACK by only sending
// the slave address (it stretches SCL before the ACK with no data),
// so we also do the register/memory address sending here.
// Returns true if the slave sends an ACK
static bool start_transfer(uint8_t request, uint32_t addr, uint8_t addr_bytes, uint8_t data_bytes, uint8_t num_tries)
{
    uint32_t  adr;
    uint8_t   i, tries;
    bool      ack;

    // Wait until the bus is free
    wait_until_ready();

    // Send the device address
    // If the device doesn't ACK the request, it may be busy
    // (EEPROMs do this during a write cycle)
    tries = 0;
    do
    {
        // Clear the buffers
        I2C1STAT1bits.CLRBF = 1;
        I2C1ERRbits.NACKIF = 0;

        I2C1CNT = addr_bytes + data_bytes;

        // Send the device address
        I2C1TXB = request;

        // Pre-calculate the address starting point
        adr = addr << (32 - (addr_bytes << 3));

        // Send the address in big endian order
        // We do an extra loop to detect the ACK/NACK on the last byte
        for (i = 0; i < (addr_bytes + 1); i++)
        {
            // Wait for buffer empty or NACK
            do
            {
                ack = (I2C1ERRbits.NACKIF == 0);
            }
            while (ack && (I2C1STAT1bits.TXBE == 0));

            if (!ack)
            {
                // NACK - retry
                if (++tries < num_tries)
                    timer_delay_us(100);
                break;
            }

            // Don't send a byte on the last loop
            if (i == addr_bytes)
                break;

            // Transmit a byte
            I2C1TXB = (uint8_t)(adr >> 24);
            adr <<= 8;
        }
    }
    while (!ack && (tries < num_tries));

    return ack;
}


/*
 * Test for an I2C slave device
 *
 * @param[in]   pins         I2C interface to use
 * @param[in]   dev_addr     I2C device address
 *
 * @return true if device ACKs
 */
bool i2c_probe(i2c_pins_t *pins, uint8_t dev_addr)
{
    bool  ack;

    if (pins == NULL)
        return false;

    // Check the device exists on the bus - we give it 2 chances to respond.
    // This hardware is awkward.  You can't just send the device address and
    // get an ACK/NACK back because it stalls the bus just before the ACK
    // slot waiting for the next byte, so we send a zero after the device
    // address.  If the device exists, it ACKs and a zero gets sent.  If the
    // device doesn't exist, it NACKs and the hardware sends an I2C STOP
    // after the device address.  There are some simple I2C devices that
    // don't need an address byte so sending a zero might actually cause a
    // problem but we don't have a choice with this hardware.
    I2C1CON0bits.RSEN = 0;
    ack = start_transfer((uint8_t)(dev_addr << 1) | I2C_WRITE, 0, 1, 0, 2);

    wait_until_ready();

    return ack;
}


/*
 * Adds a device to an I2C interface by building a device structure
 * and probing for the device
 *
 * @param[out]  device       Device to read/write.  This struct is populated by
 *                           the following parameters and should be used in calls
 *                           to the other functions.
 * @param[in]   pins         I2C interface the device is connected to
 * @param[in]   dev_addr     I2C device address
 *
 * @return true if device ACKs
 */
bool i2c_add_device(i2c_device_t *device, i2c_pins_t *pins, uint8_t dev_addr)
{
    if ((device == NULL) || (pins == NULL))
        return false;

    // Initialise the device struct
    device->pins = pins;
    device->dev_addr = dev_addr;

    // Check the device exists on the bus
    return i2c_probe(pins, dev_addr);
}


/*
 * Write num_bytes of data starting at addr
 * Generates an I2C write request followed by addr_size bytes of addr and num_bytes of data.
 * For devices that don't need an address, set addr_size to zero.
 *
 * @param[in]   pins         I2C interface to use
 * @param[in]   dev_addr     I2C device address
 * @param[in]   addr         Address to start writing at
 * @param[in]   addr_size    Number of bytes of addr to send, max. 4
 * @param[in]   num_bytes    Number of bytes of data to write
 * @param[in]   data         Pointer to data to write
 *
 * @return true if successful
 */
bool i2c_write_data(i2c_pins_t *pins, uint8_t dev_addr, uint32_t addr, uint8_t addr_size, uint8_t num_bytes, uint8_t *data)
{
    uint8_t  i;

    if ((pins == NULL) || (dev_addr > 0x7F) || (addr_size > 4))
        return false;

    if ((num_bytes > 0) && (data == NULL))
        return false;

    if ((addr_size == 0) && (num_bytes == 0))
        // Nothing to do
        return true;

    I2C1CON0bits.RSEN = 0;

    if (!start_transfer((uint8_t)(dev_addr << 1) | I2C_WRITE, addr, addr_size, num_bytes, RETRY_LIMIT))
        return false;

    // Send the data
    for (i = 0; i < num_bytes; i++)
    {
        while (I2C1STAT1bits.TXBE == 0)
            ;

        I2C1TXB = data[i];
    }

    // Wait until the bus is free
    // This is not strictly necessary because the next call will wait but it
    // is very confusing when stepping code if the write doesn't complete
    // when the breakpoint stops the clocks (the final write doesn't happen)
    wait_until_ready();

    return true;
}


/*
 * Read num_bytes of data starting at addr
 * If addr_size is zero, this generates an I2C read request, then reads num_bytes of data.
 * If addr_size is non-zero, this generates an I2C write request followed by addr_size bytes
 * of addr in big-endian order, then an I2C read request of num_bytes of data.
 *
 * @param[in]   pins         I2C interface to use
 * @param[in]   dev_addr     I2C device address
 * @param[in]   addr         Address to start reading from
 * @param[in]   addr_size    Number of bytes of addr to send, max. 4
 * @param[in]   num_bytes    Number of bytes of data to read
 * @param[out]  data         Pointer to buffer for read data
 *
 * @return true if successful
 */
bool i2c_read_data(i2c_pins_t *pins, uint8_t dev_addr, uint32_t addr, uint8_t addr_size, uint8_t num_bytes, uint8_t *data)
{
    uint8_t  i, request;
    bool     ack;

    if ((pins == NULL) || (data == NULL) || (dev_addr > 0x7F) || (addr_size > 4))
        return false;

    if (num_bytes == 0)
        // Nothing to do
        return true;

    I2C1PIRbits.CNTIF = 0;
    I2C1CON0bits.RSEN = 1;

    // Send the device address
    if (addr_size == 0)
        request = (uint8_t)(dev_addr << 1) | I2C_READ;
    else
        request = (uint8_t)(dev_addr << 1) | I2C_WRITE;

    if (!start_transfer(request, addr, addr_size, 0, RETRY_LIMIT))
    {
        I2C1CON0bits.RSEN = 0;
        return false;
    }

    // Make sure the whole address phase has finished
    while (I2C1PIRbits.CNTIF == 0)
        ;

    // Start the read phase
    I2C1CNT = num_bytes;

    if (addr_size > 0)
    {
        // Send the device address
        I2C1TXB = (uint8_t)(dev_addr << 1) | I2C_READ;

        // Wait for buffer empty or NACK
        do
        {
            ack = (I2C1ERRbits.NACKIF == 0);
        }
        while (ack && (I2C1STAT1bits.TXBE == 0));

        if (!ack)
        {
            I2C1CON0bits.RSEN = 0;
            return false;
        }
    }

    // From here on, it is safe to turn off restarts so when I2C1CNT
    // gets to zero, an I2C STOP will be signalled
    I2C1CON0bits.RSEN = 0;

    // Read the registers
    for (i = 0; i < num_bytes; i++)
    {
        while (I2C1STAT1bits.RXBF == 0)
            ;

        data[i] = I2C1RXB;
    }

    // Wait until the bus is free
    // This is not strictly necessary because the next call will wait but it
    // is very confusing when stepping code if the read doesn't complete when
    // the breakpoint stops the clocks (the stop signal is not generated)
    wait_until_ready();

    return true;
}


/*
 * Write register(s) starting at first_reg, next reg is at first_reg addr + 1 etc.
 * This is a wrapper for i2c_write_data() which is compatible with devices having less than 256 registers.
 *
 * @param[in]   device       Device to write
 * @param[in]   first_reg    Register number to start writing at
 * @param[in]   num_regs     Number of registers to write
 * @param[in]   data         Pointer to register values to write
 *
 * @return true if successful
 */
bool i2c_write_regs(i2c_device_t *device, uint8_t first_reg, uint8_t num_regs, uint8_t *data)
{
    if (device == NULL)
        return false;

    return i2c_write_data(device->pins, device->dev_addr, first_reg, 1, num_regs, data);
}


/*
 * Read register(s) starting at first_reg, next reg is at first_reg addr + 1 etc.
 * This is a wrapper for i2c_read_data() which is compatible with devices having less than 256 registers.
 *
 * @param[in]   device       Device to read
 * @param[in]   first_reg    Register number to start reading at
 * @param[in]   num_regs     Number of registers to read
 * @param[out]  data         Pointer to user storage for register values
 *
 * @return true if successful
 */
bool i2c_read_regs(i2c_device_t *device, uint8_t first_reg, uint8_t num_regs, uint8_t *data)
{
    if (device == NULL)
        return false;

    return i2c_read_data(device->pins, device->dev_addr, first_reg, 1, num_regs, data);
}

