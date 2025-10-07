/**
 * @brief  Interface to I2C master implementations
 *
 * General usage instructions...
 *
 * 1. Call i2c_init() to initialise an I2C interface
 * 2. Call i2c_add_device() for each device on that interface
 * 3. i2c_read_regs() and i2c_write_regs() give simple register-level
 *    access to most devices
 * 4. i2c_read_data() and i2c_write_data() give more control for
 *    devices with larger addresses such as EEPROMs
 * 5. Drivers for specific devices can be found in drivers/i2c
 *    Drivers typically call i2c_add_device() for you.
 *
 * Copyright Debug Innovations Ltd. 2014-2022
 */

#ifndef HAL_I2C_H
#define HAL_I2C_H

#include <stdint.h>
#include <stdbool.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// I2C pins comprising an I2C interface
struct i2c_pins
{
    uint16_t  scl_gpio;
    uint16_t  sda_gpio;
    uint8_t   params;
};
typedef struct i2c_pins  i2c_pins_t;


// I2C device
// This identifies a specific device on a specific I2C interface
struct i2c_device
{
    i2c_pins_t  *pins;        // I2C bus the device is on
    void        *dev_state;   // Device state pointer for drivers
    uint8_t     dev_addr;     // I2C device address
};
typedef struct i2c_device  i2c_device_t;


/*
 * Initialise I2C interface, call this first
 *
 * In the bit-bashed I2C driver and some hardware I2C drivers, the pins to use
 * for SCL/SDA are movable and chosen by their GPIO numbers.  On I2C hardware
 * which has fixed pins for SCL/SDA, the scl_gpio parameter is used as the I2C
 * interface number starting at zero.
 *
 * @param[out]  pins         Interface control structure
 * @param[in]   scl_gpio     GPIO pin to use for SCL, see above
 * @param[in]   sda_gpio     GPIO pin to use for SDA
 * @param[in]   scl_Hz       Required SCL clock frequency
 *
 * @return true if successful
 */
bool i2c_init(i2c_pins_t *pins, uint16_t scl_gpio, uint16_t sda_gpio, uint32_t scl_Hz);


/*
 * Test for an I2C slave device
 *
 * @param[in]   pins         I2C interface to use
 * @param[in]   dev_addr     I2C device address
 *
 * @return true if device ACKs
 */
bool i2c_probe(i2c_pins_t *pins, uint8_t dev_addr);


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
bool i2c_add_device(i2c_device_t *device, i2c_pins_t *pins, uint8_t dev_addr);


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
bool i2c_write_data(i2c_pins_t *pins, uint8_t dev_addr, uint32_t addr, uint8_t addr_size, uint8_t num_bytes, uint8_t *data);


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
bool i2c_read_data(i2c_pins_t *pins, uint8_t dev_addr, uint32_t addr, uint8_t addr_size, uint8_t num_bytes, uint8_t *data);


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
bool i2c_write_regs(i2c_device_t *device, uint8_t first_reg, uint8_t num_regs, uint8_t *data);


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
bool i2c_read_regs(i2c_device_t *device, uint8_t first_reg, uint8_t num_regs, uint8_t *data);


#ifdef __cplusplus
}
#endif

#endif

