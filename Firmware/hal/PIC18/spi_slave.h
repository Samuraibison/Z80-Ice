/**
 * @brief  Interface to SPI slave implementations
 *
 * Copyright Debug Innovations Ltd. 2022
 */

#ifndef HAL_SPI_SLAVE_H
#define HAL_SPI_SLAVE_H

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// Function signature for a register read/write callback handler
typedef void (*spi_slave_reg_fn_t)(uint8_t reg_addr, uint8_t *reg_data, bool write);


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
                    uint8_t mode, bool ttl_inputs, spi_slave_reg_fn_t reg_cb);


#ifdef __cplusplus
}
#endif

#endif

