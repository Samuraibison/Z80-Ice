/*
 * Embedded code hardware abstraction layer API
 *    - UART access functions
 *
 * Copyright Debug Innovations Ltd. 2010-2023
 */

#ifndef HAL_UART_H
#define HAL_UART_H

#include <stdint.h>
#include <stdbool.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// Parity settings
enum parity_type
{
    UART_PARITY_NONE,
    UART_PARITY_ODD,
    UART_PARITY_EVEN,
    NUM_PARITY_TYPES
};
typedef enum parity_type  parity_t;


/* UART receive callback handler must conform to this function signature.
 * When called by the HAL, uart_num indicates which UART has made the
 * callback and rx_data is the received character.  The data has already
 * been buffered at this point so the handler should flush the buffer if
 * waiting for a specific character.
 */
typedef void (*uart_rx_fn_t)(uint8_t uart_num, uint8_t rx_data);


/* UART settings */
struct uart_settings
{
    // Encoded port and pin numbers e.g. 0x0102 is port 1, pin 2
    // Set to GPIO_NOT_CONNECTED if not used
    uint16_t  txd_gpio;   // TXD output
    uint16_t  rxd_gpio;   // RXD input
    uint16_t  rts_gpio;   // RTS output
    uint16_t  cts_gpio;   // CTS input

    // Required baud rate and format
    uint32_t  baud_rate;
    uint8_t   num_data_bits;  // Normally 8
    parity_t  parity;

    // If true, input pins RXD and CTS have their pullups enabled
    // to maintain a valid logic level if the pin is disconnected
    bool      pullup_inputs;

    // If true, a falling edge on RXD will wake the CPU from a low
    // power state.  Whether this is useful is chip-dependent.
    bool      wake_on_edge;

    // Receive data callback function
    // Set to NULL if not required
    uart_rx_fn_t  rx_callback;
};
typedef struct uart_settings  uart_settings_t;


/**
 * @brief Initialise a UART channel.
 *        Call this before using any of the other UART calls.
 *        The settings struct should be populated before calling.
 * @param[in]  uart_num   0 to n depending on the device
 * @param[in]  settings   Required settings
 *
 * @returns true if successful
 *          For embedded targets, this indicates that the uart_num
 *          and settings are valid and it is probably ok to call
 *          again with different settings.  For Windows/Linux targets,
 *          this indicates the port was successfully opened and you
 *          will have to call uart_close() before calling uart_init()
 *          again.
 */
bool uart_init(uint8_t uart_num, uart_settings_t *settings);


/**
 * @brief Close a UART port.
 *        On Windows/Linux targets, closes an open port.
 *        Does nothing on embedded targets.
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_close(uint8_t uart_num);


/**
 * @brief Transmit bytes over a UART
 *        Most implementations buffer the data, though it is possible
 *        this call will block.  Behaviour when the buffer is full is
 *        implementation-dependent.  Typically data is lost when the
 *        buffer is full but implementations may block until the UART
 *        is ready.  The size of the transmit buffer is specified in
 *        hal_config.h and can be read with uart_get_buff_sizes().
 * @param[in]  uart_num    0 to n depending on the device
 * @param[in]  data        Data to transmit
 * @param[in]  num_bytes   Number of bytes to transmit
 */
void uart_transmit(uint8_t uart_num, uint8_t *data, int num_bytes);


/**
 * @brief Receive bytes over a UART
 * @param[in]      uart_num    0 to n depending on the device
 * @param[out]     data        Received data
 * @param[in/out]  num_bytes   Number of bytes wanted/received
 */
void uart_receive(uint8_t uart_num, uint8_t *data, int *num_bytes);


/**
 * @brief Peek at next byte in UART receive buffer (but don't remove
 *        from buffer).  Note that, unlike uart_receive(), num_bytes
 *        is an output parameter and does not indicate the number of
 *        bytes in data[] which is always 0 or 1.
 * @param[in]   uart_num    0 to n depending on the device
 * @param[out]  data        Next byte available to be read from the
 *                          buffer - not valid if num_bytes is zero
 * @param[out]  num_bytes   Number of bytes put in data[]
 */
void uart_peek(uint8_t uart_num, uint8_t *data, int *num_bytes);


/**
 * @brief Get the sizes of the tx/rx buffers
 * @param[in]   uart_num   0 to n depending on the device
 * @param[out]  tx_size    Size of the transmit buffer in bytes, NULL if not required
 * @param[out]  rx_size    Size of the receive buffer in bytes, NULL if not required
 */
void uart_get_buff_sizes(uint8_t uart_num, int *tx_size, int *rx_size);


/**
 * @brief Get the number of bytes currently stored in the tx/rx buffers
 * @param[in]   uart_num   0 to n depending on the device
 * @param[out]  tx_count   Number of bytes in the tx buffer, NULL if not required
 * @param[out]  rx_count   Number of bytes in the rx buffer, NULL if not required
 */
void uart_get_buff_count(uint8_t uart_num, int *tx_count, int *rx_count);


/**
 * @brief Flush UART TX buffered data
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_flush_tx(uint8_t uart_num);


/**
 * @brief Flush UART RX buffered data
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_flush_rx(uint8_t uart_num);


/**
 * @brief Returns true if all the buffered data for the specified UART has been transmitted
 * @param[in]  uart_num   0 to n depending on the device
 */
bool uart_transmit_complete(uint8_t uart_num);


/**
 * @brief Waits until all the buffered data for the specified UART has been transmitted
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_wait_for_transmit_complete(uint8_t uart_num);


/**
 * @brief Waits until all the UARTs have finished transmitting
 */
void uart_wait_for_all_transmit_complete(void);


#ifdef __cplusplus
}
#endif

#endif

