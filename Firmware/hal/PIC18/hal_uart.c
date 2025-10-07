/**
 * @brief  PIC18 HAL UART driver
 *         Note: Relies on global interrupts being enabled
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83
 *          26K83
 *          26Q10
 *          26Q43
 *          27Q43
 *
 * Only 8N1 mode is supported and no RTS/CTS.
 *
 * Copyright Debug Innovations Ltd. 2018-2023
 */

// Suppress some annoying XC8 compiler warnings
#pragma warning disable 1498   // Pointer may have no targets
#pragma warning disable 1471   // Indirect NULL function call ignored

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "pic_interrupt.h"
#include "ring_buffer.h"
#include "hal_config.h"
#include "hal_chip.h"
#include "hal_gpio.h"
#include "hal_uart.h"


// Deal with bit name differences
#if defined(_18F26Q10)

#define U1MD    UART1MD
#define U1IE    PEIE
#define U1TXIE  TX1IE
#define U1RXIE  RC1IE

#define U2MD    UART2MD
#define U2IE    PEIE
#define U2TXIE  TX2IE
#define U2RXIE  RC2IE

#endif


#if defined(_18F26Q43) || defined(_18F27Q43)

// The Q43 parts have 5 UARTs
#define NUM_UARTS  5

static const int      tx_buff_size[NUM_UARTS] = { UART0_TX_BUFF_SIZE, UART1_TX_BUFF_SIZE, UART2_TX_BUFF_SIZE, UART3_TX_BUFF_SIZE, UART4_TX_BUFF_SIZE };
static const int      rx_buff_size[NUM_UARTS] = { UART0_RX_BUFF_SIZE, UART1_RX_BUFF_SIZE, UART2_RX_BUFF_SIZE, UART3_RX_BUFF_SIZE, UART4_RX_BUFF_SIZE };
static const uint8_t  pps_txd_out[NUM_UARTS]  = { 0x20, 0x23, 0x26, 0x29, 0x2C };

#else

// The K83 parts and PIC18F26Q10 have 2 UARTs
#define NUM_UARTS  2

static const int      tx_buff_size[NUM_UARTS] = { UART0_TX_BUFF_SIZE, UART1_TX_BUFF_SIZE };
static const int      rx_buff_size[NUM_UARTS] = { UART0_RX_BUFF_SIZE, UART1_RX_BUFF_SIZE };

#if defined(_18F25K83) || defined(_18F26K83)
static const uint8_t  pps_txd_out[NUM_UARTS]  = { 0x13, 0x16 };
#else
static const uint8_t  pps_txd_out[NUM_UARTS]  = { 0x09, 0x0B };
#endif

#endif


#if (UART0_TX_BUFF_SIZE > 0)
static uint8_t     tx_buff0[UART0_TX_BUFF_SIZE];
#else
#define tx_buff0   NULL
#endif

#if (UART0_RX_BUFF_SIZE > 0)
static uint8_t     rx_buff0[UART0_RX_BUFF_SIZE];
#else
#define rx_buff0   NULL
#endif

#if (UART1_TX_BUFF_SIZE > 0)
static uint8_t     tx_buff1[UART1_TX_BUFF_SIZE];
#else
#define tx_buff1   NULL
#endif

#if (UART1_RX_BUFF_SIZE > 0)
static uint8_t     rx_buff1[UART1_RX_BUFF_SIZE];
#else
#define rx_buff1   NULL
#endif

#if (NUM_UARTS > 2)

#if (UART2_TX_BUFF_SIZE > 0)
static uint8_t     tx_buff2[UART2_TX_BUFF_SIZE];
#else
#define tx_buff2   NULL
#endif

#if (UART2_RX_BUFF_SIZE > 0)
static uint8_t     rx_buff2[UART2_RX_BUFF_SIZE];
#else
#define rx_buff2   NULL
#endif

#if (UART3_TX_BUFF_SIZE > 0)
static uint8_t     tx_buff3[UART3_TX_BUFF_SIZE];
#else
#define tx_buff3   NULL
#endif

#if (UART3_RX_BUFF_SIZE > 0)
static uint8_t     rx_buff3[UART3_RX_BUFF_SIZE];
#else
#define rx_buff3   NULL
#endif

#if (UART4_TX_BUFF_SIZE > 0)
static uint8_t     tx_buff4[UART4_TX_BUFF_SIZE];
#else
#define tx_buff4   NULL
#endif

#if (UART4_RX_BUFF_SIZE > 0)
static uint8_t     rx_buff4[UART4_RX_BUFF_SIZE];
#else
#define rx_buff4   NULL
#endif

#endif


// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outa_regs[];
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];


#if defined(_18F25K83) || defined(_18F26K83)

static uint8_t * const  tx_buff[NUM_UARTS] = { tx_buff0, tx_buff1 };
static uint8_t * const  rx_buff[NUM_UARTS] = { rx_buff0, rx_buff1 };

static uart_rx_fn_t  data_received[NUM_UARTS] = { NULL, NULL };

static volatile uint16_t * const  brg_regs[]  = { &U1BRG, &U2BRG };
static volatile uint8_t  * const  txd_regs[]  = { &U1TXB, &U2TXB };
static volatile uint8_t  * const  rxd_regs[]  = { &U1RXB, &U2RXB };
static volatile uint8_t  * const  con0_regs[] = { &U1CON0, &U2CON0 };
static volatile uint8_t  * const  con1_regs[] = { &U1CON1, &U2CON1 };
static volatile uint8_t  * const  err_regs[]  = { &U1ERRIR, &U2ERRIR };
static volatile uint8_t  * const  fifo_regs[] = { &U1FIFO, &U2FIFO };

#elif defined(_18F26Q43) || defined(_18F27Q43)

static uint8_t * const  tx_buff[NUM_UARTS] = { tx_buff0, tx_buff1, tx_buff2, tx_buff3, tx_buff4 };
static uint8_t * const  rx_buff[NUM_UARTS] = { rx_buff0, rx_buff1, rx_buff2, rx_buff3, rx_buff4 };

static uart_rx_fn_t  data_received[NUM_UARTS] = { NULL, NULL, NULL, NULL, NULL };

static volatile uint16_t * const  brg_regs[]  = { &U1BRG, &U2BRG, &U3BRG, &U4BRG, &U5BRG };
static volatile uint8_t  * const  txd_regs[]  = { &U1TXB, &U2TXB, &U3TXB, &U4TXB, &U5TXB };
static volatile uint8_t  * const  rxd_regs[]  = { &U1RXB, &U2RXB, &U3RXB, &U4RXB, &U5RXB };
static volatile uint8_t  * const  con0_regs[] = { &U1CON0, &U2CON0, &U3CON0, &U4CON0, &U5CON0 };
static volatile uint8_t  * const  con1_regs[] = { &U1CON1, &U2CON1, &U3CON1, &U4CON1, &U5CON1 };
static volatile uint8_t  * const  err_regs[]  = { &U1ERRIR, &U2ERRIR, &U3ERRIR, &U4ERRIR, &U5ERRIR };
static volatile uint8_t  * const  fifo_regs[] = { &U1FIFO, &U2FIFO, &U3FIFO, &U4FIFO, &U5FIFO };

#else

static uint8_t * const  tx_buff[NUM_UARTS] = { tx_buff0, tx_buff1 };
static uint8_t * const  rx_buff[NUM_UARTS] = { rx_buff0, rx_buff1 };

static uart_rx_fn_t  data_received[NUM_UARTS]  = { NULL, NULL };

static volatile uint16_t * const  brg_regs[]   = { &SP1BRG, &SP2BRG };
static volatile uint8_t  * const  txd_regs[]   = { &TX1REG, &TX2REG };
static volatile uint8_t  * const  rxd_regs[]   = { &RC1REG, &RC2REG };
static volatile uint8_t  * const  txsta_regs[] = { &TX1STA, &TX2STA };
static volatile uint8_t  * const  rxsta_regs[] = { &RC1STA, &RC2STA };

#endif


#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)

// Enable TX by setting the TXEN bit
#define TX_ENABLE   *con0_regs[uart_num] |= _U1CON0_TXEN_MASK
#define TX_DISABLE  *con0_regs[uart_num] &= ~_U1CON0_TXEN_MASK

// Enable RX by setting the RXEN bit
#define RX_ENABLE   *con0_regs[uart_num] |= _U1CON0_RXEN_MASK
#define RX_DISABLE  *con0_regs[uart_num] &= ~_U1CON0_RXEN_MASK

// RX error bits
#define RX_OERR     (*err_regs[uart_num] & _U1ERRIR_RXFOIF_MASK)
#define RX_FERR     (*err_regs[uart_num] & _U1ERRIR_FERIF_MASK)

// TX complete bit
#define TX_DONE     (*err_regs[uart_num] & _U1ERRIR_TXMTIF_MASK)

#else

// Enable TX by setting the TXEN bit
#define TX_ENABLE   *txsta_regs[uart_num] |= _TXSTA_TXEN_MASK
#define TX_DISABLE  *txsta_regs[uart_num] &= ~_TXSTA_TXEN_MASK

// Enable RX by setting the CREN bit
#define RX_ENABLE   *rxsta_regs[uart_num] |= _RCSTA_CREN_MASK
#define RX_DISABLE  *rxsta_regs[uart_num] &= ~_RCSTA_CREN_MASK

// RX error bits
#define RX_OERR     (*rxsta_regs[uart_num] & _RCSTA_OERR_MASK)
#define RX_FERR     (*rxsta_regs[uart_num] & _RCSTA_FERR_MASK)

// TX complete bit
#define TX_DONE     (*txsta_regs[uart_num] & _TXSTA_TRMT_MASK)

#endif


static ringbuff_t   tx_ring[NUM_UARTS], rx_ring[NUM_UARTS];
static uint8_t      saved_clocks[NUM_UARTS];
static uint16_t     saved_brg_regs[NUM_UARTS];
static uint8_t      saved_con0_regs[NUM_UARTS];
static uint8_t      saved_con1_regs[NUM_UARTS];
static bool         first_pass = true;


// Because the interrupt control bits are scattered all over the register space,
// we have to use a function to change the bits.  We could use an array of register
// pointers but we'd still have to read-modify-write the bits and if the position
// of a bit moved registers between different PICs, we'd have a total mess, so the
// easiest way is to use the pre-defined bit names which means switch statements.

#define bit_enable  (__bit)enable

// Set/clear UART master interrupt bit
static void uart_master_int(uint8_t uart_num, uint8_t enable)
{
    switch (uart_num)
    {
        case 0:
            U1IE = bit_enable;
            break;

        case 1:
            U2IE = bit_enable;
            break;

#if (NUM_UARTS > 2)

        case 2:
            U3IE = bit_enable;
            break;

        case 3:
            U4IE = bit_enable;
            break;

        case 4:
            U5IE = bit_enable;
            break;

#endif
    }
}


// Enable/disable UART transmit interrupt
static void uart_tx_int(uint8_t uart_num, uint8_t enable)
{
    switch (uart_num)
    {
        case 0:
            U1TXIE = bit_enable;
            break;

        case 1:
            U2TXIE = bit_enable;
            break;

#if (NUM_UARTS > 2)

        case 2:
            U3TXIE = bit_enable;
            break;

        case 3:
            U4TXIE = bit_enable;
            break;

        case 4:
            U5TXIE = bit_enable;
            break;

#endif
    }
}


// Enable/disable UART receive interrupt
static void uart_rx_int(uint8_t uart_num, uint8_t enable)
{
    switch (uart_num)
    {
        case 0:
            U1RXIE = bit_enable;
            break;

        case 1:
            U2RXIE = bit_enable;
            break;

#if (NUM_UARTS > 2)

        case 2:
            U3RXIE = bit_enable;
            break;

        case 3:
            U4RXIE = bit_enable;
            break;

        case 4:
            U5RXIE = bit_enable;
            break;

#endif
    }
}


// Enable/disable UART clocks
static void uart_clock(uint8_t uart_num, uint8_t enable)
{
    // Enable bits are active low
    enable ^= 1;

    switch (uart_num)
    {
        case 0:
            U1MD = bit_enable;
            break;

        case 1:
            U2MD = bit_enable;
            break;

#if (NUM_UARTS > 2)

        case 2:
            U3MD = bit_enable;
            break;

        case 3:
            U4MD = bit_enable;
            break;

        case 4:
            U5MD = bit_enable;
            break;

#endif
    }
}


// UART TX interrupt handler
static void tx_int(uint8_t uart_num)
{
    int  tx_data;

    tx_data = ringbuff_get(&tx_ring[uart_num]);

    if (tx_data == -1)
        // Ring buffer is empty - disable TX interrupt
        uart_tx_int(uart_num, 0);
    else
        // Load the TX data reg
        *txd_regs[uart_num] = (uint8_t)tx_data;
}


// UART RX interrupt handler
static void rx_int(uint8_t uart_num)
{
    uint8_t  rx_data;
    int      res;

    rx_data = *rxd_regs[uart_num];

    // Put the byte into the ring buffer if there is space
    res = ringbuff_put(&rx_ring[uart_num], rx_data);

    if (res == -1)
        // Buffer full
        return;

    // Notify RX callback handler if required
    if (data_received[uart_num] != NULL)
        data_received[uart_num](uart_num, rx_data);
}


// Whether or not the specified UART is enabled (set up)
static bool uart_enabled(uint8_t uart_num)
{
#ifdef _18F26Q10
    // Read the SPEN bit in the RX status reg
    return (uart_num < NUM_UARTS) && ((*rxsta_regs[uart_num] & _RCSTA_SPEN_MASK) != 0);
#else
    // Read the ON bit in the CON1 reg
    return (uart_num < NUM_UARTS) && ((*con1_regs[uart_num] & _U1CON1_ON_MASK) != 0);
#endif
}


#ifdef _18F26Q10

// These interrupt handlers are for chips without vectored interrupt
// support which are called from pic_interrupt.c

static void tx_int0(void)
{
    tx_int(0);
}

static void tx_int1(void)
{
    tx_int(1);
}

static void rx_int0(void)
{
    rx_int(0);
}

static void rx_int1(void)
{
    rx_int(1);
}

#endif


#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)

// These interrupt handlers are for chips with vectored interrupt support
// Other chips use pic_interrupt.c

// UART0 TX handler entry point
static void __interrupt(irq(IRQ_U1TX)) uart0_tx_isr(void)
{
    tx_int(0);
}

// UART1 TX handler entry point
static void __interrupt(irq(IRQ_U2TX)) uart1_tx_isr(void)
{
    tx_int(1);
}

#if (NUM_UARTS > 2)

// UART2 TX handler entry point
static void __interrupt(irq(IRQ_U3TX)) uart2_tx_isr(void)
{
    tx_int(2);
}

// UART3 TX handler entry point
static void __interrupt(irq(IRQ_U4TX)) uart3_tx_isr(void)
{
    tx_int(3);
}

// UART4 TX handler entry point
static void __interrupt(irq(IRQ_U5TX)) uart4_tx_isr(void)
{
    tx_int(4);
}

#endif


// UART0 RX handler entry point
static void __interrupt(irq(IRQ_U1RX)) uart0_rx_isr(void)
{
    while (U1RXIF)
        rx_int(0);
}

// UART1 RX handler entry point
static void __interrupt(irq(IRQ_U2RX)) uart1_rx_isr(void)
{
    while (U2RXIF)
        rx_int(1);
}

#if (NUM_UARTS > 2)

// UART2 RX handler entry point
static void __interrupt(irq(IRQ_U3RX)) uart2_rx_isr(void)
{
    while (U3RXIF)
        rx_int(2);
}

// UART3 RX handler entry point
static void __interrupt(irq(IRQ_U4RX)) uart3_rx_isr(void)
{
    while (U4RXIF)
        rx_int(3);
}

// UART4 RX handler entry point
static void __interrupt(irq(IRQ_U5RX)) uart4_rx_isr(void)
{
    while (U5RXIF)
        rx_int(4);
}

#endif

#endif


// Puts the UARTs to sleep to save power
static void sleep_entry(void)
{
    uint8_t  uart_num;

    // Remember the registers that are zeroed when we turn
    // off the UART clocks (for no apparent reason !)
    for (uart_num = 0; uart_num < NUM_UARTS; uart_num++)
    {
        uart_wait_for_transmit_complete(uart_num);
        saved_con0_regs[uart_num] = *con0_regs[uart_num];
        saved_con1_regs[uart_num] = *con1_regs[uart_num];
        saved_brg_regs[uart_num] = *brg_regs[uart_num];
    }

    // Switch off the UART clocks
    saved_clocks[0] = U1MD;
    U1MD = 1;
    saved_clocks[1] = U2MD;
    U2MD = 1;

#if defined(_18F26Q43) || defined(_18F27Q43)
    saved_clocks[2] = U3MD;
    U3MD = 1;
    saved_clocks[3] = U4MD;
    U4MD = 1;
    saved_clocks[4] = U5MD;
    U5MD = 1;
#endif
}


// Wakes the UARTs from sleep
static void sleep_exit(void)
{
    uint8_t  uart_num;

    // Turn on the clocks
    U1MD = (__bit)saved_clocks[0];
    U2MD = (__bit)saved_clocks[1];

#if defined(_18F26Q43) || defined(_18F27Q43)
    U3MD = (__bit)saved_clocks[2];
    U4MD = (__bit)saved_clocks[3];
    U5MD = (__bit)saved_clocks[4];
#endif

    // Restore the saved registers
    for (uart_num = 0; uart_num < NUM_UARTS; uart_num++)
    {
        *con0_regs[uart_num] = saved_con0_regs[uart_num];
        *con1_regs[uart_num] = saved_con1_regs[uart_num];
        *brg_regs[uart_num] = saved_brg_regs[uart_num];
    }
}


// This isn't officially part of hal_uart.h but can be called
// to change the baud rate after uart_init()
void uart_set_baud_rate(uint8_t uart_num, uint32_t baud_rate)
{
    uint32_t  cpu_clk_Hz, count;
    uint8_t   reg;
    bool      brg_div4;

    cpu_clk_Hz = chip_get_cpu_clock();

    // Calculate and set baud rate
    count = cpu_clk_Hz / baud_rate;

    // The count value has to fit in 16 bits after division and we
    // have a choice of division by 4 (BRGH=1) or 16 (BRGH=0)
    if (count > 0x3FFFF)
    {
        // Too big, divide by 16 (4 now, 4 later)
        count >>= 2;
        brg_div4 = false;
    }
    else
        // Divide by 4 for more accurate baud rates
        brg_div4 = true;

    // Calculate (rounded count / 4) - 1
    if (count & 2)
        // Round up
        count >>= 2;
    else
        // Round down
        count = (count >> 2) - 1;

#ifdef _18F26Q10
    // The SPEN bit is in the RX status reg but actually turns the
    // whole UART on, the CREN bit may or may not have been enabled
    // yet so we need to preserve it, clear out the rest of the bits
    reg = *rxsta_regs[uart_num];
    reg &= _RCSTA_CREN_MASK;
    reg |= _RCSTA_SPEN_MASK;
    *rxsta_regs[uart_num] = reg;

    // 8-bit async mode, set BRGH, the CREN bit may or may not have
    // been enabled yet so we need to preserve it
    reg = *txsta_regs[uart_num];
    reg &= _TXSTA_TXEN_MASK;
    reg |= brg_div4 ? _TXSTA_BRGH_MASK : 0;
    *txsta_regs[uart_num] = reg;

    // Use 16-bit baud rate generator, disable auto-baud
    if (uart_num == 0)
        BAUD1CON = _BAUD1CON_BRG16_MASK;
    else
        BAUD2CON = _BAUD2CON_BRG16_MASK;
#else
    // Turn on the whole UART and clear the rest of CON1
    *con1_regs[uart_num] = _U1CON1_ON_MASK;

    // 8-bit async mode, set BRGH, disable auto-baud, TXEN and RXEN may
    // or may not have been enabled yet so we need to preserve them
    reg = *con0_regs[uart_num];
    reg &= (_U1CON0_TXEN_MASK | _U1CON0_RXEN_MASK);
    reg |= brg_div4 ? _U1CON0_BRGS_MASK : 0;
    *con0_regs[uart_num] = reg;
#endif

    // Load the baud rate count
    *brg_regs[uart_num] = (uint16_t)count;
}


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
bool uart_init(uint8_t uart_num, uart_settings_t *settings)
{
    uint16_t  port;
    uint8_t   pullup, rxd_pps;
    uint8_t   *buff;

    if (uart_num >= NUM_UARTS)
        return false;

    // Turn on the UART clocks
    uart_clock(uart_num, 1);

    // Disable TX/RX interrupts
    uart_master_int(uart_num, 0);
    uart_tx_int(uart_num, 0);
    uart_rx_int(uart_num, 0);

    // Set up a pair of ring buffers for the data
    buff = tx_buff[uart_num];
    if (buff != NULL)
        (void)ringbuff_init(&tx_ring[uart_num], buff, (unsigned int)tx_buff_size[uart_num]);

    buff = rx_buff[uart_num];
    if (buff != NULL)
        (void)ringbuff_init(&rx_ring[uart_num], buff, (unsigned int)rx_buff_size[uart_num]);

    data_received[uart_num] = settings->rx_callback;

    // Setup the baud rate counter
    uart_set_baud_rate(uart_num, settings->baud_rate);

    // Enable RX pin and switch receiver on
    if (settings->rxd_gpio != GPIO_NOT_CONNECTED)
    {
        pullup = settings->pullup_inputs ? PIN_PULL_UP : 0;
        gpio_configure_pin(settings->rxd_gpio, PIN_DIGITAL_IN | pullup);

        // These chips have movable pin assignments
        port = settings->rxd_gpio & 0x0F00;
        if (port == 0x0000)
            // Port A PPS
            rxd_pps = settings->rxd_gpio & 7;
        else if (port == 0x0100)
            // Port B PPS
            rxd_pps = 0x08 | (settings->rxd_gpio & 7);
        else if (port == 0x0200)
            // Port C PPS
            rxd_pps = 0x10 | (settings->rxd_gpio & 7);
        else
            rxd_pps = 0;

        // Assign the RXD pin
#ifdef _18F26Q10
        if (uart_num == 0)
            RX1PPS = rxd_pps;
        else
            RX2PPS = rxd_pps;
#else
        switch (uart_num)
        {
            case 0:
                U1RXPPS = rxd_pps;
                break;

            case 1:
                U2RXPPS = rxd_pps;
                break;

#if (NUM_UARTS > 2)
            case 2:
                U3RXPPS = rxd_pps;
                break;

            case 3:
                U4RXPPS = rxd_pps;
                break;

            case 4:
                U5RXPPS = rxd_pps;
                break;
#endif
        }
#endif

#ifdef _18F26Q10
        // Register RX interrupt handler
        if (rx_buff[uart_num] != NULL)
        {
            if (uart_num == 0)
                picint_register(INT_UART0_RX, rx_int0);
            else
                picint_register(INT_UART1_RX, rx_int1);
        }
#endif
        // Enable RX interrupts
        uart_rx_int(uart_num, 1);

        // Turn on the receiver
        RX_ENABLE;
    }

    // Enable TX pin and switch transmitter on
    if (settings->txd_gpio != GPIO_NOT_CONNECTED)
    {
        gpio_configure_pin(settings->txd_gpio, PIN_DIGITAL_OUT | PIN_SET_HIGH);

        // These chips have movable pin assignments
        port = settings->txd_gpio & 0x0F00;
        if (port == 0x0000)
            // Port A PPS
            *pps_outa_regs[settings->txd_gpio & 7] = pps_txd_out[uart_num];
        else if (port == 0x0100)
            // Port B PPS
            *pps_outb_regs[settings->txd_gpio & 7] = pps_txd_out[uart_num];
        else if (port == 0x0200)
            // Port C PPS
            *pps_outc_regs[settings->txd_gpio & 7] = pps_txd_out[uart_num];

#ifdef _18F26Q10
        // Register TX interrupt handler
        if (tx_buff[uart_num] != NULL)
        {
            if (uart_num == 0)
                picint_register(INT_UART0_TX, tx_int0);
            else
                picint_register(INT_UART1_TX, tx_int1);
        }
#endif
        // Turn on the transmitter
        TX_ENABLE;
    }

    // Turn on master interrupts for this UART
    uart_master_int(uart_num, 1);

    if (first_pass)
    {
        // We want to finish sending all the text before a CPU sleep or reset
        chip_register_event_callback(CHIP_EVENT_SLEEP, sleep_entry);
        chip_register_event_callback(CHIP_EVENT_WAKE, sleep_exit);
        chip_register_event_callback(CHIP_EVENT_RESET, uart_wait_for_all_transmit_complete);

        first_pass = false;
    }

    return true;
}


/**
 * @brief Close a UART port.
 *        On Windows/Linux targets, closes an open port.
 *        Does nothing on embedded targets.
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_close(uint8_t uart_num)
{
    // Stub function - does nothing on a PIC
}


/**
 * @brief Transmit bytes over a UART
 *        This implementation buffers the data and does not block,
 *        data is lost when the buffer is full
 * @param[in]  uart_num    0 to n depending on the device
 * @param[in]  data        Data to transmit
 * @param[in]  num_bytes   Number of bytes to transmit
 */
void uart_transmit(uint8_t uart_num, uint8_t *data, int num_bytes)
{
    ringbuff_t  *rb;
    int         i, res;

    if (uart_enabled(uart_num) && (num_bytes > 0))
    {
        rb = &tx_ring[uart_num];

        for (i = 0; i < num_bytes; i++)
        {
            // Put one char in the TX buffer
            // If TX interrupts are enabled, there is a risk one
            // will happen during the ring buffer access so disable
            // them first.
            uart_tx_int(uart_num, 0);
            res = ringbuff_put(rb, data[i]);

            // We know there is at least one character to be sent so
            // enable transmit interrupts. The ISR will empty the ring
            // buffer into the TX data register.
            uart_tx_int(uart_num, 1);

            if (res == -1)
                // Buffer full
                break;
        }
    }
}


static void check_for_rx_errors(uint8_t uart_num)
{
    // If we have an error, reset the receiver
    // or we won't get any more data
    if (RX_FERR || RX_OERR)
    {
        RX_DISABLE;
        NOP();
        NOP();
        RX_ENABLE;
        NOP();
        NOP();
    }
}


/**
 * @brief Receive bytes over a UART
 * @param[in]      uart_num    0 to n depending on the device
 * @param[out]     data        Received data
 * @param[in/out]  num_bytes   Number of bytes wanted/received
 */
void uart_receive(uint8_t uart_num, uint8_t *data, int *num_bytes)
{
    ringbuff_t  *rb;
    int         max_bytes, ch;

    max_bytes = *num_bytes;
    *num_bytes = 0;

    if (uart_enabled(uart_num))
    {
        check_for_rx_errors(uart_num);
        rb = &rx_ring[uart_num];

        // Loop until we have the requested number of bytes or run out of bytes
        do
        {
            // Read one char from the RX buffer
            uart_rx_int(uart_num, 0);
            ch = ringbuff_get(rb);
            uart_rx_int(uart_num, 1);

            if (ch == -1)
                // No more bytes available
                break;

            data[(*num_bytes)++] = ch & 0xFF;
        }
        while (*num_bytes < max_bytes);
    }
}


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
void uart_peek(uint8_t uart_num, uint8_t *data, int *num_bytes)
{
    ringbuff_t  *rb;
    int         ch;

    *num_bytes = 0;

    if (uart_enabled(uart_num))
    {
        check_for_rx_errors(uart_num);
        rb = &rx_ring[uart_num];

        uart_rx_int(uart_num, 0);
        ch = ringbuff_peek(rb);
        uart_rx_int(uart_num, 1);

        if (ch != -1)
            data[(*num_bytes)++] = (uint8_t)ch;
    }
}


/**
 * @brief Get the sizes of the tx/rx buffers
 * @param[in]   uart_num   0 to n depending on the device
 * @param[out]  tx_size    Size of the transmit buffer in bytes, NULL if not required
 * @param[out]  rx_size    Size of the receive buffer in bytes, NULL if not required
 */
void uart_get_buff_sizes(uint8_t uart_num, int *tx_size, int *rx_size)
{
    if (tx_size != NULL)
        *tx_size = tx_buff_size[uart_num];

    if (rx_size != NULL)
        *rx_size = rx_buff_size[uart_num];
}


/**
 * @brief Get the number of bytes currently stored in the tx/rx buffers
 * @param[in]   uart_num   0 to n depending on the device
 * @param[out]  tx_count   Number of bytes in the tx buffer, NULL if not required
 * @param[out]  rx_count   Number of bytes in the rx buffer, NULL if not required
 */
void uart_get_buff_count(uint8_t uart_num, int *tx_count, int *rx_count)
{
    ringbuff_t  *rb;
    int         count;

    // Default values
    if (tx_count != NULL)
        *tx_count = 0;

    if (rx_count != NULL)
        *rx_count = 0;

    if (uart_enabled(uart_num))
    {
        if (tx_count != NULL)
        {
            rb = &tx_ring[uart_num];

            // We can't use TXIE to block TX interrupts while we access the ring
            // buffer because TXIE may not be enabled and enabling it after the
            // access would cause an interrupt.  So instead we turn off the UART
            // master interrupt.
            uart_master_int(uart_num, 0);
            count = ringbuff_len(rb);
            uart_master_int(uart_num, 1);

            *tx_count = count;
        }

        if (rx_count != NULL)
        {
            rb = &rx_ring[uart_num];

            uart_rx_int(uart_num, 0);
            count = ringbuff_len(rb);
            uart_rx_int(uart_num, 1);

            *rx_count = count;
        }
    }
}


/**
 * @brief Flush UART TX buffered data
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_flush_tx(uint8_t uart_num)
{
    if (uart_enabled(uart_num))
    {
        // Disable the TX interrupt and empty the TX ring buffer
        uart_tx_int(uart_num, 0);
        ringbuff_clear(&tx_ring[uart_num]);

        // Don't re-enable the TX interrupt - that will happen
        // when there is some data to send
    }
}


/**
 * @brief Flush UART RX buffered data
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_flush_rx(uint8_t uart_num)
{
    uint8_t  junk;

    if (uart_enabled(uart_num))
    {
        // Reset the receiver in case we have errors
        RX_DISABLE;
        NOP();
        NOP();
        RX_ENABLE;

        // Disable RX interrupts - we don't want any new data
        uart_rx_int(uart_num, 0);

        // Empty the RX hardware buffer
#ifdef _18F26Q10
        if (uart_num == 0)
        {
            while (RC1IF)
                junk = *rxd_regs[uart_num];
        }
        else
        {
            while (RC2IF)
                junk = *rxd_regs[uart_num];
        }
#else
        while ((*fifo_regs[uart_num] & _U1FIFO_RXBE_MASK) == 0)
            junk = *rxd_regs[uart_num];
#endif

        // and the RX ring buffer
        ringbuff_clear(&rx_ring[uart_num]);

        // Re-enable the RX interrupt
        uart_rx_int(uart_num, 1);
    }
}


/**
 * @brief Returns true if all the buffered data for the specified UART has been transmitted
 * @param[in]  uart_num   0 to n depending on the device
 */
bool uart_transmit_complete(uint8_t uart_num)
{
    int  num_tx_bytes;

    if (!uart_enabled(uart_num))
        // Return value for an error doesn't mean anything so
        // return true to avoid locking up user code
        return true;

    // First check for bytes in the ring buffer
    uart_get_buff_count(uart_num, &num_tx_bytes, NULL);
    if (num_tx_bytes != 0)
        return false;

    // Then check the UART
    return (TX_DONE != 0);
}


/**
 * @brief Waits until all the buffered data for the specified UART has been transmitted
 * @param[in]  uart_num   0 to n depending on the device
 */
void uart_wait_for_transmit_complete(uint8_t uart_num)
{
    while (!uart_transmit_complete(uart_num))
        ;
}


/**
 * @brief Waits until all the UARTs have finished transmitting
 */
void uart_wait_for_all_transmit_complete(void)
{
    uint8_t  i;

    for (i = 0; i < NUM_UARTS; i++)
        uart_wait_for_transmit_complete(i);
}

