/**
 * @brief  PIC18 interrupt driver
 *         This file is not required on chips that support vectored interrupts
 *
 * Copyright Debug Innovations Ltd. 2018-2022
 */

#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)
#error "Don't build pic_interrupt.c on chips with vectored interrupts"
#endif

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "pic_interrupt.h"


// Deal with bit name differences
#ifdef _18F26Q10

#define U1TXIE  TX1IE
#define U1TXIF  TX1IF
#define U1RXIE  RC1IE
#define U1RXIF  RC1IF

#define U2TXIE  TX2IE
#define U2TXIF  TX2IF
#define U2RXIE  RC2IE
#define U2RXIF  RC2IF

#endif


static picint_fn_t  isr[NUM_INT_TYPES];


// This is the only interrupt on a PIC
// Test the interrupt flags and dispatch to registered ISRs
#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)
// C99 syntax
static void __interrupt() master_isr(void)
#else
// C90 syntax
static void interrupt master_isr(void)
#endif
{
    // GPIO pin edge
    if (IOCIE && IOCIF && (isr[INT_GPIO_PIN] != NULL))
    {
        isr[INT_GPIO_PIN]();
        // IOCIF is cleared by clearing the individual pin flags
    }

    if (PEIE)
    {
        // UART0 RX
        if (U1RXIE && (isr[INT_UART0_RX] != NULL))
        {
            // Using 'while' caters for chips with RX buffers
            // which might have more than one character to give
            while (U1RXIF)
            {
                isr[INT_UART0_RX]();
                // RCIF is cleared by reading the RX data
            }
        }

        // UART1 RX
        if (U2RXIE && (isr[INT_UART1_RX] != NULL))
        {
            // Using 'while' caters for chips with RX buffers
            // which might have more than one character to give
            while (U2RXIF)
            {
                isr[INT_UART1_RX]();
                // RCIF is cleared by reading the RX data
            }
        }

        // UART0 TX
        if (U1TXIE && U1TXIF && (isr[INT_UART0_TX] != NULL))
        {
            isr[INT_UART0_TX]();
            // TXIF is cleared by writing the TX data
        }

        // UART1 TX
        if (U2TXIE && U2TXIF && (isr[INT_UART1_TX] != NULL))
        {
            isr[INT_UART1_TX]();
            // TXIF is cleared by writing the TX data
        }

        // Timer 5
        if (TMR5IE && TMR5IF && (isr[INT_TIMER5] != NULL))
        {
            isr[INT_TIMER5]();
            TMR5IF = 0;
        }
    }

    // Timer 0
    if (TMR0IE && TMR0IF && (isr[INT_TIMER0] != NULL))
    {
        isr[INT_TIMER0]();
        TMR0IF = 0;
    }
}


/*
 * @brief Initialise the PIC interrupts
 *        This does not enable interrupts
 */
void picint_init(void)
{
    uint8_t  i;

    for (i = 0; i < NUM_INT_TYPES; i++)
        isr[i] = NULL;
}


/*
 * @brief Register a callback for a specific interrupt
 * @param[IN]  source     Hardware interrupt source
 * @param[IN]  callback   Interrupt service routine to call
 */
void picint_register(picint_t source, picint_fn_t callback)
{
    uint8_t  enable;

    enable = 0;

    if (source < NUM_INT_TYPES)
    {
        isr[source] = callback;

        if (callback != NULL)
            enable = 1;

        switch (source)
        {
            case INT_TIMER0:
                TMR0IF = 0;
                TMR0IE = (__bit)enable;
                break;

            case INT_TIMER5:
                TMR5IF = 0;
                TMR5IE = (__bit)enable;
                break;

            case INT_GPIO_PIN:
                // Don't enable yet - done in gpio_int_control()
                break;

            case INT_UART0_RX:
                // U1RXIF can't be cleared by software
                U1RXIE = (__bit)enable;
                break;

            case INT_UART0_TX:
                // U1TXIF can't be cleared by software and U1TXIE needs
                // to be set when there is data in the transmit buffer
                U1TXIE = 0;
                break;

            case INT_UART1_RX:
                // U2RXIF can't be cleared by software
                U2RXIE = (__bit)enable;
                break;

            case INT_UART1_TX:
                // U2TXIF can't be cleared by software and U2TXIE needs
                // to be set when there is data in the transmit buffer
                U2TXIE = 0;
                break;

            default:
                break;
        }
    }
}

