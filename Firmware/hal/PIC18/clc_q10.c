/*
 * CLC (Configurable Logic Cell) driver for PIC18FxxQ10 chips
 *
 * Copyright Debug Innovations Ltd. 2021-2024
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal_gpio.h"
#include "clc.h"


#define NUM_CLCS  8

// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outa_regs[];
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];

static volatile uint8_t * const clc_ctrl[NUM_CLCS] = { &CLC1CON, &CLC2CON, &CLC3CON, &CLC4CON, &CLC5CON, &CLC6CON, &CLC7CON, &CLC8CON };

static volatile uint8_t * const input_pps[NUM_CLCS] = { &CLCIN0PPS, &CLCIN1PPS, &CLCIN2PPS, &CLCIN3PPS, &CLCIN4PPS, &CLCIN5PPS, &CLCIN6PPS, &CLCIN7PPS };

static volatile uint8_t * const gate_pol[NUM_CLCS] = { &CLC1POL, &CLC2POL, &CLC3POL, &CLC4POL, &CLC5POL, &CLC6POL, &CLC7POL, &CLC8POL };

static volatile uint8_t * const input_mux[NUM_CLCS][4] = { { &CLC1SEL0, &CLC1SEL1, &CLC1SEL2, &CLC1SEL3 },
                                                           { &CLC2SEL0, &CLC2SEL1, &CLC2SEL2, &CLC2SEL3 },
                                                           { &CLC3SEL0, &CLC3SEL1, &CLC3SEL2, &CLC3SEL3 },
                                                           { &CLC4SEL0, &CLC4SEL1, &CLC4SEL2, &CLC4SEL3 },
                                                           { &CLC5SEL0, &CLC5SEL1, &CLC5SEL2, &CLC5SEL3 },
                                                           { &CLC6SEL0, &CLC6SEL1, &CLC6SEL2, &CLC6SEL3 },
                                                           { &CLC7SEL0, &CLC7SEL1, &CLC7SEL2, &CLC7SEL3 },
                                                           { &CLC8SEL0, &CLC8SEL1, &CLC8SEL2, &CLC8SEL3 } };

static volatile uint8_t * const data_gate[NUM_CLCS][4] = { { &CLC1GLS0, &CLC1GLS1, &CLC1GLS2, &CLC1GLS3 },
                                                           { &CLC2GLS0, &CLC2GLS1, &CLC2GLS2, &CLC2GLS3 },
                                                           { &CLC3GLS0, &CLC3GLS1, &CLC3GLS2, &CLC3GLS3 },
                                                           { &CLC4GLS0, &CLC4GLS1, &CLC4GLS2, &CLC4GLS3 },
                                                           { &CLC5GLS0, &CLC5GLS1, &CLC5GLS2, &CLC5GLS3 },
                                                           { &CLC6GLS0, &CLC6GLS1, &CLC6GLS2, &CLC6GLS3 },
                                                           { &CLC7GLS0, &CLC7GLS1, &CLC7GLS2, &CLC7GLS3 },
                                                           { &CLC8GLS0, &CLC8GLS1, &CLC8GLS2, &CLC8GLS3 } };


// Enables the clocks to a CLC block
static void enable_clocks(uint8_t clc)
{
    if (clc < 4)
        PMD5 &= ~(0x10 << clc);
    else
        PMD3 &= ~(0x10 << (clc - 4));
}


// Connect a pin to a CLC input channel, see table 17-1 for possible pins.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  channel   CLC input mux channel (0-3)
// @param[in]  source    CLC input mux setting (0-63)
// @param[in]  gpio_num  Input pin (GPIO_NOT_CONNECTED if source isn't a pin)
void clc_input(uint8_t clc, uint8_t channel, uint8_t source, uint16_t gpio_num)
{
    uint8_t  port, pin;

    // Enable clocks to this CLC block
    enable_clocks(clc);

    // Start by disabling the CLC block
    *clc_ctrl[clc] &= ~0x80;

    if ((gpio_num != GPIO_NOT_CONNECTED) && (source < 8))
    {
        // Coming from a GPIO pin
        gpio_configure_pin(gpio_num, PIN_DIGITAL_IN);

        port = (uint8_t)(gpio_num >> 8);
        pin = (uint8_t)gpio_num;
        *input_pps[source] = (uint8_t)(port << 3) | (pin & 7);
    }

    *input_mux[clc][channel] = source;
}


// Gate the CLC input channels to produce a control signal, there are
// 4 gates on each CLC, each gate producing one control signal.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  gate_num  CLC data gate number (0-3)
// @param[in]  gate_fn   CLC data gate function
// @param[in]  inv_out   If true, the control signal will be inverted
void clc_gate(uint8_t clc, uint8_t gate_num, uint8_t gate_fn, bool inv_out)
{
    uint8_t  mask;

    *data_gate[clc][gate_num] = gate_fn;

    mask = (uint8_t)(1 << gate_num);

    if (inv_out)
        *gate_pol[clc] |= mask;
    else
        *gate_pol[clc] &= ~mask;
}


// Combine the control signals with the chosen logic cell function.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  logic_fn  CLC data gate function (0-7) + control bits
// @param[in]  inv_out   If true, the final output signal will be inverted
void clc_function(uint8_t clc, uint8_t logic_fn, bool inv_out)
{
    *clc_ctrl[clc] = logic_fn & 0x1F;

    if (inv_out)
        *gate_pol[clc] |= 0x80;
    else
        *gate_pol[clc] &= ~0x80;
}


// Connect a CLC output to a pin, see table 17-2 for possible pins.
// This will also enable the CLC block.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  gpio_num  Output pin (GPIO_NOT_CONNECTED if used internally)
void clc_output(uint8_t clc, uint16_t gpio_num)
{
    uint8_t  port, pin, pps;

    gpio_configure_pin(gpio_num, PIN_DIGITAL_OUT | PIN_HIGH_SPEED);

    if (gpio_num != GPIO_NOT_CONNECTED)
    {
        // Output going to a GPIO pin
        port = (uint8_t)(gpio_num >> 8);
        pin = (uint8_t)gpio_num;
        pps = 0x18 + clc;

        if (port == 0)
            // Port A PPS
            *pps_outa_regs[pin] = pps;
        else if (port == 1)
            // Port B PPS
            *pps_outb_regs[pin] = pps;
        else if (port == 2)
            // Port C PPS
            *pps_outc_regs[pin] = pps;
    }

    // Enable the block
    *clc_ctrl[clc] |= 0x80;
}

