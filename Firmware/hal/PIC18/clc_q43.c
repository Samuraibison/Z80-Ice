/*
 * CLC (Configurable Logic Cell) driver for PIC18FxxQ43 chips
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

static volatile uint8_t * const input_pps[NUM_CLCS] = { &CLCIN0PPS, &CLCIN1PPS, &CLCIN2PPS, &CLCIN3PPS, &CLCIN4PPS, &CLCIN5PPS, &CLCIN6PPS, &CLCIN7PPS };

static volatile uint8_t * const input_mux[4] = { &CLCnSEL0, &CLCnSEL1, &CLCnSEL2, &CLCnSEL3 };
static volatile uint8_t * const data_gate[4] = { &CLCnGLS0, &CLCnGLS1, &CLCnGLS2, &CLCnGLS3 };


// Connect a pin to a CLC input channel, see table 21-1 for possible pins.
// @param[in]  clc       CLC block to use
// @param[in]  channel   CLC input mux channel (0-3)
// @param[in]  source    CLC input mux setting, see table 22-2
// @param[in]  gpio_num  Input pin (GPIO_NOT_CONNECTED if source isn't a pin)
void clc_input(uint8_t clc, uint8_t channel, uint8_t source, uint16_t gpio_num)
{
    uint8_t  port, pin;

    // Enable clocks to this CLC block
    PMD7 &= ~(1 << clc);

    // Select this CLC block for CLCn register accesses
    CLCSELECT = clc & 0x7;

    // Start by disabling the CLC block
    CLCnCON &= ~0x80;

    if ((gpio_num != GPIO_NOT_CONNECTED) && (source < 8))
    {
        // Coming from a GPIO pin
        gpio_configure_pin(gpio_num, PIN_DIGITAL_IN);

        port = (uint8_t)(gpio_num >> 8);
        pin = (uint8_t)gpio_num;
        *input_pps[source] = (uint8_t)(port << 3) | (pin & 7);
    }

    *input_mux[channel] = source;
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

    // Select this CLC block for CLCn register accesses
    CLCSELECT = clc & 0x7;

    *data_gate[gate_num] = gate_fn;

    mask = (uint8_t)(1 << gate_num);

    if (inv_out)
        CLCnPOL |= mask;
    else
        CLCnPOL &= ~mask;
}


// Combine the control signals with the chosen logic cell function.
// @param[in]  clc       CLC block to use (0-7)
// @param[in]  logic_fn  CLC data gate function (0-7) + control bits
// @param[in]  inv_out   If true, the final output signal will be inverted
void clc_function(uint8_t clc, uint8_t logic_fn, bool inv_out)
{
    // Select this CLC block for CLCn register accesses
    CLCSELECT = clc & 0x7;

    CLCnCON = logic_fn & 0x1F;

    if (inv_out)
        CLCnPOL |= 0x80;
    else
        CLCnPOL &= ~0x80;
}


// Connect a CLC output to a pin, see table 21-2 for possible pins.
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
        pps = 0x01 + clc;

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
    CLCSELECT = clc & 0x7;
    CLCnCON |= 0x80;
}

