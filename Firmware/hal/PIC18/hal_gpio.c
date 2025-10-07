/**
 * @brief PIC18 HAL GPIO and ADC driver
 *        Note: Interrupts rely on GIE being set
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83
 *          26K83
 *          26Q10
 *          26Q43
 *          27Q43
 *
 * Copyright Debug Innovations Ltd. 2018-2024
 */

// Suppress some annoying XC8 compiler warnings, mostly because
// different bits of code are used with different PICs
#pragma warning disable 373   // Signed to unsigned conversions
#pragma warning disable 520   // Uncalled functions
#pragma warning disable 1090  // Unused variables
#pragma warning disable 759   // Expression generates no code
#pragma warning disable 1471  // Function call that XC thinks is NULL

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_config.h"
#include "lf_pics.h"
#include "pic_interrupt.h"
#include "hal_time.h"
#include "hal_gpio.h"


// The simplest way to deal with devices that don't have all the ports
// pinned out, is to add 'ports' so we can use generic code throughout
#ifndef PORTB
volatile uint8_t PORTB, LATB, TRISB;
#endif

#ifndef PORTD
volatile uint8_t PORTD, LATD, TRISD;
#endif

#ifndef ODCOND
volatile uint8_t ODCOND;
#endif

#ifndef PORTE
volatile uint8_t PORTE;
#endif
volatile uint8_t LATE, TRISE;

#ifndef ODCONE
volatile uint8_t ODCONE;
#endif

#ifndef ANSELA
volatile uint8_t ANSELA;
#endif

#ifndef ANSELB
volatile uint8_t ANSELB;
#endif

#ifndef ANSELC
volatile uint8_t ANSELC;
#endif

#ifndef ANSELD
volatile uint8_t ANSELD, ANSELE;
#endif

// Other register differences between devices
#if defined(nRBPU) && !defined(_OPTION_REG_nWPUEN_POSN)
#define nWPUEN  nRBPU
#endif

// Older devices don't have individual control of pullups
#ifndef WPUA
static uint8_t  WPUA;
#endif

#ifndef WPUB
static uint8_t  WPUB;
#endif

#ifndef WPUC
static uint8_t  WPUC;
#endif

#ifndef WPUD
static uint8_t  WPUD;
#endif

#ifndef WPUE
static uint8_t  WPUE;
#endif

// Deal with ADC differences
#if !defined(ADGO) && !defined(ADGO_bit)
#define ADGO  GO
#endif

#if !defined(ADFM) && !defined(ADFM_bit)
#define ADFM  ADFM0
#endif

// Deal with DAC differences
#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q10)
#define DAC1MD    DACMD
#define DAC1CON   DAC1CON0
#define DAC1DATL  DAC1CON1
#endif


// We support interrupts on up to 4 pins
#define MAX_PIN_INTS      4

struct gpio_int_channel
{
    uint16_t          gpio_num;
    uint8_t           pos_edge;
    uint8_t           neg_edge;
    uint8_t           mask;
    volatile uint8_t  *flag_reg;
    volatile uint8_t  *input_reg;
    gpio_int_fn_t     gpio_isr;
};
typedef struct gpio_int_channel  gpio_int_channel_t;


static volatile uint8_t * const input_regs[]    = { &PORTA,   &PORTB,   &PORTC,   &PORTD,  &PORTE  };
static volatile uint8_t * const output_regs[]   = { &LATA,    &LATB,    &LATC,    &LATD,   &LATE   };
static volatile uint8_t * const tris_regs[]     = { &TRISA,   &TRISB,   &TRISC,   &TRISD,  &TRISE  };
static volatile uint8_t * const pullup_regs[]   = { &WPUA,    &WPUB,    &WPUC,    &WPUD,   &WPUE   };
static volatile uint8_t * const od_regs[]       = { &ODCONA,  &ODCONB,  &ODCONC,  &ODCOND, &ODCONE };
static volatile uint8_t * const ansel_regs[]    = { &ANSELA,  &ANSELB,  &ANSELC,  &ANSELD, &ANSELE };
static volatile uint8_t * const ttl_in_regs[]   = { &INLVLA,  &INLVLB,  &INLVLC,  NULL,    &INLVLE };
static volatile uint8_t * const fast_out_regs[] = { &SLRCONA, &SLRCONB, &SLRCONC, NULL,    NULL    };
static volatile uint8_t * const int_flag_regs[] = { &IOCAF,   &IOCBF,   &IOCCF,   NULL,    &IOCEF  };
static volatile uint8_t * const int_pos_edge[]  = { &IOCAP,   &IOCBP,   &IOCCP,   NULL,    &IOCEP  };
static volatile uint8_t * const int_neg_edge[]  = { &IOCAN,   &IOCBN,   &IOCCN,   NULL,    &IOCEN  };


// These chips have movable pin assignments
// These are public arrays used by other HAL modules
volatile uint8_t * const pps_outa_regs[] = { &RA0PPS, &RA1PPS, &RA2PPS, &RA3PPS, &RA4PPS, &RA5PPS, &RA6PPS, &RA7PPS };
volatile uint8_t * const pps_outb_regs[] = { &RB0PPS, &RB1PPS, &RB2PPS, &RB3PPS, &RB4PPS, &RB5PPS, &RB6PPS, &RB7PPS };
volatile uint8_t * const pps_outc_regs[] = { &RC0PPS, &RC1PPS, &RC2PPS, &RC3PPS, &RC4PPS, &RC5PPS, &RC6PPS, &RC7PPS };


static int  pin_count = 0;

// Lookup table to optimise variable bit shifts
static const uint8_t bitmask[] = { 1, 2, 4, 8, 0x10, 0x20, 0x40, 0x80 };

// List of registered interrupts
static gpio_int_channel_t  pin_ints[MAX_PIN_INTS];

// Number of registered interrupts
static uint8_t  num_pin_ints;


/*
 * @brief Initialise the GPIO pins
 * @param[IN]  config  - The number of pins on the package
 */
void gpio_init(int config)
{
    gpio_int_channel_t  *channel;
    uint8_t             i;

    pin_count = config;

    channel = pin_ints;

    for (i = 0; i < MAX_PIN_INTS; i++)
    {
        channel->gpio_num = GPIO_NOT_CONNECTED;
        channel->pos_edge = 0;
        channel->neg_edge = 0;
        channel->mask = 0;
        channel->flag_reg = NULL;
        channel->input_reg = NULL;
        channel->gpio_isr = NULL;
        channel++;
    }
    num_pin_ints = 0;

    // Initialise all ports to be inputs
    TRISA = 0xFF;
    TRISB = 0xFF;
    TRISC = 0xFF;
    TRISD = 0xFF;
    TRISE = 0xFF;

    // Zero all the ports & latches
    PORTA = 0;
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;

    LATA = 0;
    LATB = 0;
    LATC = 0;
    LATD = 0;
    LATE = 0;

    // Configure analog inputs as digital I/O
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;

    // Turn on all the pullups
#ifdef nWPUEN
    nWPUEN = 0;
#endif
    WPUA = 0xFF;
    WPUB = 0xFF;
    WPUC = 0xFF;
    WPUD = 0xFF;
    WPUE = 0xFF;
}


/**
 * @brief Configure a GPIO pin
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @param[IN]  pin_mode - Pin type/direction etc. see GPIO pin modes in hal_gpio.h
 */
void gpio_configure_pin(uint16_t gpio_num, uint8_t pin_mode)
{
    uint8_t  port, pin, mask;

    if (gpio_num != GPIO_NOT_CONNECTED)
    {
        port = (uint8_t)((gpio_num >> 8) & 0xFF);
        pin = (uint8_t)(gpio_num & 7);
        mask = bitmask[pin];

        // Set the pullup on supported ports if required
        // PICs don't have pull-down resistors
        if (pin_mode & PIN_PULL_UP)
            *pullup_regs[port] |= mask;
        else
            *pullup_regs[port] &= ~mask;

        // If the pin is an analog input/output, enable it
        // Not all of these are ADC inputs, they can be comparators, opamps etc.
        // or analog outputs such as a DAC pin
        if (pin_mode & PIN_MODE_ANALOG)
        {
            // Analog pin, turn off the digital input buffer
            *ansel_regs[port] |= mask;

            // Turn off the digital output driver
            *tris_regs[port] |= mask;
        }
        else
        {
            // Digital pin
            *ansel_regs[port] &= ~mask;

            // Set the direction
            if ((pin_mode & PIN_DIR_MASK) == PIN_DIR_OUT)
            {
                // These chips have movable pin assignments
                if (port == 0)
                    // Port A PPS
                    *pps_outa_regs[pin] = 0;
                else if (port == 1)
                    // Port B PPS
                    *pps_outb_regs[pin] = 0;
                else if (port == 2)
                    // Port C PPS
                    *pps_outc_regs[pin] = 0;

                // Configure the output type (push-pull or open-drain)
                if (pin_mode & PIN_OPEN_DRAIN)
                    *od_regs[port] |= mask;
                else
                    *od_regs[port] &= ~mask;

                // Set the initial output value then configure as an output
                if (pin_mode & PIN_SET_HIGH)
                    *output_regs[port] |= mask;
                else
                    *output_regs[port] &= ~mask;

                *tris_regs[port] &= ~mask;

                // If possible, set the output drive speed
                if (fast_out_regs[port] != NULL)
                {
                    if (pin_mode & PIN_HIGH_SPEED)
                        *fast_out_regs[port] &= ~mask;
                    else
                        *fast_out_regs[port] |= mask;
                }
            }
            else
            {
                // Configure as an input
                *tris_regs[port] |= mask;

                // Set the input threshold
                if (ttl_in_regs[port] != NULL)
                {
                    if (pin_mode & PIN_TTL_INPUT_LEVEL)
                        *ttl_in_regs[port] &= ~mask;
                    else
                        *ttl_in_regs[port] |= mask;
                }
            }
        }
    }
}


/**
 * @brief Configure a whole GPIO port
 * @param[IN]  port - Port number, for lettered ports 0 = Port A
 * @param[IN]  mode - Port type/direction etc. see GPIO pin modes above
 */
void gpio_configure_port(uint8_t port, uint8_t mode)
{
    uint8_t  pin;

    // Set the pullup on supported ports if required
    // PICs don't have pull-down resistors
    if (mode & PIN_PULL_UP)
        *pullup_regs[port] = 0xFF;
    else
        *pullup_regs[port] = 0;

    // If the whole port is an analog input/output, enable it
    // Not all of these are ADC inputs, they can be comparators, opamps etc.
    // or analog outputs such as a DAC pin
    if (mode & PIN_MODE_ANALOG)
    {
        // Analog pins, turn off the digital input buffer
        *ansel_regs[port] = 0xFF;

        // Turn off the digital output driver
        *tris_regs[port] = 0xFF;
    }
    else
    {
        // Digital pins
        *ansel_regs[port] = 0;

        // Set the direction
        if ((mode & PIN_DIR_MASK) == PIN_DIR_OUT)
        {
            // These chips have movable pin assignments
            for (pin = 0; pin < 8; pin++)
            {
                if (port == 0)
                    // Port A PPS
                    *pps_outa_regs[pin] = 0;
                else if (port == 1)
                    // Port B PPS
                    *pps_outb_regs[pin] = 0;
                else if (port == 2)
                    // Port C PPS
                    *pps_outc_regs[pin] = 0;
            }

            // Configure the output type (push-pull or open-drain)
            if (mode & PIN_OPEN_DRAIN)
                *od_regs[port] = 0xFF;
            else
                *od_regs[port] = 0;

            // Set the initial output value then configure as an output
            if (mode & PIN_SET_HIGH)
                *output_regs[port] = 0xFF;
            else
                *output_regs[port] = 0;

            *tris_regs[port] = 0;

            // If possible, set the output drive speed
            if (fast_out_regs[port] != NULL)
            {
                if (mode & PIN_HIGH_SPEED)
                    *fast_out_regs[port] = 0;
                else
                    *fast_out_regs[port] = 0xFF;
            }
        }
        else
        {
            // Configure as an input
            *tris_regs[port] = 0xFF;

            // Set the input threshold
            if (ttl_in_regs[port] != NULL)
            {
                if (mode & PIN_TTL_INPUT_LEVEL)
                    *ttl_in_regs[port] = 0;
                else
                    *ttl_in_regs[port] = 0xFF;
            }
        }
    }
}


/**
 * @brief Set or clear a digital output pin
 * @param[IN]  gpio_num  - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @param[IN]  pin_level - New value for pin, 1/0
 */
void gpio_set(uint16_t gpio_num, uint8_t pin_level)
{
    uint8_t  port, pin, mask;

    if (gpio_num != GPIO_NOT_CONNECTED)
    {
        port = (uint8_t)((gpio_num >> 8) & 0xFF);
        pin = (uint8_t)(gpio_num & 7);
        mask = bitmask[pin];

        if (pin_level == LOW)
            *output_regs[port] &= ~mask;
        else
            *output_regs[port] |= mask;
    }
}


/**
 * @brief Set the value of a digital output port
 * @param[IN]  port  - Port number, for lettered ports 0 = Port A
 * @param[IN]  value - New value for port
 */
void gpio_set_port(uint8_t port, uint32_t value)
{
    *output_regs[port] = (uint8_t)value;
}


/**
 * @brief Get the current value of a digital input
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @return Input pin level - 1/0
 */
uint8_t gpio_get(uint16_t gpio_num)
{
    uint8_t  port, pin, mask;

    if (gpio_num == GPIO_NOT_CONNECTED)
        return 0;

    port = (uint8_t)((gpio_num >> 8) & 0xFF);
    pin = (uint8_t)(gpio_num & 7);
    mask = bitmask[pin];

    return (*input_regs[port] & mask) ? HIGH : LOW;
}


/**
 * @brief Get the value of a digital input port
 * @param[IN]  port - Port number, for lettered ports 0 = Port A
 * @return Input port value
 */
uint32_t gpio_get_port(uint8_t port)
{
    return (uint32_t)(*input_regs[port]);
}


#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)
// Entry point for chips with vectored interrupts
#define PIN_INT_ENTRY __interrupt(irq(IRQ_IOC))
#else
// Normal function called from interrupt.c
#define PIN_INT_ENTRY
#endif

// Pin interrupt handler (called for all configured GPIO input changes)
static void PIN_INT_ENTRY pin_int(void)
{
    gpio_int_channel_t  *channel;
    volatile uint8_t    *flag_reg;
    uint8_t             i, mask, flags;

    channel = pin_ints;

    for (i = 0; i < num_pin_ints; i++)
    {
        // Read the interrupt flags for this port and mask the pin
        flag_reg = channel->flag_reg;
        mask = channel->mask;
        flags = *flag_reg & mask;

        // If this is the pin we are looking for...
        if (flags)
        {
            // Call the user's interrupt function
            channel->gpio_isr(channel->gpio_num, (*channel->input_reg & mask) ? HIGH : LOW);

            // Clear this one interrupt flag
            *flag_reg &= ~flags;
        }
        channel++;
    }
}


/**
 * @brief Setup interrupts on a digital input pin (but don't enable them).
 *        Use this function to associate an interrupt handler with a GPIO pin.
 *        The handler should be a regular C function, not marked 'interrupt'.
 *        May be called a second time to change the handler function or the
 *        pin_state setting.  This function disables the pin interrupt before
 *        applying the setting - use gpio_int_control() to enable the interrupt.
 * @param[IN]  gpio_num  - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @param[IN]  handler   - Interrupt handler to call, cannot be NULL
 * @param[IN]  pin_state - Edge or level to interrupt on
 */
void gpio_int_setup(uint16_t gpio_num, gpio_int_fn_t handler, gpio_int_setting_t pin_state)
{
    gpio_int_channel_t  *channel;
    uint8_t             i, port, pin, mask, pos_edge, neg_edge;
    bool                enable_handler;

    if (gpio_num == GPIO_NOT_CONNECTED)
        return;

    port = (uint8_t)((gpio_num >> 8) & 0xFF);
    pin = (uint8_t)(gpio_num & 7);
    mask = bitmask[pin];

    // Start by disabling interrupts from this pin
    *int_pos_edge[port] &= ~mask;
    *int_neg_edge[port] &= ~mask;
    *int_flag_regs[port] &= ~mask;

#ifdef _18F26Q10
    // Register our interrupt function with the master ISR
    // It doesn't matter if we call this multiple times
    picint_register(INT_GPIO_PIN, pin_int);
#endif

    // This chip only supports edge triggered interrupts using two registers...
    //   Pos Edge   Neg Edge   Result
    //      0          0       Interrupt is disabled
    //      1          0       Interrupt on rising edge
    //      0          1       Interrupt on falling edge
    //      1          1       Interrupt on both edges
    // We calculate the bit masks required using pin_state and store it in
    // pin_ints[] ready for gpio_int_control() to use
    pos_edge = 0;
    neg_edge = 0;
    enable_handler = true;

    switch (pin_state & 0x7F)
    {
        case INT_ON_RISING_EDGE:
        case INT_WHEN_HIGH:
            pos_edge = mask;
            break;

        case INT_ON_FALLING_EDGE:
        case INT_WHEN_LOW:
            neg_edge = mask;
            break;

        case INT_ON_BOTH_EDGES:
            pos_edge = mask;
            neg_edge = mask;
            break;

        case INT_NEVER:
        default:
            // Interrupt not required
            enable_handler = false;
            break;
    }

    channel = pin_ints;

    // Search for this GPIO in the pin list
    for (i = 0; i < num_pin_ints; i++)
    {
        if (gpio_num == channel->gpio_num)
        {
            // Pin already configured - change settings
            channel->pos_edge = pos_edge;
            channel->neg_edge = neg_edge;
            channel->gpio_isr = enable_handler ? handler : NULL;
            return;
        }

        channel++;
    }

    if (enable_handler)
    {
        // Add a new pin to the list
        if (num_pin_ints < MAX_PIN_INTS)
        {
            channel = &pin_ints[num_pin_ints];

            channel->gpio_num = gpio_num;
            channel->pos_edge = pos_edge;
            channel->neg_edge = neg_edge;
            channel->mask = mask;
            channel->flag_reg = int_flag_regs[port];
            channel->input_reg = input_regs[port];
            channel->gpio_isr = handler;

            num_pin_ints++;
        }
    }
}


/**
 * @brief Enable/disable pin interrupts on a digital input pin
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @param[IN]  setting  - Required interrupt behaviour
 */
void gpio_int_control(uint16_t gpio_num, gpio_int_control_t setting)
{
    gpio_int_channel_t  *channel;
    uint8_t             port, i;

    if (gpio_num == GPIO_NOT_CONNECTED)
        return;

    port = (uint8_t)((gpio_num >> 8) & 0xFF);

    channel = pin_ints;

    // Search for this GPIO in the pin list
    for (i = 0; i < num_pin_ints; i++)
    {
        if (gpio_num == channel->gpio_num)
        {
            switch (setting)
            {
                case INT_ENABLE:
                    *int_pos_edge[port] |= channel->pos_edge;
                    *int_neg_edge[port] |= channel->neg_edge;
                    // At least one pin is enabled so enable all pin interrupts
                    IOCIE = 1;
                    break;

                case INT_DISABLE:
                    // This disables interrupts from an individual pin
                    // If all of them are disabled IOCIE has no effect
                    *int_pos_edge[port] &= ~channel->pos_edge;
                    *int_neg_edge[port] &= ~channel->neg_edge;
                    break;

                case INT_BLOCK:
                    // Unfortunately on the PIC we can't block individual pins
                    IOCIE = 0;
                    break;

                default:
                    break;
            }

            break;
        }

        channel++;
    }
}


/**
 * @brief Initialisation for the ADC subsystem, sets up clocks etc.
 *        Call this before using analog_input_get()
 * @param[OUT]  adc_bits  - ADC resolution in bits, NULL if not required
 */
void analog_input_init(uint8_t *adc_bits)
{
    if (adc_bits != NULL)
#ifdef _18F26Q10
        *adc_bits = 10;
#else
        *adc_bits = 12;
#endif

    // Turn on the ADC clocks
    ADCMD = 0;

    // Turn the ADC on, set the format and run it from the RC oscillator
    // so the conversion time is independent of the CPU frequency
    ADCON0 = 0x94;
    ADCON1 = 0;
    ADCON2 = 0;
}


// Performs an A/D conversion on the selected channel
static uint16_t read_adc(uint8_t channel)
{
    uint16_t  result;

    // Turn the ADC on, set the format and channel
    ADFM = 1;
    ADON = 1;
    ADPCH = channel;

    // Wait for acquisition
    timer_delay_us(25);

    // Kick off the conversion
    ADGO = 1;

    // Wait for the conversion to complete
    while (ADGO)
        ;

    // Read the result registers
    result = ((uint16_t)ADRESH << 8) | ADRESL;

    // Turn the A/D off to conserve power
    ADON = 0;

#ifdef _18F26Q10
    // Mask off undefined bits of ADRESH to give a 10-bit result
    result &= 0x03FF;
#else
    // Mask off undefined bits of ADRESH to give a 12-bit result
    result &= 0x0FFF;
#endif

    return result;
}


/* Helper function - checks gpio_num and decodes it into an ADC channel number
 * Returns true if the caller can continue
 */
static bool get_adc_channel_num(uint16_t gpio_num, uint8_t *channel)
{
    uint8_t  port, pin;

    port = (uint8_t)((gpio_num >> 8) & 0xFF);
    pin = (uint8_t)(gpio_num & 7);

    // These chips have all channels connected in a logical order
    if ((port <= 3) && (pin < 8))
    {
        *channel = (uint8_t)(port << 3) | pin;
        return true;
    }
    else
        // Not an ADC pin
        return false;
}


/**
 * @brief Read value of an analog input
 *        Pin must be configured as an analog input with gpio_configure_pin()
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port B, pin 2
 *
 *                        Except on the PIC18F26Q10 (no support)...
 *                        If the ADC_CHANNEL_NUM flag is specified, gpio_num is
 *                        interpreted as the ADC channel, instead of the GPIO
 *                        pin number.  The following values are supported:
 *                                0-8 : ADC channel 0-8, special values...
 *                                  0 : Temperature sensor low range
 *                                  1 : Temperature sensor high range
 *                                  2 : DAC1 output
 *                                  3 : Internal Vref (FVR1 x1 setting)
 *                                  4 : Internal Vref (FVR1 x2 setting)
 *                                  5 : Internal Vref (FVR1 x4 setting)
 *                                  6 : Internal Vref (FVR2 x1 setting) (Q43 chips only)
 *                                  7 : Internal Vref (FVR2 x2 setting) (Q43 chips only)
 *                                  8 : Internal Vref (FVR2 x4 setting) (Q43 chips only)
 *                        e.g. gpio_num = 3 | ADC_CHANNEL_NUM for FVR1 x1 reading
 *
 *                        In addition, the following 'channel numbers' can be
 *                        used to retrieve the factory calibration values...
 *                                136 : Temp sense TSLR1 value (Q43 chips only)
 *                                138 : Temp sense TSLR2 value
 *                                140 : Temp sense TSLR3 value (Q43 chips only)
 *                                142 : Temp sense TSHR1 value (Q43 chips only)
 *                                144 : Temp sense TSHR2 value
 *                                146 : Temp sense TSHR3 value (Q43 chips only)
 *                                148 : FVR1 x1 setting (mV)
 *                                150 : FVR1 x2 setting (mV)
 *                                152 : FVR1 x4 setting (mV)
 *                                154 : FVR2 x1 setting (mV)
 *                                156 : FVR2 x2 setting (mV)
 *                                158 : FVR2 x4 setting (mV)
 *                        e.g. gpio_num = 148 | ADC_CHANNEL_NUM for FVR1 x1 setting
 * @return ADC value
 */
uint32_t analog_input_get(uint16_t gpio_num)
{
    uint16_t  result;
    uint8_t   channel, fvrcon_reg;
    bool      fvrcon_changed;

    if (gpio_num == GPIO_NOT_CONNECTED)
        return 0;

    fvrcon_changed = false;

    if (gpio_num & ADC_CHANNEL_NUM)
    {
#ifdef _18F26Q10
        // ADC_CHANNEL_NUM not supported on the PIC18F26Q10
        return 0;
#endif

        // Channel number specified directly
        channel = (uint8_t)(gpio_num & ~ADC_CHANNEL_NUM);

        if (channel > 158)
            // Invalid selection
            return 0;
        else if (channel >= 100)
            // Calibration data from the Device Information Area (DIA)
            return (uint32_t)(*(uint16_t const *)(DIA_MUI + (channel - 100)));
        else if (channel > 8)
            // Invalid channel
            return 0;
        else if (channel >= 6)
        {
#if defined(_18F25K83) || defined(_18F26K83)
            // Can't read FVR2 on K83 chips
            return 0;
#endif
            fvrcon_reg = FVRCON;
            fvrcon_changed = true;

            // Turn on FVR2
            FVRCON = 0x80 | (uint8_t)((channel - 5) << 2);

            // Wait for it to be ready
            while ((FVRCON & 0x40) == 0)
                ;

            channel = 0x3F;
        }
        else if (channel >= 3)
        {
            fvrcon_reg = FVRCON;
            fvrcon_changed = true;

            // Turn on FVR1
            FVRCON = 0x80 | (uint8_t)(channel - 2);

            // Wait for it to be ready
            while ((FVRCON & 0x40) == 0)
                ;

#if defined(_18F25K83) || defined(_18F26K83)
            channel = 0x3F;
#else
            channel = 0x3E;
#endif
        }
        else if (channel >= 1)
        {
            fvrcon_reg = FVRCON;
            fvrcon_changed = true;

            // Turn on the temperature sensor
            FVRCON = 0xA0 | (uint8_t)((channel & 1) << 4);
            timer_delay_us(120);

#if defined(_18F25K83) || defined(_18F26K83)
            channel = 0x3D;
#else
            channel = 0x3C;
#endif
        }
    }
    else
    {
        // Decode GPIO port/pin to an ADC channel number
        if (!get_adc_channel_num(gpio_num, &channel))
            return 0;
    }

    // Do the conversion
    result = read_adc(channel);

    // If we set FVRCON, put it back to the original value
    if (fvrcon_changed)
    {
        FVRCON = fvrcon_reg;

        // If it was originally on and we have swapped to a different
        // setting, we need to wait for it to start up again
        if (fvrcon_reg & 0x80)
        {
            // Wait for it to be ready
            while ((FVRCON & 0x40) == 0)
                ;
        }
    }

    return (uint32_t)result;
}


/**
 * @brief Initialisation for the DAC subsystem, sets up clocks etc.
 *        Call this before using analog_output_set()
 * @param[OUT]  dac_bits  - DAC resolution in bits, NULL if not required
 */
void analog_output_init(uint8_t *dac_bits)
{
    uint8_t  num_bits;

    // Turn on the DAC clocks
    DAC1MD = 0;

    // Enable the DAC, voltage range is 0-Vdd
    // The output is connected when we know the required
    // pin number in analog_output_set()
#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q10)
    num_bits = 5;
    DAC1CON = 0x80;
#elif defined(_18F26Q43) || defined(_18F27Q43)
    num_bits = 8;
    DAC1CON = 0x80;
#else
    num_bits = 0;
#endif

    if (dac_bits != NULL)
        *dac_bits = num_bits;
}


/**
 * @brief Write value of an analog output
 *        Pin must be configured as an analog output with gpio_configure_pin()
 * @param[IN]  gpio_num - Encoded port and pin e.g. 0x0102 is port B, pin 2
 * @param[IN]  value    - DAC value to set
 */
void analog_output_set(uint16_t gpio_num, uint32_t value)
{
    if (gpio_num == GPIO_NOT_CONNECTED)
        return;

#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q10)
    value &= 0x1F;
#endif

    // Set the DAC value before enabling the output
    DAC1DATL = (uint8_t)value;

    // DAC can only output on RA2 or RB7
    if (gpio_num == 0x0002)
        DAC1CON = 0xA0;
    else if (gpio_num == 0x0107)
        DAC1CON = 0x90;
    else
        DAC1CON = 0x80;
}

