/**
 * @brief  PIC18 HAL top level chip driver
 *
 * Copyright Debug Innovations Ltd. 2018-2024
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "lf_pics.h"
#include "pic_interrupt.h"
#include "hal_config.h"
#include "hal_gpio.h"
#include "hal_chip.h"


// Deal with bit name differences
#if defined(_18F26Q10)

#define U1MD  UART1MD
#define U2MD  UART2MD

#endif


// Declared in hal_gpio.c
extern volatile uint8_t * const pps_outb_regs[];
extern volatile uint8_t * const pps_outc_regs[];


#if defined(_18F25K83) || defined(_18F26K83)
  #define PPS_CLKR  0x27
#endif

#ifdef _18F26Q10
  #define PPS_CLKR  0x14
#endif

#if defined(_18F26Q43) || defined(_18F27Q43)
  #define PPS_CLKR  0x42
#endif


// Maximum number of registered event callbacks for each type of event
#define MAX_EVENT_SUBSCRIBERS  6

// List of subscribers to each event
static chip_event_callback_t   events[NUM_CHIP_EVENT_TYPES][MAX_EVENT_SUBSCRIBERS];
static uint8_t                 num_subscribers[NUM_CHIP_EVENT_TYPES];

static uint32_t  current_cpu_clock_Hz = 0;
static uint16_t  current_cpu_clock_gpio = GPIO_NOT_CONNECTED;
static bool      watchdog_on = false;
static bool      watchdog_running = false;

// Stored registers during sleep
static uint8_t   wdt_reg;


#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)

// Default interrupt handler
static void __interrupt(irq(default)) default_isr(void)
{
    volatile uint8_t  int_src;

    // Unhandled interrupts, source indicated by W reg
    int_src = WREG;
    NOP();
}

#endif


/*
 * @brief Initialise the chip clocks etc.
 *        This also sets the default CPU clock frequency
 */
void chip_init(void)
{
    uint8_t  i;

#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q10) || defined(_18F26Q43) || defined(_18F27Q43)

    // Default 4MHz
    current_cpu_clock_Hz = 4000000UL;

    // Select internal HF oscillator
    OSCCON1 = 0x60;

    // 4MHz setting
    OSCFRQ = 2;

    // In these chips, all the peripheral clocks are enabled by default
    // To save power, we turn them off - the user can re-enable them if needed

    // Turn off clock reference generator
    CLKRMD = 1;

    // Turn off all the timers except Timer 0
    PMD1 = 0xFE;

    // Turn off all the other peripherals
    PMD3 = 0xFF;
    PMD4 = 0xFF;
    PMD5 = 0xFF;

#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)
    PMD6 = 0xFF;
    PMD7 = 0xFF;
#endif

#if defined(_18F26Q43) || defined(_18F27Q43)
    PMD8 = 0xFF;
#endif

#ifdef _18F26Q10
    picint_init();
#endif

#else
  #error HAL: Unsupported PIC type
#endif

    for (i = 0; i < NUM_CHIP_EVENT_TYPES; i++)
        num_subscribers[i] = 0;
}


/*
 * @brief Enables all CPU interrupts
 *        Call this after all interrupt initialisation is complete including
 *        HAL services chip_init(), timer_init(), gpio_init() and uart_init().
 *        Note that timers and UARTs, trace etc. won't work until after this
 *        function is called.  There is no chip_disable_interrupts() because
 *        this is part of the HAL setup and not intended for controlling
 *        individual hardware components.
 */
void chip_enable_interrupts(void)
{
#ifdef _18F26Q10
    PEIE = 1;
#endif

    GIE = 1;
}


/*
 * @brief Register a callback function for a chip_sleep() or chip_reset() call.
 *        The function will be called to clean up before sleeping or resetting
 *        the CPU e.g. to turn something off, flush a cache or issue a warning.
 *        Events cannot be removed once registered.
 * @param[in]   chip_event    The event associated with the callback
 * @param[in]   callback      The function to call
 */
void chip_register_event_callback(chip_event_t chip_event, chip_event_callback_t callback)
{
    uint8_t  i;

    if (num_subscribers[chip_event] < MAX_EVENT_SUBSCRIBERS)
    {
        for (i = 0; i < num_subscribers[chip_event]; i++)
        {
            if (events[chip_event][i] == callback)
                // Already registered
                return;
        }
        events[chip_event][num_subscribers[chip_event]++] = callback;
    }
}


// Calls back registered observers for the specified event
static void notify_subscribers(chip_event_t chip_event)
{
    uint8_t  i;

    for (i = 0; i < num_subscribers[chip_event]; i++)
        events[chip_event][i]();
}


/*
 * @brief Set the CPU clock frequency
 */
void chip_set_cpu_clock(uint32_t cpu_clock_Hz, clock_source_t source)
{
    if ((source == CLOCK_SOURCE_CRYSTAL) || (source == CLOCK_SOURCE_EXTERNAL))
    {
        uint32_t  fosc, fdiv;
        int32_t   error, max_error;
        uint8_t   div, best_div;
        bool      can_use_pll, use_pll;

        // Use external crystal or clock input
        // If your circuit doesn't need/use an external crystal,
        // then add the following line to your hal_config.h:
        // #define CPU_CLOCK_CRYSTAL_FREQ_HZ  0
        fosc = CPU_CLOCK_CRYSTAL_FREQ_HZ;

        // 4xPLL has an input range of 4-16MHz
        can_use_pll = ((fosc >= 4000000UL) && (fosc <= 16000000UL));

        // Use the PLL if the requested frequency is higher than the crystal
        use_pll = can_use_pll && (cpu_clock_Hz > fosc);

        if (use_pll)
            fosc *= 4;

        // Starting with fosc, find the divider closest to the requested frequency
        best_div = 0;
        max_error = (int32_t)fosc;

        for (div = 0; div <= 9; div++)
        {
            fdiv = fosc >> div;

            error = labs((int32_t)fdiv - (int32_t)cpu_clock_Hz);
            if (error < max_error)
            {
                max_error = error;
                best_div = div;
            }

            if (fdiv < cpu_clock_Hz)
                break;
        }

        OSCCON1 = (use_pll ? 0x20 : 0x70) | best_div;
    }
    else
    {
        // Use internal HF oscillator
        OSCCON1 = 0x60;

        switch (cpu_clock_Hz)
        {
            case 0:
                // Slowest possible clock (1MHz)
                // TODO: Use alternative slower clock

            case 1000000:
                // 1MHz
                OSCFRQ = 0;
                break;

            case 2000000:
                // 2MHz
                OSCFRQ = 1;
                break;

            case 4000000:
                // 4MHz
                OSCFRQ = 2;
                break;

            case 8000000:
                // 8MHz
                OSCFRQ = 3;
                break;

            case 12000000:
                // 12MHz
                OSCFRQ = 4;
                break;

            case 16000000:
                // 16MHz
                OSCFRQ = 5;
                break;

            case 32000000:
                // 32MHz
                OSCFRQ = 6;
                break;

            case 48000000:
                // 48MHz
                OSCFRQ = 7;
                break;

            case 64000000:
                // 64MHz
                OSCFRQ = 8;
                break;

            default:
                return;
        }
    }

    current_cpu_clock_Hz = cpu_clock_Hz;
}


/*
 * @brief Get the CPU clock frequency
 * @returns Clock frequency in Hz
 */
uint32_t chip_get_cpu_clock(void)
{
    return current_cpu_clock_Hz;
}


/*
 * @brief Output the CPU clock on a GPIO pin
 * @param[in]   gpio_num   The GPIO number to output the clock to.
 *                         Use GPIO_NOT_CONNECTED to disable.
 * @param[in]   divider    Divider value for the clock
 */
void chip_output_cpu_clock(uint16_t gpio_num, uint32_t divider)
{
    uint8_t  port, pin, mask, div, reg;

    if (gpio_num != GPIO_NOT_CONNECTED)
    {
        port = (uint8_t)((gpio_num >> 8) & 0xFF);
        pin = (uint8_t)(gpio_num & 0xFF);

        // On this chip, the CPU clock can only be output on ports B & C
        if (!((port == 1) || (port == 2)))
            return;

        // The supported divider values are 1-128 in binary steps
        if (divider > 128)
            divider = 128;

        div = (uint8_t)divider;

        if (div == 0)
            reg = 0;
        else
        {
            reg = 7;
            mask = 0x80;

            // Take the highest bit as the divider
            while ((div & mask) == 0)
            {
                mask >>= 1;
                reg--;
            }
        }

        // We need to configure the pin to start with
        gpio_configure_pin(gpio_num, PIN_DIGITAL_OUT | PIN_HIGH_SPEED | PIN_SET_LOW);

        // Turn on the clock generator
        CLKRMD = 0;
        CLKRCON = reg | 0x90;

        // Connect the pin to the clock generator
        if (port == 1)
            // Port B PPS
            *pps_outb_regs[pin] = PPS_CLKR;
        else if (port == 2)
            // Port C PPS
            *pps_outc_regs[pin] = PPS_CLKR;
    }
    else
    {
        // A gpio_num of GPIO_NOT_CONNECTED unhooks the current pin setting
        // which is a function gpio_configure_pin() already does
        gpio_configure_pin(current_cpu_clock_gpio, PIN_DIGITAL_OUT | PIN_SET_LOW);

        // Turn off clock reference generator
        CLKRCON = 0x10;
        CLKRMD = 1;
    }

    current_cpu_clock_gpio = gpio_num;
}


/*
 * @brief Cause a hardware reset of the chip
 *        'type' is implementation defined, however calls from interrupt context such
 *        as a hard fault or watchdog timeout should use -1 to indicate a 'crash'
 */
void chip_reset(int type)
{
    // If type is -1, it was caused by some kind of catastrophe like a hard
    // fault or watchdog interrupt, we may still be in interrupt context and
    // we can't assume timers and comms are working, therefore we shouldn't
    // call the user's code or it may get stuck in a loop.
    if (type != -1)
        notify_subscribers(CHIP_EVENT_RESET);

    RESET();
}


/*
 * @brief Call this from an interrupt routine to get the system into a functional state.
 *        If the interrupt woke the system from sleep this will do the post-sleep processing.
 *        If the system is already awake, this does nothing.
 */
void chip_interrupt_wakeup(void)
{
    // Restore original WDTCON0 value
    WDTCON0 = wdt_reg;
    CLRWDT();

    notify_subscribers(CHIP_EVENT_WAKE);
}


// This does the opposite of chip_interrupt_wakeup()
static void pre_sleep_tasks(void)
{
    notify_subscribers(CHIP_EVENT_SLEEP);

    // Remember the original watchdog timer value
    wdt_reg = WDTCON0;
}


/*
 * @brief Enter low power mode
 *
 * @param[in]   max_time_ms   Max sleep time in ms
 * @returns -1 if the chip slept for the full period or
 *          the GPIO number if a pin interrupt happened
 */
int chip_sleep(uint32_t max_time_ms)
{
    uint32_t  mask;
    uint8_t   setting;

    pre_sleep_tasks();

    // Set WDT to 256s
    WDTCON0 = 0x25;

    // If the sleep time is >=512 seconds, the hardware can't
    // time it so we have to sleep 256s at a time until the
    // time remaining is <512s
    while (max_time_ms >= 512000)
    {
        SLEEP();
        max_time_ms -= 256000;
    }

    // Starting with the highest (256s) watchdog setting and
    // the bit corresponding to 256s in max_time_ms, sleep
    // for the time required for each set bit in max_time_ms
    setting = 18;
    mask = 1UL << setting;

    while (mask != 0)
    {
        if (max_time_ms & mask)
        {
            WDTCON0 = (uint8_t)(setting << 1) | 1;
            SLEEP();
        }

        mask >>= 1;
        setting--;
    }

    chip_interrupt_wakeup();

    return -1;
}


/*
 * @brief Initialise the watchdog hardware
 *
 * @param[in]   period_ms   Watchdog period in ms.  A value of zero disables the watchdog.
 * @param[in]   timeout     Function to be called after period_ms if the watchdog is not kicked
 *                          Ignored: The PIC doesn't support calling a timeout function, a
 *                                   hardware reset occurs
 */
void watchdog_init(uint32_t period_ms, const fp_watchdog_t timeout)
{
    uint8_t   setting;
    uint32_t  timeout_ms;

    // Unused parameter
    (void)timeout;

    if (period_ms == 0)
    {
        // Turn watchdog off
        WDTCON0 = 0;
        watchdog_on = false;
    }
    else
    {
        // Turn watchdog on
        // Find the closest setting above period_ms that the PIC
        // can use (min 1ms, max 256s)
        timeout_ms = 1;
        for (setting = 0; setting <= 18; setting++)
        {
            if ((timeout_ms > period_ms) || (setting == 18))
            {
                WDTCON0 = (uint8_t)(setting << 1);
                break;
            }

            timeout_ms <<= 1;
        }
        watchdog_on = true;
    }

    watchdog_running = false;
}


/*
 * @brief Kick (reset) the watchdog
 *        The watchdog starts running the first time it is kicked.  From then on, it must be
 *        kicked continually.  If not kicked for period_ms (see watchdog_init above), the PIC
 *        is reset.  The PIC doesn't support calling a timeout function.
 */
void watchdog_kick(void)
{
    if (watchdog_on && !watchdog_running)
    {
        // First kick, start watchdog running
        WDTCON0 |= 1;
        watchdog_running = true;
    }

    CLRWDT();
}

