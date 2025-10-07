/*
 * Embedded code hardware abstraction layer API
 *    - Top level CPU and chip control functions
 *
 * The following #defines should be in hal_config.h...
 *
 * #define CPU_CLOCK_CRYSTAL_FREQ_HZ   CPU clock crystal frequency
 *                                     (zero if there is no external crystal)
 *
 * Copyright Debug Innovations Ltd. 2016-2023
 */

#ifndef HAL_CHIP_H
#define HAL_CHIP_H

#include <stdint.h>
#include <stdbool.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// Max. size of a text report from chip_get_core_dump()
#define DUMP_BUFF_SIZE  256


// Clock sources
enum clock_source_type
{
    CLOCK_SOURCE_AUTO,
    CLOCK_SOURCE_INTERNAL,
    CLOCK_SOURCE_EXTERNAL,
    CLOCK_SOURCE_CRYSTAL,
    NUM_CLOCK_SOURCE_TYPES
};
typedef enum clock_source_type  clock_source_t;


// Chip-level event types
enum chip_event_type
{
    CHIP_EVENT_RESET,
    CHIP_EVENT_SLEEP,
    CHIP_EVENT_WAKE,
    NUM_CHIP_EVENT_TYPES
};
typedef enum chip_event_type  chip_event_t;


/**
 * @brief Definition of a callback function for the watchdog.
 *        Watchdogs with more than one level e.g. warning/shutdown
 *        should use the type parameter to indicate the requested
 *        action the handler should take (system specific meaning).
 */
typedef void (*fp_watchdog_t)(uint32_t type);


/**
 * @brief Definition of a callback function for chip level events
 */
typedef void (*chip_event_callback_t)(void);


/*
 * @brief Call this first, initialises the chip clocks etc.
 *        This also sets the default CPU clock frequency
 */
void chip_init(void);


/*
 * @brief Enables all CPU interrupts
 *        Call this after all interrupt initialisation is complete including
 *        HAL services chip_init(), timer_init(), gpio_init() and uart_init().
 *        Note that timers and UARTs, trace etc. won't work until after this
 *        function is called.  There is no chip_disable_interrupts() because
 *        this is part of the HAL setup and not intended for controlling
 *        individual hardware components.
 */
void chip_enable_interrupts(void);


/*
 * @brief Register a callback function for a chip_sleep() or chip_reset() call.
 *        The function will be called to clean up before sleeping or resetting
 *        the CPU e.g. to turn something off, flush a cache or issue a warning.
 *        Events cannot be removed once registered.
 * @param[in]   chip_event    The event associated with the callback
 * @param[in]   callback      The function to call
 */
void chip_register_event_callback(chip_event_t chip_event, chip_event_callback_t callback);


/*
 * @brief Cause a hardware reset of the chip
 *        'type' is implementation defined, however calls from interrupt context such
 *        as a hard fault or watchdog timeout should use -1 to indicate a 'crash'
 */
void chip_reset(int type);


/*
 * @brief Enter low power mode
 *
 * @param[in]   max_time_ms   Max sleep time in ms
 * @returns -1 if the chip slept for the full period or
 *          the GPIO number if a pin interrupt happened
 */
int chip_sleep(uint32_t max_time_ms);


/*
 * @brief Retrieve a core dump report
 * @param[out]  pc           The program counter when the fault occurred, NULL if not required
 * @param[out]  error_text   Dump text for the user, NULL if not required
 * @returns 0 for no error, other values indicate the reason for the dump
 */
int chip_get_core_dump(uint32_t *pc, char error_text[DUMP_BUFF_SIZE]);


/*
 * @brief Set the CPU clock frequency
 *        'source' may not be supported by the implementation
 *        set to CLOCK_SOURCE_AUTO if unsure
 */
void chip_set_cpu_clock(uint32_t cpu_clock_Hz, clock_source_t source);


/*
 * @brief Get the CPU clock frequency
 * @returns Clock frequency in Hz
 */
uint32_t chip_get_cpu_clock(void);


/*
 * @brief Output the CPU clock on a GPIO pin
 * @param[in]   gpio_num   The GPIO number to output the clock to.
 *                         Use GPIO_NOT_CONNECTED to disable.
 * @param[in]   divider    Divider value for the clock
 */
void chip_output_cpu_clock(uint16_t gpio_num, uint32_t divider);


/*
 * @brief Call this from an interrupt routine to get the system into a functional state.
 *        If the interrupt woke the system from sleep this will do the post-sleep processing.
 *        If the system is already awake, this does nothing.
 */
void chip_interrupt_wakeup(void);


/*
 * @brief Initialise the watchdog hardware
 *
 * @param[in]   period_ms   Watchdog period in ms.  A value of zero disables the watchdog.
 * @param[in]   timeout     Function to be called after period_ms if the watchdog is not kicked
 */
void watchdog_init(uint32_t period_ms, const fp_watchdog_t timeout);


/*
 * @brief Kick (reset) the watchdog
 *        The watchdog starts running the first time it is kicked.  From then on, it must be
 *        kicked continually.  If not kicked for period_ms, the timeout function will be called
 *        (see watchdog_init above) which is expected to log it and/or reset the chip (or similar).
 */
void watchdog_kick(void);


#ifdef __cplusplus
}
#endif

#endif

