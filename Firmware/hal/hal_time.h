/*
 * Embedded code hardware abstraction layer API
 *    - Timer and RTC functions
 *
 * RTC Options:
 * You must put one of the following #defines in you project's hal_config.h...
 *
 * #define USE_HARDWARE_RTC     Use the CPU's RTC hardware for the clock
 *                              (on an emulation, this will be the PC clock)
 * #define USE_SOFTWARE_RTC     Use a software RTC for the clock
 *                              (a simple counter that starts at zero every reset)
 * #define USE_EXTERNAL_RTC     The hal_time RTC functions will not be compiled.
 *                              The user must provide an implementation (typically
 *                              by using a driver for an external RTC chip).
 * #define RTC_NOT_REQUIRED     The hal_time RTC functions will not be compiled.
 *                              Attempts to use the RTC will result in linker errors.
 *                              This setting will also #define TRACE_TIME_DISABLE.
 *
 * In addition, the following #defines can be added...
 *
 * #define USE_PC_RTC_IN_EMUL   On a product that uses the software RTC, adding this
 *                              #define causes an emulation to use the PC clock and
 *                              an embedded build to use the selected RTC.
 * #define TRACE_TIME_DISABLE   See trace.h
 *
 * Not all devices have a hardware RTC e.g. PICs.  When building for these devices,
 * the above #defines apply primarily to emulation builds.  If the product requires
 * RTC functionality, use a software RTC or an external RTC.
 *
 * On an embedded platform, a hardware RTC will continue to run when the CPU is
 * stopped (in sleep mode) whereas the software RTC will stop.  If the platform has
 * a battery for the RTC, the RTC will run with the power off whereas the software
 * RTC will start at zero every reset.  For products which are left powered on and
 * don't use sleep modes, the software RTC requires no extra crystal or battery.
 * The resolution of the RTC is platform-dependent - some platforms have millisecond
 * resolution, others only seconds.  The software RTC has millisecond resolution.
 *
 * On an emulation, if the PC clock is used, attempts to set it with rtc_set_ticks()
 * are ignored.  If the software RTC is used, it will need to be set to the correct
 * time every run and an attempt to set the incorrect time (compared to the PC's
 * clock) will show a warning message.
 *
 * Copyright Debug Innovations Ltd. 2016-2023
 */

#ifndef HAL_TIME_H
#define HAL_TIME_H

#include <stdint.h>
#include <stdbool.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


/*
 * Unfortunately, linux's time.h has a timer_create() and timer_delete()
 * so we have to hide our versions to make it build on linux as time.h
 * is commonly included when using timer functions.  The same applies to
 * V2+ of the XC8 PIC compiler which supports C99.
 */
#if defined(linux) || defined(_XOPEN_SOURCE)
#define timer_create  tim_create
#define timer_delete  tim_delete
#endif


// timegm() isn't available on some embedded platforms and has a different
// name on Windows - we use make_gmtime() wherever we need timegm()
#ifdef _MSC_VER
// Windows version of timegm()
#define make_gmtime  _mkgmtime
#elif defined(linux)
// Linux version of timegm()
#define make_gmtime  timegm
#else
// Most embedded platforms will have to use mktime()
// This may include conversion to the local time zone
#define make_gmtime  mktime
#endif


// Definitions to scale millisecond times to natural units
#define MSECS                (1UL)
#define SECS              (1000UL)
#define MINS             (60000UL)
#define HOURS          (3600000UL)
#define DAYS          (86400000UL)


/**
 * @brief Definition of a callback function for software timers
 */
typedef void (*fp_timer_t)(void *params);


/**
 * @brief Initialise the timer hardware and software timers
 *
 * @param[in]   expired   Pointer to a variable that is set to true when a timer needs servicing.
 *                        WARNING: Must be a static global as it will be set by an interrupt
 *                        - see timer_service_callbacks() below.
 */
void timer_init(volatile bool *const expired);


/**
 * @brief Register a function that is called from interrupt context every milli-second.
 *        Not all platforms support software timers and their callbacks rely on the foreground
 *        thread calling timer_service_callbacks(), so a 1ms software timer is not very reliable.
 *        In applications such as sampling a signal every millisecond, this callback can be used.
 *        Multiple callbacks can be registered up to a platform-specific limit but the total
 *        execution time of all callbacks must be much less than 1ms.  Once added, callbacks
 *        cannot be removed.
 *
 * @param[in]   callback    Function to be called every milli-second
 * @param[in]   params      Parameter that will be passed to the callback function (use 'this' for C++)
 *
 * @returns true for success, false if not supported or there are too many functions registered
 */
bool timer_register_1ms_callback(const fp_timer_t callback, void *params);


/**
 * @brief Calls all the timer callback functions that need servicing (their timers have expired).
 *        The main program loop is expected to call this function whenever the variable 'expired'
 *        is true and it will set 'expired' to false when finished - see timer_init() above.
 */
void timer_service_callbacks(void);


/**
 * @brief Delay for the specified number of milliseconds (blocking)
 *
 * @param[in]   delay_ms    Time to delay in ms
 */
void timer_delay(uint32_t delay_ms);


/**
 * @brief Delay for the specified number of microseconds (blocking)
 *        Unlike timer_delay(), there is no guarantee this function gives
 *        the same delay on different platforms.  It may just call
 *        timer_delay() or count in a loop.  Use it with caution.
 *
 * @param[in]   delay_us    Time to delay in us
 */
void timer_delay_us(uint32_t delay_us);


/*
 * @brief Gets the total number of milliseconds since reset.
 *        The time wraps back to zero after 49.7 days.
 */
uint32_t timer_get_run_time(void);


/*
 * @brief Returns the time remaining to a target time based on timer_get_run_time()
 *
 * @param[in]   target_ms   Target time in ms
 *
 * @returns Number of milliseconds left before the target time
 *          A value of zero indicates that the target time has been reached
 *          A negative value indicates that the target time has been passed
 */
int32_t timer_run_time_remaining(uint32_t target_ms);


/*
 * @brief Gets the total number of microseconds since reset.
 *        This high resolution timer is not supported on all
 *        platforms and it may have <1us resolution on others
 *        i.e. increment more than 1us at a time.
 *        Simple 64-bit microsecond counter implementations
 *        never wrap (wrap time of half a million years) but
 *        other implementations may have different behaviour.
 *        The Windows PC emulation never wraps.
 */
uint64_t timer_get_run_time_us(void);


/**
 * @brief Creates a new software timer
 *
 * @param[in]   period_ms   Timer period in milliseconds
 * @param[in]   callback    Function to be called after the specified period
 * @param[in]   params      Parameter that will be passed to the callback function (use 'this' for C++)
 * @param[in]   start       Whether to start the timer after creating
 * @param[in]   one_shot    If true, the timer will be deleted after calling the callback function
 *                          If false, the timer will be reset and trigger again after the specified period
 *
 * @return A new timer_id if the timer was successfully created or -1 if not successful.
 *         A valid timer_id is non-zero, positive and is different on each call even if the timer
 *         has been deleted and re-created.  This required behaviour prevents accidental deletion
 *         of a timer belonging to another piece of code due to bad timing.
 */
int timer_create(uint32_t period_ms, const fp_timer_t callback, void *params, bool start, bool one_shot);


/**
 * @brief Deletes a software timer
 *        Ignored if the timer doesn't exist or has already been deleted
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_delete(int timer_id);


/**
 * @brief Resets the time elapsed for a software timer
 *        If the timer is stopped, it will start from zero when it is started
 *        Ignored if the timer doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_reset(int timer_id);


/**
 * @brief Reschedules a software timer then resets it
 *        If the timer is stopped, the new setting will be used when it is started
 *        Ignored if the timer doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 * @param[in]   period_ms   New timer period in milliseconds
 */
void timer_reschedule(int timer_id, uint32_t period_ms);


/**
 * @brief Starts a stopped software timer
 *        Ignored if the timer is already running or doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_start(int timer_id);


/**
 * @brief Stops a running software timer
 *        Ignored if the timer is already stopped or doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_stop(int timer_id);


/**
 * @brief Causes a software timer to finish and set the 'expired' flag as if
 *        it had run for the full time period.  timer_service_callbacks() will
 *        then call its callback function.  This function can be used to force
 *        a callback to a foreground function from an interrupt.
 *        Ignored if the timer is stopped or doesn't exist.
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_finish(int timer_id);


/**
 * @brief Returns the time left to run on a software timer in milliseconds
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
uint32_t timer_time_left(int timer_id);


/**
 * @brief Starts a hardware timer which interrupts and sets timed_out after period_us.
 *        There is only one shared timer, the application must ensure no overlaps occur.
 *        Each call resets the timer so it is not necessary to stop the timer but the
 *        pointer to timed_out must remain valid at all times as it may be written after
 *        the caller has finished with the timer.
 *
 * @param[in]   period_us   Timeout period in microseconds, max. value is platform-specific.
 * @param[in]   timed_out   Pointer to a variable that is set to true after period_us.
 *                          WARNING: Must be a static global as it will be set by an interrupt.
 */
void timer_set_timeout(uint32_t period_us, volatile bool *const timed_out);


/**
 * @brief Initialise the real-time clock hardware (or software)
 *        This should be called before using other RTC functions
 */
void rtc_init_hardware(void);


/**
 * @brief Gets the value of the real time clock hardware in seconds and milliseconds
 *        'secs' is designed to be compatible with the 32-bit time_t type
 *
 * @param[out]  secs   The number of seconds since the start of time (user defined)
 * @param[out]  ms     The number of milliseconds as a fraction of the second (if supported by hardware)
 *                     Set this to NULL if milliseconds are not required
 */
void rtc_get_ticks(uint32_t *secs, uint16_t *ms);


/**
 * @brief Sets the value of the real time clock hardware in seconds and milliseconds
 *        'secs' is designed to be compatible with the 32-bit time_t type
 *
 * @param[in]   secs   The number of seconds since the start of time (user defined)
 * @param[in]   ms     The number of milliseconds as a fraction of the second (if supported by hardware)
 */
void rtc_set_ticks(uint32_t secs, uint16_t ms);


#ifdef __cplusplus
}
#endif

#endif

