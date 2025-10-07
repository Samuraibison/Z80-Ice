/**
 * @brief  PIC18 implementation of HAL timers (including support for software timers)
 *         Uses Timer 0 for the main 1ms tick and Timer 3 & 5 for the timeout timer.
 *         Note: Relies on GIE being set, timer_set_timeout() relies on PEIE
 *
 * Copyright Debug Innovations Ltd. 2018-2023
 */

// Suppress some annoying XC8 compiler warnings
#pragma warning disable 373   // Signed to unsigned conversions
#pragma warning disable 520   // Uncalled functions

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include "lf_pics.h"
#include "hal_config.h"
#include "hal_chip.h"
#include "pic_interrupt.h"
#include "hal_time.h"


#define MAX_SOFTWARE_TIMERS   12


// Counter for 1ms system ticks
static volatile uint32_t  ms_ticks;

static volatile bool  *timer_expired;
static volatile bool  *timer_timed_out;

static uint8_t        tmr0_period, tmr0_reload, tmr0_tick_us, last_tmr0_ticks;
static uint32_t       last_delay_us;

// Software RTC
// If USE_EXTERNAL_RTC is defined, none of the RTC functions will be built
// and the user can provide his own versions to access an RTC chip.
// If USE_SOFTWARE_RTC is defined, a software alternative will be used.
// The software RTC adds to the interrupt service time.
#ifdef USE_SOFTWARE_RTC
static volatile uint64_t  rtc_counter;
#endif

// Support for software timers
struct software_timer
{
    uint32_t    period_ms;
    uint32_t    time_left_ms;
    fp_timer_t  callback_fn;
    void        *callback_params;
    uint8_t     timer_id_qual;
    bool        running;
    bool        one_shot;
};
typedef struct software_timer  software_timer_t;

// Increments each time we create a timer and becomes part of the timer_id
static uint8_t  timer_id_qualifier;

// List of software timers and a pointer to the last + 1 active timer
static software_timer_t  sw_timers[MAX_SOFTWARE_TIMERS];
static software_timer_t  *last_timer;


// Timer0 interrupt routine
static void tmr0_int(void)
{
    software_timer_t  *timer;

    // Timer0 is automatically reloaded on PIC18s

    // Increment main millisecond counter
    ms_ticks++;

#ifdef USE_SOFTWARE_RTC
    // Software RTC
    rtc_counter++;
#endif

    // Decrement the software timers
    timer = sw_timers;

    while (timer < last_timer)
    {
        if (timer->running)
        {
            // Reduce the time left
            if (timer->time_left_ms != 0)
            {
                if (--timer->time_left_ms == 0)
                    *timer_expired = true;
            }
        }
        timer++;
    }
}


#if defined(_18F25K83) || defined(_18F26K83) || defined(_18F26Q43) || defined(_18F27Q43)

// Timer0 interrupt entry point
static void __interrupt(irq(IRQ_TMR0)) timer0_isr(void)
{
    tmr0_int();
    TMR0IF = 0;
}

#endif


#ifdef _18F26Q10

// Timer3/5 interrupt routine
static void tmr5_int(void)
{
    *timer_timed_out = true;
    T3CONbits.ON = 0;
}

#else

// Timer3/5 interrupt routine for vectored interrupts
static void __interrupt(irq(IRQ_TMR5)) timer5_isr(void)
{
    *timer_timed_out = true;
    T3CONbits.ON = 0;
    TMR5IF = 0;
}

#endif


/**
 * @brief Initialise the timer hardware and software timers
 *
 * @param[in]   expired   Pointer to a variable that is set to true when a timer needs servicing.
 *                        WARNING: Must be a static global as it will be set by an interrupt
 *                        - see timer_service_callbacks() below.
 */
void timer_init(volatile bool *const expired)
{
    uint32_t  timer_clk;
    uint8_t   prescaler;

    // Check the input parameters
    if (expired == NULL)
        return;

    timer_expired = expired;
    *timer_expired = false;

    last_delay_us = 0;
    last_tmr0_ticks = 0;
    ms_ticks = 0;

    memset(sw_timers, 0, sizeof(sw_timers));
    timer_id_qualifier = 0;
    last_timer = sw_timers;

    // Work out the timer clock rate, which is the instruction clock
    // rate (CPU clock / 4) divided by the minimum pre-scaler setting
    timer_clk = chip_get_cpu_clock();

    // Minimum pre-scaler is 1:1
    timer_clk >>= 2;

    // Calculate the pre-scaler needed to get a 1ms system tick
    timer_clk /= 1000;
    prescaler = 0;

    while (timer_clk > 256)
    {
        timer_clk >>= 1;
        prescaler++;
    }

    // Calculate the reload value for Timer 0 and
    // the microseconds per tick for timer_delay_us()
    tmr0_period = (uint8_t)timer_clk;
    tmr0_reload = (uint8_t)0 - tmr0_period;
    tmr0_tick_us = (uint8_t)(1000 / timer_clk);

    // Avoid division by zero errors
    if (tmr0_tick_us == 0)
        tmr0_tick_us = 1;

    // Load Timer0 prescaler
    T0CON1 = (prescaler & 0xF) | 0x40;
    T0EN = 1;

    // Start Timer 0
    TMR0H = tmr0_period - 1;
    TMR0L = 0;

#ifdef _18F26Q10
    picint_register(INT_TIMER0, tmr0_int);
#else
    TMR0IF = 0;
    TMR0IE = 1;
#endif

    // Setup TIMER3 and TIMER5 as a 32-bit cascaded counter
    // running at 1MHz for the timer_set_timeout()
    TMR3MD = 0;
    TMR5MD = 0;

    // We have a choice of CLK or CLK/4 and a pre-scaler of up to 8:1
    timer_clk = chip_get_cpu_clock();

    if (timer_clk <= 4000000)
    {
        // Run from CLK
        T3CLK = 2;

        if (timer_clk <= 1000000)
            // 1:1
            prescaler = 0;
        else if (timer_clk <= 2000000)
            // 2:1
            prescaler = 1;
        else
            // 4:1
            prescaler = 2;
    }
    else
    {
        // Run from CLK/4
        T3CLK = 1;

        if (timer_clk <= 8000000)
            // 2:1
            prescaler = 1;
        else if (timer_clk <= 16000000)
            // 4:1
            prescaler = 2;
        else
            // 8:1
            // For CPU clocks > 32MHz, we can only use 0.5us ticks
            prescaler = 3;
    }

    // This will also stop TIMER3 if it is running
    T3CON = (uint8_t)(prescaler << 4) | 6;

    // TIMER5 is set up in timer_set_timeout()
    // Simply load a default clock and stop it for now
    T5CON = 6;
    T5CLK = 1;

#ifdef _18F26Q10
    picint_register(INT_TIMER5, tmr5_int);
#else
    TMR5IF = 0;
    TMR5IE = 1;
#endif
}


/**
 * @brief Delay for the specified number of milliseconds (blocking)
 *
 * @param[in]   delay_ms    Time to delay in ms
 */
void timer_delay(uint32_t delay_ms)
{
    uint32_t  now, end_time;

    if (delay_ms == 0)
        return;

    // Since ms_ticks is 32-bit, reading it is a non-atomic operation on a PIC,
    // so we have to disable the timer interrupt around this read
    TMR0IE = 0;
    end_time = ms_ticks;
    TMR0IE = 1;

    // We wait at least delay_ms by adding 1ms
    // Average delay is 0.5ms longer than requested
    end_time += delay_ms + 1;

    // Now loop until the required number of ticks passes.  The second term stops the loop
    // terminating just before ms_ticks rolls over when end_time is past zero.
    // In principle, this loop should spin so fast that ms_ticks can't overrun end_time
    // if no roll over occurs.  However interrupts may happen during the loop so we have
    // to allow some overrun e.g. 10ms.  Making it 10s allows for really badly behaved
    // interrupts and allows stepping through the code when debugging (if the debugger
    // supports it).
    do
    {
        TMR0IE = 0;
        now = ms_ticks;
        TMR0IE = 1;
    }
    while ((now < end_time) || ((now - end_time) > 10000));
}


// Highly optimised microsecond delay function
// Despite this, the minimum realisable delay is ~10us with a 32MHz CPU clock
void timer_delay_us(uint32_t delay_us)
{
    uint8_t   tmr0_ticks, start, ms_start;
    uint16_t  end;

    // In this function, we only ever read the l.s. byte of ms_ticks which is an
    // atomic operation so there is no need to disable interrupts around the read
    ms_start = (uint8_t)ms_ticks;
    start = TMR0;

    if (delay_us < 10)
        // Too short, just return
        return;
    else if (delay_us < 1000)
    {
        // < 1ms delay - use Timer 0 directly
        // This divide is slow so we optimise it as much as possible
        if (delay_us == last_delay_us)
            // Use cached value to avoid the divide
            tmr0_ticks = last_tmr0_ticks;
        else
        {
            if (delay_us < 256)
                // Short delay - use an 8-bit divide
                tmr0_ticks = (uint8_t)delay_us / tmr0_tick_us;
            else
                // Long delay - use a 16-bit divide
                tmr0_ticks = (uint8_t)((uint16_t)delay_us / tmr0_tick_us);

            last_delay_us = delay_us;
            last_tmr0_ticks = tmr0_ticks;
        }

        end = (uint16_t)start + tmr0_ticks;

        // Deal with the wrap-around case
        if (end >= tmr0_period)
        {
            end -= tmr0_period;

            while ((uint8_t)ms_ticks == ms_start)
                ;
        }

        // Wait for Timer 0 to reach the end time
        while (TMR0 < (uint8_t)end)
            ;
    }
    else
        // 1ms+ delay
        timer_delay(delay_us / 1000);
}


/*
 * @brief Gets the total number of (1ms) ticks since reset
 */
uint32_t timer_get_run_time(void)
{
    uint32_t  ticks;

    // Since ms_ticks is 32-bit, reading it is a non-atomic operation on a PIC,
    // so we have to disable the timer interrupt around this read
    TMR0IE = 0;
    ticks = ms_ticks;
    TMR0IE = 1;

    return ticks;
}


/*
 * @brief Returns the time remaining to a target time based on timer_get_run_time()
 *
 * @param[in]   target_ms   Target time in ms
 *
 * @returns Number of milliseconds left before the target time
 *          A value of zero indicates that the target time has been reached
 *          A negative value indicates that the target time has been passed
 */
int32_t timer_run_time_remaining(uint32_t target_ms)
{
    int32_t  gap;

    gap = (int32_t)target_ms - (int32_t)timer_get_run_time();

    return gap;
}


/**
 * @brief Calls all the timer callback functions that need servicing (their timers have expired).
 *        The main program loop is expected to call this function whenever the variable 'expired'
 *        is true and it will set 'expired' to false when finished - see timer_init() above.
 */
void timer_service_callbacks(void)
{
    software_timer_t  *timer;
    fp_timer_t        callback_fn;
    void              *callback_params;
    int               timer_id;
    uint8_t           i;

    *timer_expired = false;

    timer = sw_timers;
    i = 0;

    while (timer < last_timer)
    {
        if (timer->running && (timer->time_left_ms == 0))
        {
            // Do all the timer processing we need to before the callback
            // as the callback could do anything to our timer including
            // deleting it, so we need to make sure we don't do anything
            // after the callback that might alter the user's intentions.
            timer_id = (int)(((uint16_t)timer->timer_id_qual << 8) | (uint16_t)i);

            callback_fn = timer->callback_fn;
            callback_params = timer->callback_params;

            if (timer->one_shot)
                // One shot timer - we don't need it any more
                timer_delete(timer_id);
            else
                // Repetitive timer - reset it
                timer_reset(timer_id);

            // Call the user's callback function.
            if (callback_fn != NULL)
                callback_fn(callback_params);
        }

        timer++;
        i++;
    }
}


// Helper function that calculates last_timer when we create or delete
// timers from the list.  The timer interrupt and timer_service_callbacks()
// only service up to last_timer which saves time.  We can have holes in
// the list so last_timer has to include the holes as well which means
// we have to search the list backwards.
static void set_last_timer(void)
{
    software_timer_t  *timer;

    timer = &sw_timers[MAX_SOFTWARE_TIMERS - 1];

    while (timer >= sw_timers)
    {
        if (timer->period_ms != 0)
            break;

        timer--;
    }
    timer++;

    // Since last_timer is a pointer, writing it is a non-atomic operation on
    // a PIC, so we have to disable the timer interrupt around this write
    TMR0IE = 0;
    last_timer = timer;
    TMR0IE = 1;
}


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
int timer_create(uint32_t period_ms, const fp_timer_t callback, void *params, bool start, bool one_shot)
{
    software_timer_t  *timer;
    uint8_t           i;

    // Check the input parameters
    if ((callback == NULL) || (period_ms == 0))
        return -1;

    // Find a free slot
    for (i = 0; i < MAX_SOFTWARE_TIMERS; i++)
    {
        timer = &sw_timers[i];

        if (timer->period_ms == 0)
        {
            // Increment the qualifier
            // Legal values are 1-127
            timer_id_qualifier++;
            timer_id_qualifier &= 0x7F;
            if (timer_id_qualifier == 0)
                timer_id_qualifier = 1;

            // Fill in the data
            timer->period_ms = period_ms;
            timer->time_left_ms = period_ms;
            timer->callback_fn = callback;
            timer->callback_params = params;
            timer->timer_id_qual = timer_id_qualifier;
            timer->one_shot = one_shot;
            timer->running = start;

            set_last_timer();

            // Timer created, merge the qualifier and index to create a timer_id value.
            // The top (sign) bit is always zero, the next 7 bits are the qualifier
            // and the lower 8 bits are the index into sw_timers[].
            // We don't need or allow 256 timers on a PIC but keeping the index and
            // qualifier in separate bytes avoids a costly two-byte 7-bit shift
            // operation in timer_id_valid() and timer_service_callbacks().
            return (int)(((uint16_t)timer_id_qualifier << 8) | (uint16_t)i);
        }
    }

    // No free slots
    return -1;
}


// Helper function that checks the timer_id and returns its index in the timer list
static bool timer_id_valid(int timer_id, uint8_t *index)
{
    uint8_t  qual;

    *index = (uint8_t)timer_id;
    qual = (uint8_t)((uint16_t)timer_id >> 8);

    // Return true if timer_id is valid
    return ((timer_id > 0) && (*index < MAX_SOFTWARE_TIMERS) && (sw_timers[*index].timer_id_qual == qual));
}


/**
 * @brief Deletes a software timer
 *        Ignored if the timer doesn't exist or has already been deleted
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_delete(int timer_id)
{
    software_timer_t  *timer;
    uint8_t           index;

    if (timer_id_valid(timer_id, &index))
    {
        timer = &sw_timers[index];

        // Stop the timer and reset all the variables so the timer slot can be re-used
        timer->running = false;
        timer->period_ms = 0;
        timer->time_left_ms = 0;
        timer->callback_fn = NULL;
        timer->callback_params = NULL;
        timer->timer_id_qual = 0;
        timer->one_shot = false;

        set_last_timer();
    }
}


/**
 * @brief Resets the time elapsed for a software timer
 *        If the timer is stopped, it will start from zero when it is started
 *        Ignored if the timer doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_reset(int timer_id)
{
    software_timer_t  *timer;
    uint32_t          period;
    uint8_t           index;

    if (timer_id_valid(timer_id, &index))
    {
        // Reload original period
        timer = &sw_timers[index];
        period = timer->period_ms;

        // 32-bit write is non-atomic on a PIC
        TMR0IE = 0;
        timer->time_left_ms = period;
        TMR0IE = 1;
    }
}


/**
 * @brief Reschedules a software timer then resets it
 *        If the timer is stopped, the new setting will be used when it is started
 *        Ignored if the timer doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 * @param[in]   period_ms   New timer period in milliseconds
 */
void timer_reschedule(int timer_id, uint32_t period_ms)
{
    uint8_t  index;

    if (timer_id_valid(timer_id, &index))
    {
        if (period_ms != 0)
        {
            sw_timers[index].period_ms = period_ms;
            timer_reset(timer_id);
        }
    }
}


/**
 * @brief Starts a stopped software timer
 *        Ignored if the timer is already running or doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_start(int timer_id)
{
    uint8_t  index;

    if (timer_id_valid(timer_id, &index))
        sw_timers[index].running = true;
}


/**
 * @brief Stops a running software timer
 *        Ignored if the timer is already stopped or doesn't exist
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_stop(int timer_id)
{
    uint8_t  index;

    if (timer_id_valid(timer_id, &index))
        sw_timers[index].running = false;
}


/**
 * @brief Causes a software timer to finish and set the 'expired' flag as if
 *        it had run for the full time period.  timer_service_callbacks() will
 *        then call its callback function.  This function can be used to force
 *        a callback to a foreground function from an interrupt.
 *        Ignored if the timer is stopped or doesn't exist.
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
void timer_finish(int timer_id)
{
    software_timer_t  *timer;
    uint8_t           index;

    if (timer_id_valid(timer_id, &index))
    {
        timer = &sw_timers[index];

        if (timer->running)
        {
            // 32-bit write is non-atomic on a PIC
            TMR0IE = 0;
            timer->time_left_ms = 0;
            TMR0IE = 1;

            // Set the 'expired' flag
            *timer_expired = true;
        }
    }
}


/**
 * @brief Returns the time left to run on a software timer in milliseconds
 *
 * @param[in]   timer_id    The timer id provided by timer_create()
 */
uint32_t timer_time_left(int timer_id)
{
    software_timer_t  *timer;
    uint32_t          time_left;
    uint8_t           index;

    if (timer_id_valid(timer_id, &index))
    {
        timer = &sw_timers[index];

        // 32-bit read is non-atomic on a PIC
        TMR0IE = 0;
        time_left = timer->time_left_ms;
        TMR0IE = 1;

        return time_left;
    }

    return 0;
}


/**
 * @brief Starts a hardware timer which interrupts and sets timed_out after period_us.
 *        There is only one shared timer, the application must ensure no overlaps occur.
 *        Each call resets the timer so it is not necessary to stop the timer but the
 *        pointer to timed_out must remain valid at all times as it may be written after
 *        the caller has finished with the timer.
 *
 * @param[in]   period_us   Timeout period in microseconds
 * @param[in]   timed_out   Pointer to a variable that is set to true after period_us.
 *                          WARNING: Must be a static global as it will be set by an interrupt.
 */
void timer_set_timeout(uint32_t period_us, volatile bool *const timed_out)
{
    uint32_t  load;

    // Stop any interrupts happening while we setup the timers
    TMR5IE = 0;

    timer_timed_out = timed_out;
    *timer_timed_out = false;

    // On the PIC we use TIMER3 and TIMER5 joined to form a 32-bit
    // register which we clock at 1us unless the CPU clock is > 32MHz,
    // in which case the timer runs 0.5us ticks - see timer_init()
    if (chip_get_cpu_clock() > 32000000)
        period_us <<= 1;

    // Stop TIMER3 - also stops TIMER5 as TIMER5 is clocked from TIMER3
    // If the previous timer interrupted this will already have happened
    T3CONbits.ON = 0;

    // Calculate the value to load so that it overflows after period_us
    load = (uint32_t)((int32_t)0 - (int32_t)period_us);

    // We want to load the timer now but it won't work...
    // Due to a Silicon bug, if we load TIMER5 with FFFF then increment,
    // the interrupt doesn't always happen.  This is because the overflow
    // logic requires a timer clock after an overflow to reset itself and
    // we stop the timer on overflow.  To work around this, we temporarily
    // connect TIMER5 to the CPU clock which is enough to clear the logic.
    T5CON = 7;
    T5CLK = 2;

    // Now connect TIMER5 to TIMER3 overflow
    // The required value is different for different chips !!
#if defined(_18F25K83) || defined(_18F26K83)
    T5CLK = 0xB;
#elif defined(_18F26Q10)
    T5CLK = 0xA;
#elif defined(_18F26Q43) || defined(_18F27Q43)
    T5CLK = 0xC;
#endif

    // Now load the timers
    TMR5H = (uint8_t)(load >> 24);
    TMR5L = (uint8_t)(load >> 16);
    TMR3H = (uint8_t)(load >> 8);
    TMR3L = (uint8_t)load;

    // Enable TIMER5 overflow interrupt
    TMR5IF = 0;
    TMR5IE = 1;

    // Start the timer
    T3CONbits.ON = 1;
}


#ifdef USE_HARDWARE_RTC
#error "USE_HARDWARE_RTC specified - PIC18 devices do not have a hardware RTC"
#endif

// PIC18s don't have on-board RTCs so this is the only option
#ifdef USE_SOFTWARE_RTC


/**
 * @brief Initialise the real-time clock hardware (or software)
 *        This should be called before using other RTC functions
 */
void rtc_init_hardware(void)
{
    // rtc_counter is a 64-bit variable.  We need to not have an
    // interrupt which changes the value during the 8 byte writes.
    TMR0IE = 0;
    rtc_counter = 0;
    TMR0IE = 1;
}


/**
 * @brief Gets the value of the real time clock hardware in seconds and milliseconds
 *        'secs' is designed to be compatible with the 32-bit time_t type
 *
 * @param[out]  secs   The number of seconds since the start of time (user defined)
 * @param[out]  ms     The number of milliseconds as a fraction of the second (if supported by hardware)
 *                     Set this to NULL if milliseconds are not required
 */
void rtc_get_ticks(uint32_t *secs, uint16_t *ms)
{
    uint64_t  rtc_count;

    // rtc_counter is a 64-bit variable.  We need to not have an
    // interrupt which changes the value during the 8 byte reads.
    TMR0IE = 0;
    rtc_count = rtc_counter;
    TMR0IE = 1;

    *secs = (uint32_t)(rtc_count / 1000);

    if (ms != NULL)
    {
        // What we want is *ms = (uint16_t)(rtc_count % 1000);
        // However, XC8 V2.2...V2.32 has a bug when returning a
        // 16-bit value after a div or mod library call on PIC18
        // target architecture so we avoid the mod operator...
        *ms = (uint16_t)(rtc_count - ((uint64_t)(*secs) * 1000));
    }
}


/**
 * @brief Sets the value of the real time clock hardware in seconds and milliseconds
 *        'secs' is designed to be compatible with the 32-bit time_t type
 *
 * @param[in]   secs   The number of seconds since the start of time (user defined)
 * @param[in]   ms     The number of milliseconds as a fraction of the second (if supported by hardware)
 */
void rtc_set_ticks(uint32_t secs, uint16_t ms)
{
    uint64_t  new_rtc_count;

    new_rtc_count = ((uint64_t)secs * 1000) + ms;

    // rtc_counter is a 64-bit variable.  We need to not have an
    // interrupt which changes the value during the 8 byte writes.
    TMR0IE = 0;
    rtc_counter = new_rtc_count;
    TMR0IE = 1;
}


#endif

