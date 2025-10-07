/**
 * Interface to trace functions
 *
 * The following #defines can be placed in hal_config.h to control trace operation...
 *
 * #define TRACE_TIME_DISABLE  If hal_time is not available on the platform.
 *                             This will mean that trace_time() is not available.
 *
 * #define TRACE_NON_BLOCKING  To stop trace blocking if the buffer is full on serial
 *                             port implementations.  This may be required if the UART
 *                             is configured with flow control and the cable can be
 *                             disconnected or PC powered down preventing transmission.
 *                             It may also be used in time-sensitive applications to
 *                             prevent unexpected delays when tracing.  In non-blocking
 *                             mode, trace data may be lost when the UART buffer is full.
 *
 * Copyright Debug Innovations Ltd. 2015-2022
 */

#ifndef TRACE_H
#define TRACE_H

#include <stdint.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif

// If the RTC is not required, trace_time() cannot work.
// This test requires that hal_config.h is included before trace.h and there
// is no requirement for an include guard in the user's project code so we
// can't just include it here in case the user puts code in the header, so
// we also repeat this test in every trace implementation to catch all cases.
#if defined(RTC_NOT_REQUIRED) && !defined(TRACE_TIME_DISABLE)
#define TRACE_TIME_DISABLE  1
#endif


// Raw colour values (but see below)
#define TRACE_RED       9
#define TRACE_GREEN     10
#define TRACE_BLUE      12
#define TRACE_YELLOW    (TRACE_RED | TRACE_GREEN)
#define TRACE_MAGENTA   (TRACE_RED | TRACE_BLUE)
#define TRACE_CYAN      (TRACE_BLUE | TRACE_GREEN)
#define TRACE_WHITE     (TRACE_RED | TRACE_GREEN | TRACE_BLUE)

// Better to use these most of the time to keep a consistent colour scheme
#define TRACE_INFO      7
#define TRACE_WARNING   TRACE_YELLOW
#define TRACE_ERROR     TRACE_RED
#define TRACE_ANNOUNCE  TRACE_CYAN


/* Colour Mode 0 : Black and white
 *             1 : Colour (default)
 *             2 : Prints a char representing the colour
 *                 followed by the text in black and white
 */
void trace_colour_mode(uint8_t mode);

// Standard trace, like printf(), adds a \n at the end of the line,
// and if required by the platform, adds CRs to every \n so that
// \n is guaranteed to go to the start of the next line on any
// platform.  Use trace_raw() for more control.
void trace(char colour, const char *format, ...);

// As above but doesn't add a newline at the end or add CRs.
// To construct a single line from 2 parts, call trace_raw() for
// the first part followed by trace() for the second part.
void trace_raw(char colour, const char *format, ...);

// This requires a #define because it relies on hal_time which may not be available
#ifndef TRACE_TIME_DISABLE
// Timed trace, like trace() but displays a timestamp as well
void trace_time(char colour, const char *format, ...);
#else
// Use regular trace() for trace_time() calls if time not available
#define trace_time  trace
#endif

// Trace for buffer dumps, shows in hex and ASCII
void trace_dump(char colour, uint8_t *data, int num_bytes);

// Sets the display format for buffer dumps.
// By default num_addr_digits is zero and bytes_per_line is 16.
// If num_addr_digits is non-zero, addr will auto-increment with
// each displayed line and each call to trace_dump().
void trace_dump_format(uint32_t addr, uint8_t num_addr_digits, uint8_t bytes_per_line);

// getch() for the trace terminal
char trace_getch(void);

// Flush any buffered trace data
void trace_flush(void);


#ifdef __cplusplus
}
#endif

#endif

