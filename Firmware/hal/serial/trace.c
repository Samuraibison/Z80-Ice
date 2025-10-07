/**
 * @brief  Trace functions for use with PuTTY over a HAL based UART
 *
 * TRACE_UART is the UART number to be used and must be defined in hal_config.h.
 *
 * By default, the trace calls block if the UART buffer is full so no text is lost.
 * To prevent blocking, #define TRACE_NON_BLOCKING in hal_config.h.
 *
 * If the platform has no RTC support, #define TRACE_TIME_DISABLE in hal_config.h.
 *
 * Copyright Debug Innovations Ltd. 2015-2023
 */

// Suppress deprecation warnings if built on Windows
#define _CRT_SECURE_NO_WARNINGS   1

#ifdef __XC8
// Suppress some annoying XC8 compiler warnings
#pragma warning disable 1496   // Complains about Microchip's own va_start() lib code
#endif

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "hal_config.h"
#include "hal_uart.h"
#include "trace.h"

#if defined(RTC_NOT_REQUIRED) && !defined(TRACE_TIME_DISABLE)
#define TRACE_TIME_DISABLE  1
#endif

#ifndef TRACE_TIME_DISABLE
#include <time.h>
#include "hal_time.h"
#endif


static char      str[200];
static uint8_t   colour_mode = 1;
static uint32_t  dump_addr = 0;
static uint8_t   dump_num_addr_digits = 0;
static uint8_t   dump_bytes_per_line = 16;


/* Colour Mode 0 : Black and white
 *             1 : Colour
 *             2 : Prints a char representing the colour
 *                 followed by the text in black and white
 */
static void translate_colour(char *colour)
{
    if (*colour < 0x20)
    {
        if (colour_mode == 0)
            // Black & White
            *colour = 7;
        else if (colour_mode == 2)
        {
            // Use a character to classify the colour
            switch (*colour)
            {
                case TRACE_INFO:
                    *colour = 'I';
                    break;

                case TRACE_WARNING:
                    *colour = 'W';
                    break;

                case TRACE_ERROR:
                    *colour = 'E';
                    break;

                default:
                    *colour = '?';
                    break;
            }
        }
    }
}


static void convert_lf_to_crlf(char *str)
{
    int   str_len;
    char  *c, *s, *d;
    bool  before, after;

    c = str;

    while (*c != '\0')
    {
        if (*c == '\n')
        {
            // Check if there is already a CR before or after the LF
            before = ((c != str) && (*(c - 1) == '\r'));
            after = (*(c + 1) == '\r');

            if (!before && !after)
            {
                // Move all the characters after the LF including the LF
                // up 1 char to make space for a CR
                str_len = (int)strlen(str);

                // Work backwards from and include the null terminator
                s = &str[str_len];
                d = s + 1;

                while (d > c)
                    *d-- = *s--;

                // Then insert a CR in the gap
                *d = '\r';
                c++;
            }
        }
        c++;
    }
}


static void trace_display(char colour, const char *format, va_list args, bool add_newline)
{
#ifndef TRACE_NON_BLOCKING
    int      tx_count, tx_space;
#endif
    uint8_t  *s;
    int      len, tx_buff_size;
    bool     bright;


#ifdef __XC8
    /* As always, XC8 needs some kind of workaround to make it work properly.
     * The PIC XC8 compiler tries to be clever and optimise out printf() format handlers
     * for format specifiers that haven't been used in a project.  Just as it fails to
     * calculate the correct size of a pointer if it is assigned indirectly, it also fails
     * to calculate which specifiers have been used if you pass in a variable format string.
     * To avoid this problem occurring every time trace is used, we have a sprintf() here
     * which is executed once on XC8 builds, which includes all the common formatting a
     * typical application might use in a trace call. The library code is in doprnt.c, which
     * can be found approx. here: Program Files/Microchip/xc8/v2.2/pic/sources/c99/common/
     * Because the extra code takes up so much memory, you can #define symbols in hal_config.h
     * to remove support for the categories below.  What a mess XC8 is.
     */
    static bool  xc8_init = false;

    if (!xc8_init)
    {
#ifndef XC8_TRACE_NO_32BIT
        // 32-bit integers, adds about 1.2K code space
        sprintf(str, "%ld%lu", (long)0, (unsigned long)0);
#endif

#ifndef XC8_TRACE_NO_64BIT
        // 64-bit integers, adds about 1.3K code space
        sprintf(str, "%lld%llu", (long long)0, (unsigned long long)0);
#endif

#ifndef XC8_TRACE_NO_FLOAT
        // Floating point, adds about 7K code space
        sprintf(str, "%f%lf", (float)0, (double)0);
#endif
        xc8_init = true;
    }
#endif

    // Adjust the colour based on the colour mode
    translate_colour(&colour);

    len = 0;

    // If the colour code is < 0x20 (a control code), treat it as a colour
    // Otherwise, print the character at the start of the line
    if (colour < 0x20)
    {
        if (colour != 7)
        {
            bright = (colour >= 8);

            if (bright)
                colour &= 7;

            // Start with an ANSI colour sequence
            // suitable for PuTTY
            str[len++] = 0x1B;  // ESC
            str[len++] = '[';
            str[len++] = '3';
            str[len++] = colour + '0';

            if (bright)
            {
                // Using a bright clolour, insert ';1'
                str[len++] = ';';
                str[len++] = '1';
            }

            str[len++] = 'm';
        }
    }
    else
    {
        str[len++] = colour;
        str[len++] = ':';
    }

    len += vsprintf(&str[len], format, args);

    if (add_newline)
    {
        // Add a CRLF at the end of the main text
        str[len++] = '\r';
        str[len++] = '\n';
    }

    if ((colour < 0x20) && (colour != 7))
    {
        // Put a 'back to default' colour at the end
        str[len++] = 0x1B;  // ESC
        str[len++] = '[';
        str[len++] = '0';
        str[len++] = 'm';
    }

    if (add_newline)
    {
        // Translate LFs to CRLFs
        str[len] = '\0';
        convert_lf_to_crlf(str);
        len = (int)strlen(str);
    }

    // Transmitting the text is complicated by the blocking behaviour...
    uart_get_buff_sizes(TRACE_UART, &tx_buff_size, NULL);

    if (tx_buff_size == 0)
    {
        // hal_uart implementations that have no buffer simply block until there is
        // space in the hardware buffer so they can't support TRACE_NON_BLOCKING
        uart_transmit(TRACE_UART, (uint8_t *)str, len);
    }
    else
    {
#ifdef TRACE_NON_BLOCKING

        // Non-blocking behaviour
        // Just call transmit and hope it has enough space in its buffer
        uart_transmit(TRACE_UART, (uint8_t *)str, len);

#else

        // Blocking behaviour
        s = (uint8_t *)str;

        while (len > 0)
        {
            // Wait for space in the buffer
            do
            {
                uart_get_buff_count(TRACE_UART, &tx_count, NULL);
                tx_space = tx_buff_size - tx_count;
            }
            while (tx_space == 0);

            // Send as much as we can fit in the buffer
            tx_count = len;
            if (tx_count > tx_space)
                tx_count = tx_space;

            uart_transmit(TRACE_UART, s, tx_count);
            len -= tx_count;
            s += tx_count;
        }

#endif
    }
}


/* Colour Mode 0 : Black and white
 *             1 : Colour (default)
 *             2 : Prints a char representing the colour
 *                 followed by the text in black and white
 */
void trace_colour_mode(uint8_t mode)
{
    colour_mode = mode;
}


// Standard trace, like printf(), adds a \n at the end of the line,
// and if required by the platform, adds CRs to every \n so that
// \n is guaranteed to go to the start of the next line on any
// platform.  Use trace_raw() for more control.
void trace(char colour, const char *format, ...)
{
    va_list  args;

    va_start(args, format);
    trace_display(colour, format, args, true);
    va_end(args);
}


// As above but doesn't add a newline at the end or add CRs.
// To construct a single line from 2 parts, call trace_raw() for
// the first part followed by trace() for the second part.
void trace_raw(char colour, const char *format, ...)
{
    va_list  args;

    va_start(args, format);
    trace_display(colour, format, args, false);
    va_end(args);
}


#ifndef TRACE_TIME_DISABLE

// Timed trace, like trace() but displays a timestamp as well
void trace_time(char colour, const char *format, ...)
{
#ifdef __XC8
static
#endif
    char       buff[sizeof(str) + 15];
    time_t     now;
    struct tm  *local_time;
    va_list    args;
    uint32_t   secs;
    uint16_t   msec;

    rtc_get_ticks(&secs, &msec);
    now = (time_t)secs;

    local_time = localtime(&now);
    sprintf(buff, "%02d:%02d:%02d.%03u %s", local_time->tm_hour, local_time->tm_min, local_time->tm_sec, msec, format);

    va_start(args, format);
    trace_display(colour, buff, args, true);
    va_end(args);
}

#endif


static char hex_digit(uint8_t data)
{
    if (data > 9)
        data += 7;

    return data + '0';
}


// Trace for buffer dumps, shows in hex and ASCII
void trace_dump(char colour, uint8_t *data, int num_bytes)
{
    char      address[11], line[67];
    char      ch;
    char      *ptr, *chars;
    uint32_t  addr;
    uint16_t  i, byte, bytes_left;


    byte = 0;
    address[0] = '\0';
    line[sizeof(line) - 1] = '\0';

    while (byte < num_bytes)
    {
        // At the start of a new line, fill line[] with spaces
        memset(line, (int)' ', sizeof(line) - 1);

        bytes_left = (uint16_t)num_bytes - byte;
        if (bytes_left > dump_bytes_per_line)
            bytes_left = dump_bytes_per_line;

        // Construct the address if required
        if (dump_num_addr_digits > 0)
        {
            addr = dump_addr;
            ptr = &address[dump_num_addr_digits + 2];
            *ptr-- = '\0';
            *ptr-- = ' ';
            *ptr-- = ' ';

            for (i = 0; i < dump_num_addr_digits; i++)
            {
                *ptr-- = hex_digit(addr & 0xF);
                addr >>= 4;
            }
            dump_addr += dump_bytes_per_line;
        }

        // Followed by the hex dump
        ptr = line;
        chars = &line[50];

        for (i = 0; i < bytes_left; i++)
        {
            // Add a space between blocks of 8 values
            if ((i > 0) && ((i % 8) == 0))
                ptr++;

            // Print each byte as 2 hex digits
            *ptr++ = hex_digit((data[byte] >> 4) & 0xF);
            *ptr++ = hex_digit(data[byte] & 0xF);
            ptr++;

            // Fill in the character representation block
            ch = (char)(data[byte]);
            if ((ch < ' ') || (ch > 126))
                ch = '.';
            *chars++ = ch;

            byte++;
        }

        // At the end of the line, send it to be traced
        trace(colour, "%s%s", address, line);
    }
}


// Sets the display format for buffer dumps.
// By default num_addr_digits is zero and bytes_per_line is 16.
// If num_addr_digits is non-zero, addr will auto-increment with
// each displayed line and each call to trace_dump().
void trace_dump_format(uint32_t addr, uint8_t num_addr_digits, uint8_t bytes_per_line)
{
    dump_addr = addr;
    dump_num_addr_digits = num_addr_digits;
    dump_bytes_per_line = bytes_per_line;

    // Impose some upper limits on settings
    if (dump_num_addr_digits > 8)
        dump_num_addr_digits = 8;

    if (dump_bytes_per_line > 16)
        dump_bytes_per_line = 16;
}


// getch() for the trace terminal
char trace_getch(void)
{
    uint8_t  ch;
    int      num_bytes;

    do
    {
        num_bytes = 1;
        uart_receive(TRACE_UART, &ch, &num_bytes);
    }
    while (num_bytes == 0);

    return (char)ch;
}


// Flush any buffered trace data
void trace_flush(void)
{
    uart_wait_for_transmit_complete(TRACE_UART);
}

