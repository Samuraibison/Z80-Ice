/*
 * This is a HAL-compatible port of a Xilinx CPLD programmer written by
 * ESD Electronics (see below).  GPIOs XTCK, XTMS, XTDI and XTDO need to
 * be defined in hal_config.h and an implementation of xprog_read_xsvf_byte()
 * needs to be provided.  Then just call xprog_init() and xprog_program().
 *
 * (C) Copyright 2003
 * Stefan Roese, esd gmbh germany, stefan.roese@esd-electronics.com
 *
 * SPDX-License-Identifier: GPL-2.0+
 */

/* Error codes for program_xilinx() */
#define XSVF_ERROR_NONE         0
#define XSVF_ERROR_UNKNOWN      1
#define XSVF_ERROR_TDOMISMATCH  2
#define XSVF_ERROR_MAXRETRIES   3   /* TDO mismatch after max retries */
#define XSVF_ERROR_ILLEGALCMD   4
#define XSVF_ERROR_ILLEGALSTATE 5
#define XSVF_ERROR_DATAOVERFLOW 6   /* Data > lenVal MAX_LEN buffer size*/
/* Insert new errors here */
#define XSVF_ERROR_LAST         7


// Call this first
void xprog_init(void);


/*****************************************************************************
 * Function:     program_xilinx
 * Description:  Process, interpret, and apply the XSVF commands.
 *               The source of XSVF data is xprog_read_xsvf_byte()
 * Returns:      int - For error codes see above.
 *****************************************************************************/
int xprog_program(void);


/* Reads the next byte of data from the xsvf file */
void xprog_read_xsvf_byte(uint8_t *data);

