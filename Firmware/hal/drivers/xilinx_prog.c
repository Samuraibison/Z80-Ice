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

/*****************************************************************************
 * abstract:     This file contains the function, xprog_program(),
 *               call for interpreting the XSVF commands.
 * Usage:        Call xprog_program() to process XSVF data.
 *               The XSVF data is retrieved by xprog_read_xsvf_byte()
 * Options:      XSVF_SUPPORT_COMPRESSION
 *                   This define supports the XC9500/XL compression scheme.
 *                   This define adds support for XSDRINC and XSETSDRMASKS.
 * Debugging:    DEBUG_MODE (Legacy name)
 *               Define DEBUG_MODE to compile with debugging features.
 * History:      v5.00   - Improve XSTATE support.
 *                         Added XWAIT.
 *****************************************************************************/

#include "hal_config.h"
#include "hal_time.h"
#include "hal_gpio.h"
#include "xilinx_prog.h"


/*============================================================================
 * XSVF #define
 ============================================================================*/

#define XSVF_VERSION    "5.00"

/*****************************************************************************
 * Define:       XSVF_SUPPORT_COMPRESSION
 * Description:  Define this to support the XC9500/XL XSVF data compression
 *               scheme.
 *               Code size can be reduced by NOT supporting this feature.
 *               However, you must use the -nc (no compress) option when
 *               translating SVF to XSVF using the SVF2XSVF translator.
 *               Corresponding, uncompressed XSVF may be larger.
 *****************************************************************************/
#ifndef XSVF_SUPPORT_COMPRESSION
#define XSVF_SUPPORT_COMPRESSION    1
#endif

#define XSVF_ERRORCODE(errorCode)   errorCode


/*============================================================================
 * DEBUG_MODE #define
 ============================================================================*/
//#define DEBUG_MODE

#ifdef  DEBUG_MODE
#define XSVFDBG_PRINTF(iDebugLevel, pzFormat) \
        { if (xsvf_iDebugLevel >= iDebugLevel) \
            printf(pzFormat); }
#define XSVFDBG_PRINTF1(iDebugLevel, pzFormat, arg1) \
        { if (xsvf_iDebugLevel >= iDebugLevel) \
            printf(pzFormat, arg1); }
#define XSVFDBG_PRINTF2(iDebugLevel, pzFormat, arg1, arg2) \
        { if (xsvf_iDebugLevel >= iDebugLevel) \
            printf(pzFormat, arg1, arg2); }
#define XSVFDBG_PRINTF3(iDebugLevel, pzFormat, arg1, arg2, arg3) \
        { if (xsvf_iDebugLevel >= iDebugLevel) \
            printf(pzFormat, arg1, arg2, arg3); }
#define XSVFDBG_PRINTLENVAL(iDebugLevel, plen_val) \
        { if (xsvf_iDebugLevel >= iDebugLevel) \
            xsvfPrintLenVal(plen_val); }
#else   /* !DEBUG_MODE */
#define XSVFDBG_PRINTF(iDebugLevel, pzFormat)
#define XSVFDBG_PRINTF1(iDebugLevel, pzFormat, arg1)
#define XSVFDBG_PRINTF2(iDebugLevel, pzFormat, arg1, arg2)
#define XSVFDBG_PRINTF3(iDebugLevel, pzFormat, arg1, arg2, arg3)
#define XSVFDBG_PRINTLENVAL(iDebugLevel, plen_val)
#endif  /* DEBUG_MODE */


/*============================================================================
 * len_val Type Declaration
 ============================================================================*/

/* The len_val structure is a byte oriented type used to store an
 * arbitrary length binary value. As an example, the hex value
 * 0x0e3d is represented as a len_val with len=2 (since 2 bytes
 * and val[0]=0e and val[1]=3d.  val[2-MAX_LEN] are undefined.
 */

/* Maximum length (in bytes) of value to read in
 * this needs to be at least 4, and longer than the
 * length of the longest SDR instruction.  If there is,
 * only 1 device in the chain, MAX_LEN must be at least
 * ceil(27/8) == 4.  For 6 devices in a chain, MAX_LEN
 * must be 5, for 14 devices MAX_LEN must be 6, for 20
 * devices MAX_LEN must be 7, etc..
 * You can safely set MAX_LEN to a smaller number if you
 * know how many devices will be in your chain.
 */
#define MAX_LEN  10

typedef struct var_len_byte
{
    short          len;               /* number of chars in this value */
    unsigned char  val[MAX_LEN + 1];  /* bytes of data */
} len_val;


/*============================================================================
 * XSVF Type Declarations
 ============================================================================*/

/*****************************************************************************
 * Struct:       SXsvfInfo
 * Description:  This structure contains all of the data used during the
 *               execution of the XSVF.  Some data is persistent, predefined
 *               information (e.g. lRunTestTime).  The bulk of this struct's
 *               size is due to the len_val structs (defined in lenval.h)
 *               which contain buffers for the active shift data.  The MAX_LEN
 *               #define in lenval.h defines the size of these buffers.
 *               These buffers must be large enough to store the longest
 *               shift data in your XSVF file.  For example:
 *                   MAX_LEN >= ( longest_shift_data_in_bits / 8 )
 *               Because the len_val struct dominates the space usage of this
 *               struct, the rough size of this struct is:
 *                   sizeof( SXsvfInfo ) ~= MAX_LEN * 7 (number of len_vals)
 *               xsvfInitialize() contains initialization code for the data
 *               in this struct.
 *               xsvfCleanup() contains cleanup code for the data in this
 *               struct.
 *****************************************************************************/
typedef struct tagSXsvfInfo
{
    /* XSVF status information */
    unsigned char   ucComplete;         /* 0 = running; 1 = complete */
    unsigned char   ucCommand;          /* Current XSVF command byte */
    long            lCommandCount;      /* Number of commands processed */
    int             iErrorCode;         /* An error code. 0 = no error. */

    /* TAP state/sequencing information */
    unsigned char   ucTapState;         /* Current TAP state */
    unsigned char   ucEndIR;            /* ENDIR TAP state (See SVF) */
    unsigned char   ucEndDR;            /* ENDDR TAP state (See SVF) */

    /* RUNTEST information */
    unsigned char   ucMaxRepeat;        /* Max repeat loops (for xc9500/xl) */
    long            lRunTestTime;       /* Pre-specified RUNTEST time (usec) */

    /* Shift Data Info and Buffers */
    long            lShiftLengthBits;   /* Len. current shift data in bits */
    short           sShiftLengthBytes;  /* Len. current shift data in bytes */

    len_val         lvTdi;              /* Current TDI shift data */
    len_val         lvTdoExpected;      /* Expected TDO shift data */
    len_val         lvTdoCaptured;      /* Captured TDO shift data */
    len_val         lvTdoMask;          /* TDO mask: 0=dontcare; 1=compare */

#ifdef  XSVF_SUPPORT_COMPRESSION
    /* XSDRINC Data Buffers */
    len_val         lvAddressMask;      /* Address mask for XSDRINC */
    len_val         lvDataMask;         /* Data mask for XSDRINC */
    len_val         lvNextData;         /* Next data for XSDRINC */
#endif  /* XSVF_SUPPORT_COMPRESSION */
} SXsvfInfo;

/* Declare pointer to functions that perform XSVF commands */
typedef int (*TXsvfDoCmdFuncPtr)(SXsvfInfo *);

// See xprog_program()
static SXsvfInfo  xsvfInfo;


/*============================================================================
 * XSVF Command Bytes
 ============================================================================*/

/* encodings of xsvf instructions */
#define XCOMPLETE        0
#define XTDOMASK         1
#define XSIR             2
#define XSDR             3
#define XRUNTEST         4
/* Reserved              5 */
/* Reserved              6 */
#define XREPEAT          7
#define XSDRSIZE         8
#define XSDRTDO          9
#define XSETSDRMASKS     10
#define XSDRINC          11
#define XSDRB            12
#define XSDRC            13
#define XSDRE            14
#define XSDRTDOB         15
#define XSDRTDOC         16
#define XSDRTDOE         17
#define XSTATE           18         /* 4.00 */
#define XENDIR           19         /* 4.04 */
#define XENDDR           20         /* 4.04 */
#define XSIR2            21         /* 4.10 */
#define XCOMMENT         22         /* 4.14 */
#define XWAIT            23         /* 5.00 */
/* Insert new commands here */
/* and add corresponding xsvfDoCmd function to xsvf_pfDoCmd further down */
#define XLASTCMD         24         /* Last command marker */


/*============================================================================
 * XSVF Command Parameter Values
 ============================================================================*/

#define XSTATE_RESET     0          /* 4.00 parameter for XSTATE */
#define XSTATE_RUNTEST   1          /* 4.00 parameter for XSTATE */

#define XENDXR_RUNTEST   0          /* 4.04 parameter for XENDIR/DR */
#define XENDXR_PAUSE     1          /* 4.04 parameter for XENDIR/DR */

/* TAP states */
#define XTAPSTATE_RESET     0x00
#define XTAPSTATE_RUNTEST   0x01    /* a.k.a. IDLE */
#define XTAPSTATE_SELECTDR  0x02
#define XTAPSTATE_CAPTUREDR 0x03
#define XTAPSTATE_SHIFTDR   0x04
#define XTAPSTATE_EXIT1DR   0x05
#define XTAPSTATE_PAUSEDR   0x06
#define XTAPSTATE_EXIT2DR   0x07
#define XTAPSTATE_UPDATEDR  0x08
#define XTAPSTATE_IRSTATES  0x09    /* All IR states begin here */
#define XTAPSTATE_SELECTIR  0x09
#define XTAPSTATE_CAPTUREIR 0x0A
#define XTAPSTATE_SHIFTIR   0x0B
#define XTAPSTATE_EXIT1IR   0x0C
#define XTAPSTATE_PAUSEIR   0x0D
#define XTAPSTATE_EXIT2IR   0x0E
#define XTAPSTATE_UPDATEIR  0x0F

/*============================================================================
 * XSVF Global Variables
 ============================================================================*/

#ifdef  DEBUG_MODE
char *xsvf_pzCommandName[] =
{
    "XCOMPLETE",
    "XTDOMASK",
    "XSIR",
    "XSDR",
    "XRUNTEST",
    "Reserved5",
    "Reserved6",
    "XREPEAT",
    "XSDRSIZE",
    "XSDRTDO",
    "XSETSDRMASKS",
    "XSDRINC",
    "XSDRB",
    "XSDRC",
    "XSDRE",
    "XSDRTDOB",
    "XSDRTDOC",
    "XSDRTDOE",
    "XSTATE",
    "XENDIR",
    "XENDDR",
    "XSIR2",
    "XCOMMENT",
    "XWAIT"
};

char *xsvf_pzErrorName[] =
{
    "No error",
    "ERROR: Unknown",
    "ERROR: TDO mismatch",
    "ERROR: TDO mismatch and exceeded max retries",
    "ERROR: Unsupported XSVF command",
    "ERROR: Illegal state specification",
    "ERROR: Data overflows allocated MAX_LEN buffer size"
};

char *xsvf_pzTapState[] =
{
    "RESET",        /* 0x00 */
    "RUNTEST/IDLE", /* 0x01 */
    "DRSELECT",     /* 0x02 */
    "DRCAPTURE",    /* 0x03 */
    "DRSHIFT",      /* 0x04 */
    "DREXIT1",      /* 0x05 */
    "DRPAUSE",      /* 0x06 */
    "DREXIT2",      /* 0x07 */
    "DRUPDATE",     /* 0x08 */
    "IRSELECT",     /* 0x09 */
    "IRCAPTURE",    /* 0x0A */
    "IRSHIFT",      /* 0x0B */
    "IREXIT1",      /* 0x0C */
    "IRPAUSE",      /* 0x0D */
    "IREXIT2",      /* 0x0E */
    "IRUPDATE"      /* 0x0F */
};
#endif  /* DEBUG_MODE */

/*#ifdef DEBUG_MODE */
/*    FILE* in;   /XXX* Legacy DEBUG_MODE file pointer */
int xsvf_iDebugLevel;
/*#endif /XXX* DEBUG_MODE */


/*============================================================================
 * Utility Functions for using the len_val structure
 ============================================================================*/

/*****************************************************************************
 * Function:     value
 * Description:  Extract the long value from the lenval array.
 * Parameters:   plvValue    - ptr to lenval.
 * Returns:      long        - the extracted value.
 *****************************************************************************/
static long value(len_val *lv)
{
    long   value;         /* result to hold the accumulated result */
    short  sIndex;

    value  = 0;
    for (sIndex = 0; sIndex < lv->len; ++sIndex)
    {
        value <<= 8;                /* shift the accumulated result */
        value |= lv->val[sIndex];   /* get the last byte first */
    }

    return value;
}


/*****************************************************************************
 * Function:     EqualLenVal
 * Description:  Compare two lenval arrays with an optional mask.
 * Parameters:   expected  - ptr to lenval #1.
 *               actual    - ptr to lenval #2.
 *               mask      - optional ptr to mask (=0 if no mask).
 * Returns:      short   - 0 = mismatch; 1 = equal.
 *****************************************************************************/
static short EqualLenVal(len_val *expected, len_val *actual, len_val *mask)
{
    short          sEqual;
    short          sIndex;
    unsigned char  ucByteVal1;
    unsigned char  ucByteVal2;
    unsigned char  ucByteMask;

    sEqual = 1;
    sIndex = expected->len;

    while (sEqual && sIndex--)
    {
        ucByteVal1 = expected->val[sIndex];
        ucByteVal2 = actual->val[sIndex];

        if (mask)
        {
            ucByteMask = mask->val[sIndex];
            ucByteVal1 &= ucByteMask;
            ucByteVal2 &= ucByteMask;
        }
        if (ucByteVal1 != ucByteVal2)
        {
            sEqual = 0;
        }
    }

    return sEqual;
}


/*****************************************************************************
 * Function:     AddVal
 * Description:  add val1 to val2 and store in resVal
 *               assumes val1 and val2  are of equal length.
 * Parameters:   resVal   - ptr to result.
 *               val1     - ptr of addendum.
 *               val2     - ptr of addendum.
 *****************************************************************************/
static void addVal(len_val *resVal, len_val *val1, len_val *val2)
{
    unsigned char   ucCarry;
    unsigned short  usSum, usVal1, usVal2;
    short           sIndex;

    /* set up length of result */
    resVal->len  = val1->len;

    /* start at least significant bit and add bytes */
    ucCarry = 0;
    sIndex = val1->len;

    while (sIndex--)
    {
        usVal1 = val1->val[sIndex];   /* i'th byte of val1 */
        usVal2 = val2->val[sIndex];   /* i'th byte of val2 */

        /* add the two bytes plus carry from previous addition */
        usSum = (unsigned short)(usVal1 + usVal2 + ucCarry);

        /* set up carry for next byte */
        ucCarry = (unsigned char)((usSum > 255 ) ? 1 : 0);

        /* set the i'th byte of the result */
        resVal->val[sIndex] = (unsigned char)usSum;
    }
}


/*****************************************************************************
 * Function:     readVal
 * Description:  read from XSVF numBytes bytes of data into x.
 * Parameters:   lv         - ptr to lenval in which to put the bytes read.
 *               numBytes   - the number of bytes to read.
 *****************************************************************************/
static void readVal(len_val *lv, short numBytes)
{
    unsigned char  *pucVal;

    /* set the length of the len_val */
    lv->len = numBytes;

    for (pucVal = lv->val; numBytes; --numBytes, ++pucVal)
    {
        /* read a byte of data into the len_val */
        xprog_read_xsvf_byte(pucVal);
    }
}


/*****************************************************************************
 * Function:     xsvfPrintLenVal
 * Description:  Print the lenval value in hex.
 * Parameters:   plv     - ptr to lenval.
 * Returns:      void.
 *****************************************************************************/
#ifdef  DEBUG_MODE
static void xsvfPrintLenVal(len_val *plv)
{
    int i;

    if (plv)
    {
        printf("0x");
        for (i = 0; i < plv->len; ++i)
        {
            printf("%02x", ((unsigned int)(plv->val[i])));
        }
    }
}
#endif  /* DEBUG_MODE */


/*****************************************************************************
 * Function:     xsvfInfoInit
 * Description:  Initialize the xsvfInfo data.
 * Parameters:   pXsvfInfo   - ptr to the XSVF info structure.
 * Returns:      int         - 0 = success; otherwise error.
 *****************************************************************************/
static int xsvfInfoInit(SXsvfInfo *pXsvfInfo)
{
    XSVFDBG_PRINTF1(4, "    sizeof( SXsvfInfo ) = %d bytes\n",  sizeof(SXsvfInfo));

    pXsvfInfo->ucComplete       = 0;
    pXsvfInfo->ucCommand        = XCOMPLETE;
    pXsvfInfo->lCommandCount    = 0;
    pXsvfInfo->iErrorCode       = XSVF_ERROR_NONE;
    pXsvfInfo->ucMaxRepeat      = 0;
    pXsvfInfo->ucTapState       = XTAPSTATE_RESET;
    pXsvfInfo->ucEndIR          = XTAPSTATE_RUNTEST;
    pXsvfInfo->ucEndDR          = XTAPSTATE_RUNTEST;
    pXsvfInfo->lShiftLengthBits = 0L;
    pXsvfInfo->sShiftLengthBytes= 0;
    pXsvfInfo->lRunTestTime     = 0L;

    return 0;
}


/*****************************************************************************
 * Function:     xsvfInfoCleanup
 * Description:  Cleanup the xsvfInfo data.
 * Parameters:   pXsvfInfo   - ptr to the XSVF info structure.
 * Returns:      void.
 *****************************************************************************/
static void xsvfInfoCleanup(SXsvfInfo *pXsvfInfo)
{
}


/*****************************************************************************
 * Function:     xsvfGetAsNumBytes
 * Description:  Calculate the number of bytes the given number of bits
 *               consumes.
 * Parameters:   lNumBits    - the number of bits.
 * Returns:      short       - the number of bytes to store the number of bits.
 *****************************************************************************/
static short xsvfGetAsNumBytes(long lNumBits)
{
    return (short)((lNumBits + 7L) / 8L);
}


/*****************************************************************************
 * Function:     xsvfTmsTransition
 * Description:  Apply TMS and transition TAP controller by applying one TCK
 *               cycle.
 * Parameters:   sTms    - new TMS value.
 * Returns:      void.
 *****************************************************************************/
static void xsvfTmsTransition(short sTms)
{
    gpio_set(XTMS, (uint8_t)sTms);
    gpio_set(XTCK, 0);
    gpio_set(XTCK, 1);
}


/*****************************************************************************
 * Function:     xsvfGotoTapState
 * Description:  From the current TAP state, go to the named TAP state.
 *               A target state of RESET ALWAYS causes TMS reset sequence.
 *               All SVF standard stable state paths are supported.
 *               All state transitions are supported except for the following
 *               which cause an XSVF_ERROR_ILLEGALSTATE:
 *                   - Target==DREXIT2;  Start!=DRPAUSE
 *                   - Target==IREXIT2;  Start!=IRPAUSE
 * Parameters:   pucTapState     - Current TAP state; returns final TAP state.
 *               ucTargetState   - New target TAP state.
 * Returns:      int             - 0 = success; otherwise error.
 *****************************************************************************/
static int xsvfGotoTapState(unsigned char *pucTapState, unsigned char ucTargetState)
{
    int  i, iErrorCode;

    iErrorCode = XSVF_ERROR_NONE;

    if (ucTargetState == XTAPSTATE_RESET)
    {
        /* If RESET, always perform TMS reset sequence to reset/sync TAPs */
        xsvfTmsTransition(1);
        for (i = 0; i < 5; ++i)
        {
            gpio_set(XTCK, 0);
            gpio_set(XTCK, 1);
        }
        *pucTapState = XTAPSTATE_RESET;
        XSVFDBG_PRINTF(3, "   TMS Reset Sequence -> Test-Logic-Reset\n");
        XSVFDBG_PRINTF1(3, "   TAP State = %s\n", xsvf_pzTapState[*pucTapState]);
    }
    else if ((ucTargetState != *pucTapState) &&
             (((ucTargetState == XTAPSTATE_EXIT2DR) && (*pucTapState != XTAPSTATE_PAUSEDR)) ||
              ((ucTargetState == XTAPSTATE_EXIT2IR) && (*pucTapState != XTAPSTATE_PAUSEIR))))
    {
        /* Trap illegal TAP state path specification */
        iErrorCode = XSVF_ERROR_ILLEGALSTATE;
    }
    else
    {
        if (ucTargetState == *pucTapState)
        {
            /* Already in target state.  Do nothing except when in DRPAUSE
               or in IRPAUSE to comply with SVF standard */
            if (ucTargetState == XTAPSTATE_PAUSEDR)
            {
                xsvfTmsTransition(1);
                *pucTapState = XTAPSTATE_EXIT2DR;
                XSVFDBG_PRINTF1(3, "   TAP State = %s\n", xsvf_pzTapState[*pucTapState]);
            }
            else if (ucTargetState == XTAPSTATE_PAUSEIR)
            {
                xsvfTmsTransition(1);
                *pucTapState = XTAPSTATE_EXIT2IR;
                XSVFDBG_PRINTF1(3, "   TAP State = %s\n", xsvf_pzTapState[*pucTapState]);
            }
        }

        /* Perform TAP state transitions to get to the target state */
        while (ucTargetState != *pucTapState)
        {
            switch (*pucTapState)
            {
                case XTAPSTATE_RESET:
                    xsvfTmsTransition(0);
                    *pucTapState = XTAPSTATE_RUNTEST;
                    break;

                case XTAPSTATE_RUNTEST:
                    xsvfTmsTransition(1);
                    *pucTapState = XTAPSTATE_SELECTDR;
                    break;

                case XTAPSTATE_SELECTDR:
                    if (ucTargetState >= XTAPSTATE_IRSTATES)
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_SELECTIR;
                    }
                    else
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_CAPTUREDR;
                    }
                    break;

                case XTAPSTATE_CAPTUREDR:
                    if (ucTargetState == XTAPSTATE_SHIFTDR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_SHIFTDR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_EXIT1DR;
                    }
                    break;

                case XTAPSTATE_SHIFTDR:
                    xsvfTmsTransition(1);
                    *pucTapState = XTAPSTATE_EXIT1DR;
                    break;

                case XTAPSTATE_EXIT1DR:
                    if (ucTargetState == XTAPSTATE_PAUSEDR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_PAUSEDR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_UPDATEDR;
                    }
                    break;

                case XTAPSTATE_PAUSEDR:
                    xsvfTmsTransition(1);
                    *pucTapState = XTAPSTATE_EXIT2DR;
                    break;

                case XTAPSTATE_EXIT2DR:
                    if (ucTargetState == XTAPSTATE_SHIFTDR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_SHIFTDR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_UPDATEDR;
                    }
                    break;

                case XTAPSTATE_UPDATEDR:
                    if (ucTargetState == XTAPSTATE_RUNTEST)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_RUNTEST;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_SELECTDR;
                    }
                    break;

                case XTAPSTATE_SELECTIR:
                    xsvfTmsTransition(0);
                    *pucTapState = XTAPSTATE_CAPTUREIR;
                    break;

                case XTAPSTATE_CAPTUREIR:
                    if (ucTargetState == XTAPSTATE_SHIFTIR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_SHIFTIR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_EXIT1IR;
                    }
                    break;

                case XTAPSTATE_SHIFTIR:
                    xsvfTmsTransition(1);
                    *pucTapState = XTAPSTATE_EXIT1IR;
                    break;

                case XTAPSTATE_EXIT1IR:
                    if (ucTargetState == XTAPSTATE_PAUSEIR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_PAUSEIR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_UPDATEIR;
                    }
                    break;

                case XTAPSTATE_PAUSEIR:
                    xsvfTmsTransition(1);
                    *pucTapState = XTAPSTATE_EXIT2IR;
                    break;

                case XTAPSTATE_EXIT2IR:
                    if (ucTargetState == XTAPSTATE_SHIFTIR)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_SHIFTIR;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_UPDATEIR;
                    }
                    break;

                case XTAPSTATE_UPDATEIR:
                    if (ucTargetState == XTAPSTATE_RUNTEST)
                    {
                        xsvfTmsTransition(0);
                        *pucTapState = XTAPSTATE_RUNTEST;
                    }
                    else
                    {
                        xsvfTmsTransition(1);
                        *pucTapState = XTAPSTATE_SELECTDR;
                    }
                    break;

                default:
                    iErrorCode = XSVF_ERROR_ILLEGALSTATE;
                    *pucTapState = ucTargetState;    /* Exit while loop */
                    break;
            }
            XSVFDBG_PRINTF1(3, "   TAP State = %s\n", xsvf_pzTapState[*pucTapState]);
        }
    }

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfShiftOnly
 * Description:  Assumes that starting TAP state is SHIFT-DR or SHIFT-IR.
 *               Shift the given TDI data into the JTAG scan chain.
 *               Optionally, save the TDO data shifted out of the scan chain.
 *               Last shift cycle is special:  capture last TDO, set last TDI,
 *               but does not pulse TCK.  Caller must pulse TCK and optionally
 *               set TMS=1 to exit shift state.
 * Parameters:   lNumBits        - number of bits to shift.
 *               plvTdi          - ptr to lenval for TDI data.
 *               plvTdoCaptured  - ptr to lenval for storing captured TDO data.
 *               iExitShift      - 1=exit at end of shift; 0=stay in Shift-DR.
 * Returns:      void.
 *****************************************************************************/
static void xsvfShiftOnly(long lNumBits, len_val *plvTdi, len_val *plvTdoCaptured, int iExitShift)
{
    unsigned char  *pucTdi, *pucTdo;
    unsigned char  ucTdiByte, ucTdoByte, ucTdoBit;
    int            i;

    /* assert( ( ( lNumBits + 7 ) / 8 ) == plvTdi->len ); */

    /* Initialize TDO storage len == TDI len */
    pucTdo = 0;
    if (plvTdoCaptured)
    {
        plvTdoCaptured->len = plvTdi->len;
        pucTdo              = plvTdoCaptured->val + plvTdi->len;
    }

    /* Shift LSB first.  val[N-1] == LSB.  val[0] == MSB. */
    pucTdi = plvTdi->val + plvTdi->len;
    while (lNumBits)
    {
        /* Process on a byte-basis */
        ucTdiByte = (*(--pucTdi));
        ucTdoByte = 0;
        for (i = 0; (lNumBits && (i < 8)); ++i)
        {
            --lNumBits;
            if (iExitShift && !lNumBits)
            {
                /* Exit Shift-DR state */
                gpio_set(XTMS, 1);
            }

            /* Set the new TDI value */
            gpio_set(XTDI, ucTdiByte & 1);
            ucTdiByte >>= 1;

            /* Set TCK low */
            gpio_set(XTCK, 0);

            if (pucTdo)
            {
                /* Save the TDO value */
                ucTdoBit  = gpio_get(XTDO);
                ucTdoByte |= (ucTdoBit << i);
            }

            /* Set TCK high */
            gpio_set(XTCK, 1);
        }

        /* Save the TDO byte value */
        if (pucTdo)
        {
            (*(--pucTdo)) = ucTdoByte;
        }
    }
}


/*****************************************************************************
 * Function:     xsvfShift
 * Description:  Goes to the given starting TAP state.
 *               Calls xsvfShiftOnly to shift in the given TDI data and
 *               optionally capture the TDO data.
 *               Compares the TDO captured data against the TDO expected
 *               data.
 *               If a data mismatch occurs, then executes the exception
 *               handling loop upto ucMaxRepeat times.
 * Parameters:   pucTapState     - Ptr to current TAP state.
 *               ucStartState    - Starting shift state: Shift-DR or Shift-IR.
 *               lNumBits        - number of bits to shift.
 *               plvTdi          - ptr to lenval for TDI data.
 *               plvTdoCaptured  - ptr to lenval for storing TDO data.
 *               plvTdoExpected  - ptr to expected TDO data.
 *               plvTdoMask      - ptr to TDO mask.
 *               ucEndState      - state in which to end the shift.
 *               lRunTestTime    - amount of time to wait after the shift.
 *               ucMaxRepeat     - Maximum number of retries on TDO mismatch.
 * Returns:      int             - 0 = success; otherwise TDO mismatch.
 * Notes:        XC9500XL-only Optimization:
 *               Skip the waitTime() if plvTdoMask->val[0:plvTdoMask->len-1]
 *               is NOT all zeros and sMatch==1.
 *****************************************************************************/
static int xsvfShift(unsigned char  *pucTapState,
                     unsigned char  ucStartState,
                     long           lNumBits,
                     len_val        *plvTdi,
                     len_val        *plvTdoCaptured,
                     len_val        *plvTdoExpected,
                     len_val        *plvTdoMask,
                     unsigned char  ucEndState,
                     long           lRunTestTime,
                     unsigned char  ucMaxRepeat)
{
    unsigned char  ucRepeat;
    int            iErrorCode, iMismatch, iExitShift;

    iErrorCode = XSVF_ERROR_NONE;
    iMismatch  = 0;
    ucRepeat   = 0;
    iExitShift = (ucStartState != ucEndState);

    XSVFDBG_PRINTF1(3, "   Shift Length = %ld\n", lNumBits);
    XSVFDBG_PRINTF(4, "    TDI          = ");
    XSVFDBG_PRINTLENVAL(4, plvTdi);
    XSVFDBG_PRINTF(4, "\n");
    XSVFDBG_PRINTF(4, "    TDO Expected = ");
    XSVFDBG_PRINTLENVAL(4, plvTdoExpected);
    XSVFDBG_PRINTF(4, "\n");

    if (!lNumBits)
    {
        /* Compatibility with XSVF2.00:  XSDR 0 = no shift, but wait in RTI */
        if (lRunTestTime)
        {
            /* Wait for prespecified XRUNTEST time */
            xsvfGotoTapState(pucTapState, XTAPSTATE_RUNTEST);
            XSVFDBG_PRINTF1(3, "   Wait = %ld usec\n", lRunTestTime);
            timer_delay_us((uint32_t)lRunTestTime);
        }
    }
    else
    {
        do
        {
            /* Goto Shift-DR or Shift-IR */
            xsvfGotoTapState(pucTapState, ucStartState);

            /* Shift TDI and capture TDO */
            xsvfShiftOnly(lNumBits, plvTdi, plvTdoCaptured, iExitShift);

            if (plvTdoExpected)
            {
                /* Compare TDO data to expected TDO data */
                iMismatch = !EqualLenVal(plvTdoExpected, plvTdoCaptured, plvTdoMask);
            }

            if (iExitShift)
            {
                /* Update TAP state:  Shift->Exit */
                ++(*pucTapState);
                XSVFDBG_PRINTF1(3, "   TAP State = %s\n", xsvf_pzTapState[*pucTapState]);

                if (iMismatch && lRunTestTime && (ucRepeat < ucMaxRepeat))
                {
                    XSVFDBG_PRINTF(4, "    TDO Expected = ");
                    XSVFDBG_PRINTLENVAL(4, plvTdoExpected);
                    XSVFDBG_PRINTF(4, "\n");
                    XSVFDBG_PRINTF(4, "    TDO Captured = ");
                    XSVFDBG_PRINTLENVAL(4, plvTdoCaptured);
                    XSVFDBG_PRINTF(4, "\n");
                    XSVFDBG_PRINTF(4, "    TDO Mask     = ");
                    XSVFDBG_PRINTLENVAL(4, plvTdoMask);
                    XSVFDBG_PRINTF(4, "\n");
                    XSVFDBG_PRINTF1(3, "   Retry #%d\n", (ucRepeat + 1));
                    /* Do exception handling retry - ShiftDR only */
                    xsvfGotoTapState(pucTapState, XTAPSTATE_PAUSEDR);
                    /* Shift 1 extra bit */
                    xsvfGotoTapState(pucTapState, XTAPSTATE_SHIFTDR);
                    /* Increment RUNTEST time by an additional 25% */
                    lRunTestTime += (lRunTestTime >> 2);
                }
                else
                {
                    /* Do normal exit from Shift-XR */
                    xsvfGotoTapState(pucTapState, ucEndState);
                }

                if (lRunTestTime)
                {
                    /* Wait for prespecified XRUNTEST time */
                    xsvfGotoTapState(pucTapState, XTAPSTATE_RUNTEST);
                    XSVFDBG_PRINTF1(3, "   Wait = %ld usec\n", lRunTestTime);
                    timer_delay_us((uint32_t)lRunTestTime);
                }
            }
        }
        while (iMismatch && (ucRepeat++ < ucMaxRepeat));
    }

    if (iMismatch)
    {
        XSVFDBG_PRINTF(1, " TDO Expected = ");
        XSVFDBG_PRINTLENVAL(1, plvTdoExpected);
        XSVFDBG_PRINTF(1, "\n");
        XSVFDBG_PRINTF(1, " TDO Captured = ");
        XSVFDBG_PRINTLENVAL(1, plvTdoCaptured);
        XSVFDBG_PRINTF(1, "\n");
        XSVFDBG_PRINTF(1, " TDO Mask     = ");
        XSVFDBG_PRINTLENVAL(1, plvTdoMask);
        XSVFDBG_PRINTF(1, "\n");

        if (ucMaxRepeat && (ucRepeat > ucMaxRepeat))
            iErrorCode = XSVF_ERROR_MAXRETRIES;
        else
            iErrorCode = XSVF_ERROR_TDOMISMATCH;
    }

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfBasicXSDRTDO
 * Description:  Get the XSDRTDO parameters and execute the XSDRTDO command.
 *               This is the common function for all XSDRTDO commands.
 * Parameters:   pucTapState         - Current TAP state.
 *               lShiftLengthBits    - number of bits to shift.
 *               sShiftLengthBytes   - number of bytes to read.
 *               plvTdi              - ptr to lenval for TDI data.
 *               lvTdoCaptured       - ptr to lenval for storing TDO data.
 *               iEndState           - state in which to end the shift.
 *               lRunTestTime        - amount of time to wait after the shift.
 *               ucMaxRepeat         - maximum xc9500/xl retries.
 * Returns:      int                 - 0 = success; otherwise TDO mismatch.
 *****************************************************************************/
static int xsvfBasicXSDRTDO(unsigned char  *pucTapState,
                            long           lShiftLengthBits,
                            short          sShiftLengthBytes,
                            len_val        *plvTdi,
                            len_val        *plvTdoCaptured,
                            len_val        *plvTdoExpected,
                            len_val        *plvTdoMask,
                            unsigned char  ucEndState,
                            long           lRunTestTime,
                            unsigned char  ucMaxRepeat)
{
    readVal(plvTdi, sShiftLengthBytes);

    if (plvTdoExpected)
    {
        readVal(plvTdoExpected, sShiftLengthBytes);
    }

    return xsvfShift(pucTapState, XTAPSTATE_SHIFTDR, lShiftLengthBits, plvTdi, plvTdoCaptured,
                     plvTdoExpected, plvTdoMask, ucEndState, lRunTestTime, ucMaxRepeat);
}


/*****************************************************************************
 * Function:     xsvfDoSDRMasking
 * Description:  Update the data value with the next XSDRINC data and address.
 * Example:      dataVal=0x01ff, nextData=0xab, addressMask=0x0100,
 *               dataMask=0x00ff, should set dataVal to 0x02ab
 * Parameters:   plvTdi          - The current TDI value.
 *               plvNextData     - the next data value.
 *               plvAddressMask  - the address mask.
 *               plvDataMask     - the data mask.
 * Returns:      void.
 *****************************************************************************/
#ifdef  XSVF_SUPPORT_COMPRESSION

static void xsvfDoSDRMasking(len_val *plvTdi, len_val *plvNextData, len_val *plvAddressMask, len_val *plvDataMask)
{
    int            i;
    unsigned char  ucTdi, ucTdiMask, ucDataMask, ucNextData, ucNextMask;
    short          sNextData;

    /* add the address Mask to dataVal and return as a new dataVal */
    addVal(plvTdi, plvTdi, plvAddressMask);

    ucNextData = 0;
    ucNextMask = 0;
    sNextData  = plvNextData->len;

    for (i = plvDataMask->len - 1; i >= 0; --i)
    {
        /* Go through data mask in reverse order looking for mask (1) bits */
        ucDataMask = plvDataMask->val[i];
        if (ucDataMask)
        {
            /* Retrieve the corresponding TDI byte value */
            ucTdi = plvTdi->val[i];

            /* For each bit in the data mask byte, look for 1's */
            ucTdiMask = 1;

            while (ucDataMask)
            {
                if (ucDataMask & 1)
                {
                    if (!ucNextMask)
                    {
                        /* Get the next data byte */
                        ucNextData = plvNextData->val[--sNextData];
                        ucNextMask = 1;
                    }

                    /* Set or clear the data bit according to the next data */
                    if (ucNextData & ucNextMask)
                    {
                        ucTdi |= ucTdiMask;   /* Set bit */
                    }
                    else
                    {
                        ucTdi &= ~ucTdiMask;  /* Clear bit */
                    }

                    /* Update the next data */
                    ucNextMask <<= 1;
                }
                ucTdiMask <<= 1;
                ucDataMask >>= 1;
            }

            /* Update the TDI value */
            plvTdi->val[i] = ucTdi;
        }
    }
}

#endif  /* XSVF_SUPPORT_COMPRESSION */


/*============================================================================
 * XSVF Command Functions (type = TXsvfDoCmdFuncPtr)
 * These functions update pXsvfInfo->iErrorCode only on an error.
 * Otherwise, the error code is left alone.
 * The function returns the error code from the function.
 ============================================================================*/

/*****************************************************************************
 * Function:     xsvfDoIllegalCmd
 * Description:  Function place holder for illegal/unsupported commands.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoIllegalCmd(SXsvfInfo *pXsvfInfo)
{
    XSVFDBG_PRINTF2(0, "ERROR:  Encountered unsupported command #%d (%s)\n", ((unsigned int)(pXsvfInfo->ucCommand)),
                       ((pXsvfInfo->ucCommand < XLASTCMD) ? (xsvf_pzCommandName[pXsvfInfo->ucCommand]) : "Unknown"));

    pXsvfInfo->iErrorCode = XSVF_ERROR_ILLEGALCMD;

    return pXsvfInfo->iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXCOMPLETE
 * Description:  XCOMPLETE (no parameters)
 *               Update complete status for XSVF player.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXCOMPLETE(SXsvfInfo *pXsvfInfo)
{
    pXsvfInfo->ucComplete = 1;

    return XSVF_ERROR_NONE;
}


/*****************************************************************************
 * Function:     xsvfDoXTDOMASK
 * Description:  XTDOMASK <len_val.TdoMask[XSDRSIZE]>
 *               Prespecify the TDO compare mask.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXTDOMASK(SXsvfInfo *pXsvfInfo)
{
    readVal(&(pXsvfInfo->lvTdoMask), pXsvfInfo->sShiftLengthBytes);

    XSVFDBG_PRINTF(4, "    TDO Mask     = ");
    XSVFDBG_PRINTLENVAL(4, &(pXsvfInfo->lvTdoMask));
    XSVFDBG_PRINTF(4, "\n");

    return XSVF_ERROR_NONE;
}


/*****************************************************************************
 * Function:     xsvfDoXSIR
 * Description:  XSIR <(byte)shiftlen> <len_val.TDI[shiftlen]>
 *               Get the instruction and shift the instruction into the TAP.
 *               If prespecified XRUNTEST!=0, goto RUNTEST and wait after
 *               the shift for XRUNTEST usec.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSIR(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucShiftIrBits;
    short          sShiftIrBytes;
    int            iErrorCode;

    /* Get the shift length and store */
    xprog_read_xsvf_byte(&ucShiftIrBits);
    sShiftIrBytes = xsvfGetAsNumBytes(ucShiftIrBits);
    XSVFDBG_PRINTF1(3, "   XSIR length = %d\n", ((unsigned int)ucShiftIrBits));

    if (sShiftIrBytes > MAX_LEN)
    {
        iErrorCode = XSVF_ERROR_DATAOVERFLOW;
    }
    else
    {
        /* Get and store instruction to shift in */
        readVal(&(pXsvfInfo->lvTdi), xsvfGetAsNumBytes(ucShiftIrBits));

        /* Shift the data */
        iErrorCode = xsvfShift(&(pXsvfInfo->ucTapState), XTAPSTATE_SHIFTIR, ucShiftIrBits, &(pXsvfInfo->lvTdi),
                                 /*plvTdoCaptured*/0, /*plvTdoExpected*/0, /*plvTdoMask*/0, pXsvfInfo->ucEndIR,
                                 pXsvfInfo->lRunTestTime, /*ucMaxRepeat*/0);
    }

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSIR2
 * Description:  XSIR <(2-byte)shiftlen> <len_val.TDI[shiftlen]>
 *               Get the instruction and shift the instruction into the TAP.
 *               If prespecified XRUNTEST!=0, goto RUNTEST and wait after
 *               the shift for XRUNTEST usec.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSIR2(SXsvfInfo *pXsvfInfo)
{
    long   lShiftIrBits;
    short  sShiftIrBytes;
    int    iErrorCode;

    /* Get the shift length and store */
    readVal(&(pXsvfInfo->lvTdi), 2);
    lShiftIrBits = value(&(pXsvfInfo->lvTdi));
    sShiftIrBytes = xsvfGetAsNumBytes(lShiftIrBits);
    XSVFDBG_PRINTF1(3, "   XSIR2 length = %d\n", (int)lShiftIrBits);

    if (sShiftIrBytes > MAX_LEN)
    {
        iErrorCode = XSVF_ERROR_DATAOVERFLOW;
    }
    else
    {
        /* Get and store instruction to shift in */
        readVal(&(pXsvfInfo->lvTdi), xsvfGetAsNumBytes(lShiftIrBits));

        /* Shift the data */
        iErrorCode = xsvfShift(&(pXsvfInfo->ucTapState), XTAPSTATE_SHIFTIR, lShiftIrBits, &(pXsvfInfo->lvTdi),
                               /*plvTdoCaptured*/0, /*plvTdoExpected*/0, /*plvTdoMask*/0, pXsvfInfo->ucEndIR,
                               pXsvfInfo->lRunTestTime, /*ucMaxRepeat*/0);
    }

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSDR
 * Description:  XSDR <len_val.TDI[XSDRSIZE]>
 *               Shift the given TDI data into the JTAG scan chain.
 *               Compare the captured TDO with the expected TDO from the
 *               previous XSDRTDO command using the previously specified
 *               XTDOMASK.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSDR(SXsvfInfo *pXsvfInfo)
{
    int  iErrorCode;

    readVal(&(pXsvfInfo->lvTdi), pXsvfInfo->sShiftLengthBytes);

    /* use TDOExpected from last XSDRTDO instruction */
    iErrorCode = xsvfShift(&(pXsvfInfo->ucTapState), XTAPSTATE_SHIFTDR, pXsvfInfo->lShiftLengthBits, &(pXsvfInfo->lvTdi),
                           &(pXsvfInfo->lvTdoCaptured), &(pXsvfInfo->lvTdoExpected), &(pXsvfInfo->lvTdoMask), pXsvfInfo->ucEndDR,
                           pXsvfInfo->lRunTestTime, pXsvfInfo->ucMaxRepeat);

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXRUNTEST
 * Description:  XRUNTEST <uint32>
 *               Prespecify the XRUNTEST wait time for shift operations.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXRUNTEST(SXsvfInfo *pXsvfInfo)
{
    readVal(&(pXsvfInfo->lvTdi), 4);
    pXsvfInfo->lRunTestTime = value(&(pXsvfInfo->lvTdi));
    XSVFDBG_PRINTF1(3, "   XRUNTEST = %ld\n", pXsvfInfo->lRunTestTime);

    return XSVF_ERROR_NONE;
}


/*****************************************************************************
 * Function:     xsvfDoXREPEAT
 * Description:  XREPEAT <byte>
 *               Prespecify the maximum number of XC9500/XL retries.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXREPEAT(SXsvfInfo *pXsvfInfo)
{
    xprog_read_xsvf_byte(&pXsvfInfo->ucMaxRepeat);
    XSVFDBG_PRINTF1(3, "   XREPEAT = %d\n", ((unsigned int)(pXsvfInfo->ucMaxRepeat)));

    return XSVF_ERROR_NONE;
}


/*****************************************************************************
 * Function:     xsvfDoXSDRSIZE
 * Description:  XSDRSIZE <uint32>
 *               Prespecify the XRUNTEST wait time for shift operations.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSDRSIZE(SXsvfInfo *pXsvfInfo)
{
    int  iErrorCode;

    iErrorCode = XSVF_ERROR_NONE;

    readVal(&(pXsvfInfo->lvTdi), 4);

    pXsvfInfo->lShiftLengthBits = value(&(pXsvfInfo->lvTdi));
    pXsvfInfo->sShiftLengthBytes = xsvfGetAsNumBytes(pXsvfInfo->lShiftLengthBits);
    XSVFDBG_PRINTF1(3, "   XSDRSIZE = %ld\n", pXsvfInfo->lShiftLengthBits);

    if (pXsvfInfo->sShiftLengthBytes > MAX_LEN)
    {
        iErrorCode = XSVF_ERROR_DATAOVERFLOW;
        pXsvfInfo->iErrorCode = iErrorCode;
    }

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSDRTDO
 * Description:  XSDRTDO <len_val.TDI[XSDRSIZE]> <len_val.TDO[XSDRSIZE]>
 *               Get the TDI and expected TDO values.  Then, shift.
 *               Compare the expected TDO with the captured TDO using the
 *               prespecified XTDOMASK.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSDRTDO(SXsvfInfo *pXsvfInfo)
{
    int  iErrorCode;

    iErrorCode = xsvfBasicXSDRTDO(&(pXsvfInfo->ucTapState), pXsvfInfo->lShiftLengthBits, pXsvfInfo->sShiftLengthBytes,
                                  &(pXsvfInfo->lvTdi), &(pXsvfInfo->lvTdoCaptured), &(pXsvfInfo->lvTdoExpected),
                                  &(pXsvfInfo->lvTdoMask), pXsvfInfo->ucEndDR, pXsvfInfo->lRunTestTime, pXsvfInfo->ucMaxRepeat);

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSETSDRMASKS
 * Description:  XSETSDRMASKS <len_val.AddressMask[XSDRSIZE]>
 *                            <len_val.DataMask[XSDRSIZE]>
 *               Get the prespecified address and data mask for the XSDRINC
 *               command.
 *               Used for xc9500/xl compressed XSVF data.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
#ifdef  XSVF_SUPPORT_COMPRESSION

static int xsvfDoXSETSDRMASKS(SXsvfInfo *pXsvfInfo)
{
    /* read the addressMask */
    readVal(&(pXsvfInfo->lvAddressMask), pXsvfInfo->sShiftLengthBytes);

    /* read the dataMask */
    readVal(&(pXsvfInfo->lvDataMask), pXsvfInfo->sShiftLengthBytes);

    XSVFDBG_PRINTF(4, "    Address Mask = ");
    XSVFDBG_PRINTLENVAL(4, &(pXsvfInfo->lvAddressMask));
    XSVFDBG_PRINTF(4, "\n");
    XSVFDBG_PRINTF(4, "    Data Mask    = ");
    XSVFDBG_PRINTLENVAL(4, &(pXsvfInfo->lvDataMask));
    XSVFDBG_PRINTF(4, "\n");

    return XSVF_ERROR_NONE;
}

#endif  /* XSVF_SUPPORT_COMPRESSION */


/*****************************************************************************
 * Function:     xsvfDoXSDRINC
 * Description:  XSDRINC <len_val.firstTDI[XSDRSIZE]> <byte(numTimes)>
 *                       <len_val.data[XSETSDRMASKS.dataMask.len]> ...
 *               Get the XSDRINC parameters and execute the XSDRINC command.
 *               XSDRINC starts by loading the first TDI shift value.
 *               Then, for numTimes, XSDRINC gets the next piece of data,
 *               replaces the bits from the starting TDI as defined by the
 *               XSETSDRMASKS.dataMask, adds the address mask from
 *               XSETSDRMASKS.addressMask, shifts the new TDI value,
 *               and compares the TDO to the expected TDO from the previous
 *               XSDRTDO command using the XTDOMASK.
 *               Used for xc9500/xl compressed XSVF data.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
#ifdef  XSVF_SUPPORT_COMPRESSION

static int xsvfDoXSDRINC(SXsvfInfo *pXsvfInfo)
{
    int            iErrorCode, iDataMaskLen;
    unsigned char  ucDataMask, ucNumTimes, i;

    readVal(&(pXsvfInfo->lvTdi), pXsvfInfo->sShiftLengthBytes);

    iErrorCode = xsvfShift(&(pXsvfInfo->ucTapState), XTAPSTATE_SHIFTDR, pXsvfInfo->lShiftLengthBits,
                           &(pXsvfInfo->lvTdi), &(pXsvfInfo->lvTdoCaptured), &(pXsvfInfo->lvTdoExpected),
                           &(pXsvfInfo->lvTdoMask), pXsvfInfo->ucEndDR, pXsvfInfo->lRunTestTime, pXsvfInfo->ucMaxRepeat);

    if (!iErrorCode)
    {
        /* Calculate number of data mask bits */
        iDataMaskLen = 0;
        for (i = 0; i < pXsvfInfo->lvDataMask.len; ++i)
        {
            ucDataMask = pXsvfInfo->lvDataMask.val[i];
            while (ucDataMask)
            {
                iDataMaskLen += (ucDataMask & 1);
                ucDataMask >>= 1;
            }
        }

        /* Get the number of data pieces, i.e. number of times to shift */
        xprog_read_xsvf_byte(&ucNumTimes);

        /* For numTimes, get data, fix TDI, and shift */
        for (i = 0; !iErrorCode && (i < ucNumTimes); ++i)
        {
            readVal(&(pXsvfInfo->lvNextData), xsvfGetAsNumBytes(iDataMaskLen));

            xsvfDoSDRMasking(&(pXsvfInfo->lvTdi), &(pXsvfInfo->lvNextData),
                             &(pXsvfInfo->lvAddressMask), &(pXsvfInfo->lvDataMask));

            iErrorCode = xsvfShift(&(pXsvfInfo->ucTapState), XTAPSTATE_SHIFTDR, pXsvfInfo->lShiftLengthBits,
                                   &(pXsvfInfo->lvTdi), &(pXsvfInfo->lvTdoCaptured), &(pXsvfInfo->lvTdoExpected),
                                   &(pXsvfInfo->lvTdoMask), pXsvfInfo->ucEndDR, pXsvfInfo->lRunTestTime, pXsvfInfo->ucMaxRepeat);
        }
    }

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}

#endif  /* XSVF_SUPPORT_COMPRESSION */


/*****************************************************************************
 * Function:     xsvfDoXSDRBCE
 * Description:  XSDRB/XSDRC/XSDRE <len_val.TDI[XSDRSIZE]>
 *               If not already in SHIFTDR, goto SHIFTDR.
 *               Shift the given TDI data into the JTAG scan chain.
 *               Ignore TDO.
 *               If cmd==XSDRE, then goto ENDDR.  Otherwise, stay in ShiftDR.
 *               XSDRB, XSDRC, and XSDRE are the same implementation.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSDRBCE(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucEndDR;
    int            iErrorCode;

    ucEndDR = (unsigned char)((pXsvfInfo->ucCommand == XSDRE) ? pXsvfInfo->ucEndDR : XTAPSTATE_SHIFTDR);

    iErrorCode = xsvfBasicXSDRTDO(&(pXsvfInfo->ucTapState), pXsvfInfo->lShiftLengthBits, pXsvfInfo->sShiftLengthBytes,
                                  &(pXsvfInfo->lvTdi), /*plvTdoCaptured*/0, /*plvTdoExpected*/0, /*plvTdoMask*/0,
                                  ucEndDR, /*lRunTestTime*/0, /*ucMaxRepeat*/0);

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSDRTDOBCE
 * Description:  XSDRB/XSDRC/XSDRE <len_val.TDI[XSDRSIZE]> <len_val.TDO[XSDRSIZE]>
 *               If not already in SHIFTDR, goto SHIFTDR.
 *               Shift the given TDI data into the JTAG scan chain.
 *               Compare TDO, but do NOT use XTDOMASK.
 *               If cmd==XSDRTDOE, then goto ENDDR.  Otherwise, stay in ShiftDR.
 *               XSDRTDOB, XSDRTDOC, and XSDRTDOE are the same implementation.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSDRTDOBCE(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucEndDR;
    int            iErrorCode;

    ucEndDR = (unsigned char)((pXsvfInfo->ucCommand == XSDRTDOE) ? pXsvfInfo->ucEndDR : XTAPSTATE_SHIFTDR);

    iErrorCode = xsvfBasicXSDRTDO(&(pXsvfInfo->ucTapState), pXsvfInfo->lShiftLengthBits, pXsvfInfo->sShiftLengthBytes,
                                  &(pXsvfInfo->lvTdi), &(pXsvfInfo->lvTdoCaptured), &(pXsvfInfo->lvTdoExpected),
                                  /*plvTdoMask*/0, ucEndDR, /*lRunTestTime*/0, /*ucMaxRepeat*/0);

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXSTATE
 * Description:  XSTATE <byte>
 *               <byte> == XTAPSTATE;
 *               Get the state parameter and transition the TAP to that state.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXSTATE(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucNextState;
    int            iErrorCode;

    xprog_read_xsvf_byte(&ucNextState);

    iErrorCode = xsvfGotoTapState(&(pXsvfInfo->ucTapState), ucNextState);

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXENDXR
 * Description:  XENDIR/XENDDR <byte>
 *               <byte>:  0 = RUNTEST;  1 = PAUSE.
 *               Get the prespecified XENDIR or XENDDR.
 *               Both XENDIR and XENDDR use the same implementation.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXENDXR(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucEndState;
    int            iErrorCode;

    iErrorCode = XSVF_ERROR_NONE;

    xprog_read_xsvf_byte(&ucEndState);

    if ((ucEndState != XENDXR_RUNTEST) && (ucEndState != XENDXR_PAUSE))
    {
        iErrorCode = XSVF_ERROR_ILLEGALSTATE;
    }
    else
    {
        if (pXsvfInfo->ucCommand == XENDIR)
        {
            if (ucEndState == XENDXR_RUNTEST)
                pXsvfInfo->ucEndIR = XTAPSTATE_RUNTEST;
            else
                pXsvfInfo->ucEndIR = XTAPSTATE_PAUSEIR;

            XSVFDBG_PRINTF1(3, "   ENDIR State = %s\n", xsvf_pzTapState[pXsvfInfo->ucEndIR]);
        }
        else    /* XENDDR */
        {
            if (ucEndState == XENDXR_RUNTEST)
                pXsvfInfo->ucEndDR = XTAPSTATE_RUNTEST;
            else
                pXsvfInfo->ucEndDR = XTAPSTATE_PAUSEDR;

            XSVFDBG_PRINTF1(3, "   ENDDR State = %s\n", xsvf_pzTapState[pXsvfInfo->ucEndDR]);
        }
    }

    if (iErrorCode != XSVF_ERROR_NONE)
        pXsvfInfo->iErrorCode = iErrorCode;

    return iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXCOMMENT
 * Description:  XCOMMENT <text string ending in \0>
 *               <text string ending in \0> == text comment;
 *               Arbitrary comment embedded in the XSVF.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXCOMMENT(SXsvfInfo *pXsvfInfo)
{
    /* Use the comment for debugging */
    /* Otherwise, read through the comment to the end '\0' and ignore */
    unsigned char  ucText;

    if (xsvf_iDebugLevel > 0)
    {
        //putc(' ');
    }

    do
    {
        xprog_read_xsvf_byte(&ucText);

        if (xsvf_iDebugLevel > 0)
        {
            //putc(ucText ? ucText : '\n');
        }
    }
    while (ucText);

    pXsvfInfo->iErrorCode = XSVF_ERROR_NONE;

    return pXsvfInfo->iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfDoXWAIT
 * Description:  XWAIT <wait_state> <end_state> <wait_time>
 *               If not already in <wait_state>, then go to <wait_state>.
 *               Wait in <wait_state> for <wait_time> microseconds.
 *               Finally, if not already in <end_state>, then goto <end_state>.
 * Parameters:   pXsvfInfo   - XSVF information pointer.
 * Returns:      int         - 0 = success;  non-zero = error.
 *****************************************************************************/
static int xsvfDoXWAIT(SXsvfInfo *pXsvfInfo)
{
    unsigned char  ucWaitState, ucEndState;
    long           lWaitTime;

    /* Get Parameters */
    /* <wait_state> */
    readVal(&(pXsvfInfo->lvTdi), 1);
    ucWaitState = pXsvfInfo->lvTdi.val[0];

    /* <end_state> */
    readVal(&(pXsvfInfo->lvTdi), 1);
    ucEndState = pXsvfInfo->lvTdi.val[0];

    /* <wait_time> */
    readVal(&(pXsvfInfo->lvTdi), 4);
    lWaitTime = value(&(pXsvfInfo->lvTdi));
    XSVFDBG_PRINTF2(3, "   XWAIT:  state = %s; time = %ld\n", xsvf_pzTapState[ucWaitState], lWaitTime);

    /* If not already in <wait_state>, go to <wait_state> */
    if (pXsvfInfo->ucTapState != ucWaitState)
    {
        xsvfGotoTapState(&(pXsvfInfo->ucTapState), ucWaitState);
    }

    /* Wait for <wait_time> microseconds */
    timer_delay_us((uint32_t)lWaitTime);

    /* If not already in <end_state>, go to <end_state> */
    if (pXsvfInfo->ucTapState != ucEndState)
    {
        xsvfGotoTapState(&(pXsvfInfo->ucTapState), ucEndState);
    }

    return XSVF_ERROR_NONE;
}


/* Array of XSVF command functions.  Must follow command byte value order! */
/* If your compiler cannot take this form, then convert to a switch statement */
static const TXsvfDoCmdFuncPtr xsvf_pfDoCmd[] =
{
    xsvfDoXCOMPLETE,        /*  0 */
    xsvfDoXTDOMASK,         /*  1 */
    xsvfDoXSIR,             /*  2 */
    xsvfDoXSDR,             /*  3 */
    xsvfDoXRUNTEST,         /*  4 */
    xsvfDoIllegalCmd,       /*  5 */
    xsvfDoIllegalCmd,       /*  6 */
    xsvfDoXREPEAT,          /*  7 */
    xsvfDoXSDRSIZE,         /*  8 */
    xsvfDoXSDRTDO,          /*  9 */
#ifdef  XSVF_SUPPORT_COMPRESSION
    xsvfDoXSETSDRMASKS,     /* 10 */
    xsvfDoXSDRINC,          /* 11 */
#else
    xsvfDoIllegalCmd,       /* 10 */
    xsvfDoIllegalCmd,       /* 11 */
#endif
    xsvfDoXSDRBCE,          /* 12 */
    xsvfDoXSDRBCE,          /* 13 */
    xsvfDoXSDRBCE,          /* 14 */
    xsvfDoXSDRTDOBCE,       /* 15 */
    xsvfDoXSDRTDOBCE,       /* 16 */
    xsvfDoXSDRTDOBCE,       /* 17 */
    xsvfDoXSTATE,           /* 18 */
    xsvfDoXENDXR,           /* 19 */
    xsvfDoXENDXR,           /* 20 */
    xsvfDoXSIR2,            /* 21 */
    xsvfDoXCOMMENT,         /* 22 */
    xsvfDoXWAIT             /* 23 */
/* Insert new command functions here */
};


/*============================================================================
 * Execution Control Functions
 ============================================================================*/

/*****************************************************************************
 * Function:     xsvfInitialize
 * Description:  Initialize the xsvf player.
 *               Call this before running the player to initialize the data
 *               in the SXsvfInfo struct.
 *               xsvfCleanup is called to clean up the data in SXsvfInfo
 *               after the XSVF is played.
 * Parameters:   pXsvfInfo   - ptr to the XSVF information.
 * Returns:      int - 0 = success; otherwise error.
 *****************************************************************************/
static int xsvfInitialize(SXsvfInfo *pXsvfInfo)
{
    /* Initialize values */
    pXsvfInfo->iErrorCode = xsvfInfoInit(pXsvfInfo);

    if (!pXsvfInfo->iErrorCode)
    {
        /* Initialize the TAPs */
        pXsvfInfo->iErrorCode = xsvfGotoTapState(&(pXsvfInfo->ucTapState), XTAPSTATE_RESET);
    }

    return pXsvfInfo->iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfRun
 * Description:  Run the xsvf player for a single command and return.
 *               First, call xsvfInitialize.
 *               Then, repeatedly call this function until an error is detected
 *               or until the pXsvfInfo->ucComplete variable is non-zero.
 *               Finally, call xsvfCleanup to cleanup any remnants.
 * Parameters:   pXsvfInfo   - ptr to the XSVF information.
 * Returns:      int         - 0 = success; otherwise error.
 *****************************************************************************/
static int xsvfRun(SXsvfInfo *pXsvfInfo)
{
    /* Process the XSVF commands */
    if ((!pXsvfInfo->iErrorCode) && (!pXsvfInfo->ucComplete))
    {
        /* read 1 byte for the instruction */
        xprog_read_xsvf_byte(&pXsvfInfo->ucCommand);
        ++(pXsvfInfo->lCommandCount);

        if (pXsvfInfo->ucCommand < XLASTCMD)
        {
            /* Execute the command.  Func sets error code. */
            XSVFDBG_PRINTF1(2, "  %s\n", xsvf_pzCommandName[pXsvfInfo->ucCommand]);
            xsvf_pfDoCmd[pXsvfInfo->ucCommand](pXsvfInfo);
        }
        else
        {
            /* Illegal command value.  Func sets error code. */
            xsvfDoIllegalCmd(pXsvfInfo);
        }
    }

    return pXsvfInfo->iErrorCode;
}


/*****************************************************************************
 * Function:     xsvfCleanup
 * Description:  cleanup remnants of the xsvf player.
 * Parameters:   pXsvfInfo   - ptr to the XSVF information.
 * Returns:      void.
 *****************************************************************************/
static void xsvfCleanup(SXsvfInfo *pXsvfInfo)
{
    xsvfInfoCleanup(pXsvfInfo);
}


void xprog_init(void)
{
    gpio_configure_pin(XTCK, PIN_DIGITAL_OUT | PIN_SET_HIGH);
    gpio_configure_pin(XTMS, PIN_DIGITAL_OUT | PIN_SET_HIGH);
    gpio_configure_pin(XTDI, PIN_DIGITAL_OUT | PIN_SET_HIGH);

    // TDO can go Hi-Z so we need a pullup
    gpio_configure_pin(XTDO, PIN_DIGITAL_IN | PIN_PULL_UP);
}


/*****************************************************************************
 * Function:     xprog_program
 * Description:  Process, interpret, and apply the XSVF commands.
 *               The source of XSVF data is xprog_read_xsvf_byte()
 * Returns:      int - For error codes see above.
 *****************************************************************************/
int xprog_program(void)
{
    // We'd like to declare an SXsvfInfo block here on the stack
    // but some CPUs e.g. PICs can't take such a large struct so
    // we've already declared xsvfInfo as a static at the top

    xsvfInitialize(&xsvfInfo);

    while ((xsvfInfo.iErrorCode == 0) && !xsvfInfo.ucComplete)
    {
        xsvfRun(&xsvfInfo);
    }

    if (xsvfInfo.iErrorCode)
    {
        XSVFDBG_PRINTF1(0, "%s\n", xsvf_pzErrorName[(xsvfInfo.iErrorCode < XSVF_ERROR_LAST) ? xsvfInfo.iErrorCode : XSVF_ERROR_UNKNOWN]);
        XSVFDBG_PRINTF2(0, "ERROR at or near XSVF command #%ld.  See line #%ld in the XSVF ASCII file.\n",
                           xsvfInfo.lCommandCount, xsvfInfo.lCommandCount);
    }
    else
    {
        XSVFDBG_PRINTF(0, "SUCCESS - Completed XSVF execution.\n");
    }

    xsvfCleanup(&xsvfInfo);

    return XSVF_ERRORCODE(xsvfInfo.iErrorCode);
}

