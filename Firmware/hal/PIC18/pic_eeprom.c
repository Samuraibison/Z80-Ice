/**
 * @brief  PIC internal EEPROM read/write functions
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC16F 18857
 *
 *   PIC18F 25K83
 *          26K83
 *          26Q10
 *          26Q43
 *          27Q43
 *
 * For completeness, the old EEPROM access mechanism is included
 * if the selected device isn't one of the above.
 *
 * Copyright Debug Innovations Ltd. 2010-2022
 */

#include <xc.h>
#include <stdint.h>
#include "../PIC16/lf_pics.h"
#include "../PIC18/lf_pics.h"
#include "pic_eeprom.h"


#ifdef _16F18857

#define EE_BASE        0x7000
#define EE_SIZE        256

#define SETUP          NVMCON1bits.NVMREGS = 1
#define NVMLOCK        NVMCON2
#define EEREAD         NVMCON1bits.RD
#define EEWRITE        NVMCON1bits.WR
#define READ_CMD
#define WRITE_ENABLE   NVMCON1bits.WREN = 1
#define WRITE_DISABLE  NVMCON1bits.WREN = 0

#elif defined(_18F26Q10)

#define EE_BASE        0x310000
#define EE_SIZE        1024

#define SETUP          NVMCON1bits.NVMREGS = 1
#define NVMLOCK        NVMCON2
#define EEREAD         NVMCON1bits.RD
#define EEWRITE        NVMCON1bits.WR
#define READ_CMD
#define WRITE_ENABLE   NVMEN = 1
#define WRITE_DISABLE  NVMEN = 0

#elif defined(_18F25K83) || defined(_18F26K83)

#define EE_BASE        0
#define EE_SIZE        1024

#define SETUP          NVMCON1bits.REG = 0
#define NVMLOCK        NVMCON2
#define EEREAD         NVMCON1bits.RD
#define EEWRITE        NVMCON1bits.WR
#define READ_CMD
#define WRITE_ENABLE   NVMCON1bits.WREN = 1
#define WRITE_DISABLE  NVMCON1bits.WREN = 0
#define NVMDATL        NVMDAT

#elif defined(_18F26Q43) || defined(_18F27Q43)

#define EE_BASE        0x380000
#define EE_SIZE        1024

#define SETUP
#define EEREAD         NVMCON0bits.GO
#define EEWRITE        NVMCON0bits.GO
#define READ_CMD       NVMCON1bits.CMD = 0
#define WRITE_ENABLE   NVMCON1bits.CMD = 3
#define WRITE_DISABLE  READ_CMD

#else

// Old EEPROM access method via EEADR/EEDATA

// Account for register naming differences
#if defined(_12F629) || defined(_12F675) || defined(_16F684)
#define NVMDATL  EEDAT
#else
#define NVMDATL  EEDATL
#endif

#define EE_BASE           0
#define EE_SIZE           256
#define NVMADR            EEADR
#define SETUP
#define NVMLOCK           EECON2
#define EEREAD            RD
#define EEWRITE           WR
#define READ_CMD
#define WRITE_ENABLE      WREN = 1
#define WRITE_DISABLE     WREN = 0

#endif


// Read a byte from location 'addr'
uint8_t ee_read(uint16_t addr)
{
    if (addr >= EE_SIZE)
        return 0;

    SETUP;

#if defined(_16F18857) || defined(_18F25K83) || defined(_18F26K83)
    // Apparently some PICs need the data in H/L order
    // Also note that the debugger doesn't seem to be able to read NVMADRH
    NVMADRH = (uint8_t)((EE_BASE + addr) >> 8);
    NVMADRL = (uint8_t)addr;
#else
    NVMADR = EE_BASE + addr;
#endif

    // Read command
    READ_CMD;

    // Execute request and wait for read to complete
    EEREAD = 1;
    while (EEREAD)
        ;

    return NVMDATL;
}


// Write a byte 'data' to location 'addr'
void ee_write(uint16_t addr, uint8_t data)
{
    uint8_t  GIEbit;

    if (addr >= EE_SIZE)
        return;

    SETUP;

#if defined(_16F18857) || defined(_18F25K83) || defined(_18F26K83)
    // Apparently some PICs need the data in H/L order
    // Also note that the debugger doesn't seem to be able to read NVMADRH
    NVMADRH = (uint8_t)((EE_BASE + addr) >> 8);
    NVMADRL = (uint8_t)addr;
#else
    NVMADR = EE_BASE + addr;
#endif

    NVMDATL = data;

    // Save the current interrupt enable state
    GIEbit = (uint8_t)GIE;

    // Disable interrupts
    GIE = 0;

    // Enable writes
    WRITE_ENABLE;

    // Unlock EEPROM
    NVMLOCK = 0x55;
    NVMLOCK = 0xAA;

    // Execute write request
    EEWRITE = 1;

    // Re-enable interrupts if they were originally enabled
    GIE = (__bit)GIEbit;

    // Wait for write to complete
    while (EEWRITE)
        ;

    //if (NVMCON1bits.WRERR)
        // Write error - TODO: Recovery?

    // Disable further writes
    WRITE_DISABLE;
}

