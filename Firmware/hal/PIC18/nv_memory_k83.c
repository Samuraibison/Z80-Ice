/**
 * @brief  Non-volatile (flash) memory driver for PIC18 K83 devices.
 *
 * The following devices plus their LF equivalents are supported:
 *   PIC18F 25K83  Has 32K bytes of flash (0-0x7FFF)
 *          26K83  Has 64K bytes of flash (0-0xFFFF)
 *
 * IMPORTANT: The CPU stalls when the flash is written on a PIC so
 *            using the flash will slow down the timer services.
 *            If using the timers to measure flash performance,
 *            the results will appear to be better than they are.
 *            Secondly, to avoid corrupting UART data in transit,
 *            this code flushes the UARTs before writing the flash
 *            so the UARTs must be initialised before writing the
 *            flash memory.
 *
 * On these platforms, PIC instructions are 16-bit and stored in
 * little-endian order aligned on even addresses starting at zero.
 * Data can be stored as bytes with no restriction on alignment.
 * It is the programmer's responsibility not to overwrite the code
 * by splitting the flash into code and data drives in hal_config.h.
 *
 * Copyright Debug Innovations Ltd. 2024
 */

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "lf_pics.h"
#include "hal_config.h"
#include "hal_uart.h"
#include "trace.h"
#include "nv_memory.h"


// #define NO_NV_MEM_TRACE in hal_config.h if you don't want nv_memory
// trace messages - this saves memory and avoids UART channel conflicts
#ifdef NO_NV_MEM_TRACE
#define trace(level, format)
#endif

#if defined(_18F25K83)
#define FLASH_SIZE     0x8000    // 32K
#elif defined(_18F26K83)
#define FLASH_SIZE     0x10000   // 64K
#else
#error "nv_memory_k83.c doesn't support this device"
#endif

#define FLASH_TOP      (FLASH_SIZE - 1)
#define SECTOR_SIZE    128


// These are defined in hal_config.h
static const uint32_t  drive_base_addrs[NUM_DRIVES] = DRIVE_BASE_ADDRS;
static const uint32_t  drive_sizes[NUM_DRIVES]      = DRIVE_SIZES;
static const char      *drive_names[NUM_DRIVES]     = DRIVE_NAMES;


// Used for the read-modify-write operation to edit the flash sectors
static uint8_t  sector_buffer[SECTOR_SIZE];


// Write a blank sector, flash_addr must be aligned to the sector base,
// returns true if successful
static bool flash_write_sector(uint32_t flash_addr, uint8_t data[SECTOR_SIZE])
{
    uint8_t  GIEbit, count;
    bool     ret;

    if (flash_addr > FLASH_TOP)
        return false;

    // Because the CPU stalls on flash writes, we need to make sure all
    // the UART data is sent first or we get corrupted UART data
    uart_wait_for_all_transmit_complete();

    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    // Addressing is complicated by the way the table write instructions work.
    // Each write increments the address but the final address has to be within
    // the sector.  We could decrement the address by doing a dummy read at the
    // end but when writing the last sector in the flash, the final address
    // will be outside the flash.  The datasheet shows some assembler which we
    // copy by starting at the address before the sector by doing a dummy read
    // at the start and using the pre-increment write so the final address is
    // at the end of the sector.

    // Read the first address in the sector and decrement the address so it
    // points to the byte before the sector start
    __asm("TBLRD*-");

    // Load the sector buffer with data
    // This form of loop produces the smallest/fastest PIC code
    count = SECTOR_SIZE;
    do
    {
        TABLAT = *data++;

        // Increment the address then write one byte
        __asm("TBLWT+*");
    }
    while (--count != 0);

    // Save the current interrupt enable state and disable interrupts
    GIEbit = (uint8_t)GIE;
    GIE = 0;

    // Enable flash writes
    NVMCON1bits.WREN = 1;

    // Unlock Flash
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;

    // Execute write request
    // CPU stalls until this completes
    NVMCON1bits.WR = 1;

    // Re-enable interrupts if they were originally enabled
    GIE = (__bit)GIEbit;

    if (NVMCON1bits.WRERR)
        // Write error, flash is probably write-protected
        ret = false;
    else
        ret = true;

    // Disable further writes
    NVMCON1 = 0x80;

    return ret;
}


// Erase a single sector, flash_addr must be within the sector
static bool flash_erase_sector(uint32_t flash_addr)
{
    uint8_t  GIEbit;
    bool     ret;

    if (flash_addr > FLASH_TOP)
        return false;

    // Because the CPU stalls on flash erases, we need to make sure all
    // the UART data is sent first or we get corrupted UART data
    uart_wait_for_all_transmit_complete();

    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    // Save the current interrupt enable state and disable interrupts
    GIEbit = (uint8_t)GIE;
    GIE = 0;

    // Enable flash writes and set FREE bit
    NVMCON1 = 0x94;

    // Unlock Flash
    NVMCON2 = 0x55;
    NVMCON2 = 0xAA;

    // Execute sector erase request
    // CPU stalls until this completes
    NVMCON1bits.WR = 1;

    // Re-enable interrupts if they were originally enabled
    GIE = (__bit)GIEbit;

    if (NVMCON1bits.WRERR)
        // Write error, flash is probably write-protected
        ret = false;
    else
        ret = true;

    // Disable further writes
    NVMCON1 = 0x80;

    return ret;
}


// Erase a range of sectors (start to end address inclusive)
static bool erase_sectors(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t  addr;
    bool      ret;

    ret = true;

    for (addr = start_addr; addr <= end_addr; addr += SECTOR_SIZE)
    {
        if (!flash_erase_sector(addr))
        {
            ret = false;
            break;
        }
    }

    return ret;
}


// Checks 'expected' against flash contents
// Returns true if they match
static bool verify(uint32_t flash_addr, uint8_t *expected, uint32_t num_bytes)
{
    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    while (num_bytes-- != 0)
    {
        __asm("TBLRD*+");

        if (TABLAT != *expected++)
            return false;
    }

    return true;
}


// Returns true if a sector is blank
static bool is_blank_sector(uint32_t flash_addr)
{
    uint8_t  count;

    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    // This form of loop produces the smallest/fastest PIC code
    count = SECTOR_SIZE;
    do
    {
        __asm("TBLRD*+");

        if (TABLAT != 0xFF)
            return false;
    }
    while (--count != 0);

    return true;
}


static status_t sector_write(uint32_t sector_base_addr, uint8_t *data)
{
    uint32_t  *flash, *end;
    bool      erase_needed;

    // Compare the new data to what is already in the flash
    // If it is identical, there is nothing to write
    if (verify(sector_base_addr, data, SECTOR_SIZE))
        return STATUS_SUCCESS;

    // If the sector isn't blank, we need to erase it before writing
    erase_needed = !is_blank_sector(sector_base_addr);

    if (erase_needed && !erase_sectors(sector_base_addr, sector_base_addr))
        return STATUS_NV_MEM_ERASE_FAILURE;

    if (!flash_write_sector(sector_base_addr, data))
        return STATUS_NV_MEM_WRITE_FAILURE;

    return STATUS_SUCCESS;
}


// Read a single sector
static void flash_read_sector(uint32_t flash_addr, uint8_t *data)
{
    uint8_t  count;

    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    // This form of loop produces the smallest/fastest PIC code
    count = SECTOR_SIZE;
    do
    {
        __asm("TBLRD*+");
        *data++ = TABLAT;
    }
    while (--count != 0);
}


status_t nv_read(uint8_t drive, uint32_t addr, uint8_t *data, uint32_t num_bytes)
{
    uint32_t  flash_addr;

    if (data == NULL)
    {
        trace(TRACE_ERROR, "Error: NULL pointer passed to nv_read()");
        return STATUS_NV_MEM_INVALID_REQUEST;
    }

    // flash_addr is the address relative to the flash base
    flash_addr = drive_base_addrs[drive] + addr;

    // Check the address
    if ((addr >= drive_sizes[drive]) || ((addr + num_bytes) > drive_sizes[drive]) || (flash_addr > FLASH_TOP))
    {
        trace(TRACE_ERROR, "Error: nv_read() addr range outside flash range");
        return STATUS_NV_MEM_INVALID_ADDR;
    }

    // Select Flash region
    NVMCON1 = 0x80;

    // Load the address
    TBLPTRU = (uint8_t)(flash_addr >> 16);
    TBLPTRH = (uint8_t)(flash_addr >> 8);
    TBLPTRL = (uint8_t)flash_addr;

    while (num_bytes-- != 0)
    {
        __asm("TBLRD*+");
        *data++ = TABLAT;
    }

    return STATUS_SUCCESS;
}


status_t nv_write(uint8_t drive, uint32_t addr, uint8_t *data, uint32_t num_bytes)
{
    status_t  result;
    uint32_t  flash_addr, addr_offset, start_len, main_len, end_len, byte_count, sector_addr;

    // Check parameters
    if (data == NULL)
    {
        trace(TRACE_ERROR, "Error: NULL pointer passed to nv_write()");
        return STATUS_NV_MEM_INVALID_REQUEST;
    }

    // flash_addr is the address relative to the flash base
    flash_addr = drive_base_addrs[drive] + addr;

    // Check the address
    if ((addr >= drive_sizes[drive]) || ((addr + num_bytes) > drive_sizes[drive]) || (flash_addr > FLASH_TOP))
    {
        trace(TRACE_ERROR, "Error: nv_write() addr range outside flash range");
        return STATUS_NV_MEM_INVALID_ADDR;
    }

    // Compare the new data to what is already in the flash
    // If it is identical, there is nothing to write
    if (verify(flash_addr, data, num_bytes))
        return STATUS_SUCCESS;

    // Work out the sector base address and offset within the sector
    addr_offset = flash_addr & (SECTOR_SIZE - 1);
    sector_addr = flash_addr - addr_offset;

    /* We can only write whole sectors to the flash, so to write a
     * smaller amount of data, we have to read the sector, modify it
     * and write it back.  If the data crosses a sector boundary, it
     * gets trickier.  So the algorithm we use is:
     *   1. Write the data up to the first sector boundary using
     *      the read-modify-write technique (the start block).
     *   2. Write a number of complete sectors (the main block).
     *   3. Write the remaining data in the final sector similar
     *      to the first sector (the end block).
     * First calculate how many bytes are in each data block.
     */

    /* Start block length */
    if (addr_offset != 0)
    {
        start_len = SECTOR_SIZE - addr_offset;
        if (start_len > num_bytes)
            start_len = num_bytes;
    }
    else
        start_len = 0;

    // Program the start block
    if (start_len > 0)
    {
        // Read existing flash contents
        flash_read_sector(sector_addr, sector_buffer);

        // Modify buffer with the data we want
        memcpy(&sector_buffer[addr_offset], data, (size_t)start_len);

        // Write the sector, erasing if necessary
        result = sector_write(sector_addr, sector_buffer);
        if (result != STATUS_SUCCESS)
            return result;

        sector_addr += SECTOR_SIZE;
    }

    // Main block length
    num_bytes -= start_len;
    main_len = num_bytes;

    // Program the main block
    for (byte_count = 0; byte_count < main_len; byte_count += SECTOR_SIZE)
    {
        // If there aren't enough bytes left for a whole sector read, then we must have gone into the end block
        if (num_bytes < SECTOR_SIZE)
            break;
        else
            num_bytes -= SECTOR_SIZE;

        // Write the sector, erasing if necessary
        result = sector_write(sector_addr, &data[start_len + byte_count]);
        if (result != STATUS_SUCCESS)
            return result;

        sector_addr += SECTOR_SIZE;
    }

    // End block length - must be whatever's left after we're done with the main blocks
    end_len = num_bytes;

    // Program the end block
    if (end_len > 0)
    {
        // Read existing flash contents
        flash_read_sector(sector_addr, sector_buffer);

        // Modify buffer with the data we want
        memcpy(sector_buffer, &data[start_len + main_len - end_len], (size_t)end_len);

        // Write the sector, erasing if necessary
        result = sector_write(sector_addr, sector_buffer);
        if (result != STATUS_SUCCESS)
            return result;
    }

    return STATUS_SUCCESS;
}


status_t nv_init(void)
{
#ifndef NO_NV_MEM_TRACE

    uint8_t  drive;

    trace(TRACE_INFO, "Using PIC18 Flash driver");

    for (drive = 0; drive < NUM_DRIVES; drive++)
        trace(TRACE_INFO, "Drive %u: %s (%uK bytes)", drive, drive_names[drive], drive_sizes[drive] >> 10);
#endif

    return STATUS_SUCCESS;
}


uint8_t nv_get_num_drives(void)
{
    return NUM_DRIVES;
}


status_t nv_get_properties(uint8_t drive, drive_desc_t *properties)
{
    if (drive < NUM_DRIVES)
    {
        properties->drive_name = (char *)drive_names[drive];
        properties->size_in_bytes = drive_sizes[drive];
        properties->drive_type = FLASH_MEMORY | INTERNAL_MEMORY;
        return STATUS_SUCCESS;
    }
    else
        return STATUS_NV_MEM_INVALID_REQUEST;
}


status_t nv_flush(uint8_t drive)
{
    return STATUS_SUCCESS;
}


status_t nv_erase(uint8_t drive)
{
    uint32_t  base_addr, start_addr, end_addr;
    status_t  result;

    //trace(TRACE_INFO, "Flash Erase, drive %u...", drive);

    base_addr = drive_base_addrs[drive];

    start_addr = base_addr;
    end_addr = (base_addr + drive_sizes[drive]) - 1;

    result = erase_sectors(start_addr, end_addr) ? STATUS_SUCCESS : STATUS_NV_MEM_ERASE_FAILURE;

    return result;
}

