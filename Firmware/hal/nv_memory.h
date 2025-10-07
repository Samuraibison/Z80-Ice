/**
 * @brief  Non-volatile memory interface
 *
 * Copyright Debug Innovations Ltd. 2016-2021
 */

#ifndef NV_MEMORY_H
#define NV_MEMORY_H

#include <stdint.h>

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


/* Bitfields for drive_type field below */
#define FLASH_MEMORY       0
#define EEPROM_MEMORY      1
#define INTERNAL_MEMORY    0
#define EXTERNAL_MEMORY    2
#define FILE_EMULATION     0x80

/* Drive properties descriptor */
struct drive_desc
{
    uint32_t  size_in_bytes;
    char      *drive_name;
    uint8_t   drive_type;
};
typedef struct drive_desc  drive_desc_t;


typedef enum
{
    STATUS_SUCCESS,
    STATUS_ERROR,
    STATUS_NV_MEM_INVALID_ADDR,
    STATUS_NV_MEM_INVALID_REQUEST,
    STATUS_NV_MEM_READ_ONLY,
    STATUS_NV_MEM_WRITE_FAILURE,
    STATUS_NV_MEM_ERASE_FAILURE,
    STATUS_NV_MEM_COMMS_FAILURE,
    STATUS_NV_MEM_FULL,
    STATUS_FS_INVALID_REQUEST,
    STATUS_FS_FILE_NOT_FOUND,
    STATUS_FS_FULL,
    STATUS_FS_CORRUPT_FILE
} status_t;


/* @brief  Initializes the non-volatile storage and all the drives supported by this driver */
status_t nv_init(void);


/* @brief  Gets the number of drives available.  Drive numbers start at zero and are used
 *         to identify the drive in calls to the other functions.
 * @returns Number of drives available
 */
uint8_t nv_get_num_drives(void);


/* @brief  Gets the properties of a specific drive
 * @param[in]   drive       Drive number
 * @param[out]  properties  Drive properties
 */
status_t nv_get_properties(uint8_t drive, drive_desc_t *properties);


/* @brief  Reads non-volatile memory
 * @param[in]   drive       Drive number
 * @param[in]   addr        Address offset to start reading from
 * @param[out]  data        Data read
 * @param[in]   num_bytes   Number of bytes to read
 */
status_t nv_read(uint8_t drive, uint32_t addr, uint8_t *data, uint32_t num_bytes);


/* @brief  Writes non-volatile memory
 * @param[in]   drive       Drive number
 * @param[in]   addr        Address offset to start writing to
 * @param[in]   data        Data to be written
 * @param[in]   num_bytes   Number of bytes to write
 */
status_t nv_write(uint8_t drive, uint32_t addr, uint8_t *data, uint32_t num_bytes);


/* @brief  Flushes pending writes to the non-volatile memory if they have been cached by the driver
 *         If the driver doesn't do any cacheing, it should provide a stub version of this function
 *         which simply returns STATUS_SUCCESS.
 * @param[in]   drive       Drive number
 */
status_t nv_flush(uint8_t drive);


/* @brief  Erases the whole memory for this drive */
status_t nv_erase(uint8_t drive);


#ifdef __cplusplus
}
#endif

#endif

