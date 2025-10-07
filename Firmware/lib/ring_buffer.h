/**
 * Interface to the ring buffer module
 *
 * IMPORTANT: This is portable code.  If used with interrupts, the foreground code
 *            will need need interrupt disable/enable around some of the calls.
 *            In a multi-threaded environment, mutex get/release is required around
 *            these calls.  The affected calls are identified in the headers below.
 *            To allow embedded code with interrupt disable/enable to be emulated
 *            on Windows (requiring mutexes), there is a Windows implementation
 *            containing the necessary mutexes to make it thread-safe.
 *
 * Copyright Debug Innovations Ltd. 2014-2019
 */

#ifdef _MSC_VER
// This is for HANDLE
// We break our normal rule about headers including headers because
// without this all the embedded code that uses ring buffers would
// need this conditional windows.h include and that is worse.
#include <windows.h>
#endif


/* Make it usable with C and C++ */
#ifdef __cplusplus
extern "C" {
#endif


/**
 * Descriptor for a ringbuffer
 *
 * Do not access this directly, use the functions provided
 */
typedef struct
{
    uint8_t                *memory;       // Memory used for the buffer itself
    unsigned int           size;          // Size of the buffer
    volatile unsigned int  fill;          // Number of bytes currently in the buffer
    volatile unsigned int  read_pos;      // Reading position
    volatile unsigned int  write_pos;     // Writing position
#ifdef _MSC_VER
    HANDLE                 lock_handle;   // Handle for mutex lock on Windows
#endif
}
ringbuff_t;


/**
 * Initialises a ringbuffer
 *
 * @param[in]   buffer          Pointer to the ringbuffer descriptor
 * @param[in]   buff_mem        Pointer to the ringbuffer memory area
 * @param[in]   buff_mem_size   Size of the ringbuffer memory area
 *
 * @returns true if the ringbuffer is successfully created
 */
bool ringbuff_init(ringbuff_t *buffer, void *buff_mem, unsigned int buff_mem_size);


/**
 * Get the number of bytes in a ringbuffer
 * Note: On CPUs that cannot read an int in one memory operation,
 *       this function will need interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer   Pointer to the ringbuffer descriptor
 *
 * @returns the number of bytes in the buffer
 */
int ringbuff_len(ringbuff_t *buffer);


/**
 * Clears a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer   Pointer to the ringbuffer descriptor
 */
void ringbuff_clear(ringbuff_t *buffer);


/**
 * Puts a byte in a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 * @param[in]   byte      Byte to store
 *
 * @returns number of bytes now in the buffer, -1 if the buffer is full
 */
int ringbuff_put(ringbuff_t *buffer, uint8_t byte);


/**
 * Gets a byte from a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns a byte from the buffer, -1 if the buffer is empty
 */
int ringbuff_get(ringbuff_t *buffer);


/**
 * Peeks at the next byte in a ringbuffer but doesn't remove it
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns a byte from the buffer, -1 if the buffer is empty
 */
int ringbuff_peek(ringbuff_t *buffer);


/**
 * Deletes the last byte put in a ringbuffer if there is one.
 * Useful for implementing the backspace function for UARTs.
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of the file
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns number of bytes in the buffer after deletion, -1 if the buffer
 *          is empty before we try to delete a byte
 */
int ringbuff_delete_last(ringbuff_t *buffer);


#ifdef __cplusplus
}
#endif

