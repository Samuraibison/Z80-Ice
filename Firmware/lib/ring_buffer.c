/**
 * Portable ring buffer implementation
 *
 * IMPORTANT: This is portable code.  If used with interrupts, the foreground code
 *            will need need interrupt disable/enable around some of the calls.
 *            In a multi-threaded environment, mutex get/release is required around
 *            these calls.  The affected calls are identified in the headers below.
 *            For Windows, use ring_buffer_windows.c instead which contains the
 *            necessary mutexes to make it thread safe.
 *
 * Copyright Debug Innovations Ltd. 2014-2021
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "ring_buffer.h"

#ifdef _MSC_VER
#error "Don't use ring_buffer.c on Windows - use ring_buffer_windows.c instead"
#endif


/**
 * Initialises a ringbuffer
 *
 * @param[in]   buffer          Pointer to the ringbuffer descriptor
 * @param[in]   buff_mem        Pointer to the ringbuffer memory area
 * @param[in]   buff_mem_size   Size of the ringbuffer memory area
 *
 * @returns true if the ringbuffer is successfully created
 */
bool ringbuff_init(ringbuff_t *buffer, void *buff_mem, unsigned int buff_mem_size)
{
    if ((buffer == NULL) || (buff_mem == NULL) || (buff_mem_size == 0))
        return false;

    buffer->memory = (uint8_t *)buff_mem;
    buffer->size = buff_mem_size;
    buffer->fill = 0;
    buffer->read_pos = 0;
    buffer->write_pos = 0;

    return true;
}


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
int ringbuff_len(ringbuff_t *buffer)
{
    return (int)(buffer->fill);
}


/**
 * Clears a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of ring_buffer.h
 *
 * @param[in]   buffer   Pointer to the ringbuffer descriptor
 */
void ringbuff_clear(ringbuff_t *buffer)
{
    buffer->fill = 0;
    buffer->read_pos = 0;
    buffer->write_pos = 0;
}


/**
 * Puts a byte in a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of ring_buffer.h
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 * @param[in]   byte      Byte to store
 *
 * @returns number of bytes now in the buffer, -1 if the buffer is full
 */
int ringbuff_put(ringbuff_t *buffer, uint8_t byte)
{
    unsigned int  write_pos;

    if (buffer->fill >= buffer->size)
    {
        // Buffer full
        return -1;
    }
    else
    {
        write_pos = buffer->write_pos;

        buffer->memory[write_pos++] = byte;
        if (write_pos >= buffer->size)
            write_pos = 0;

        buffer->write_pos = write_pos;

        return (int)(++buffer->fill);
    }
}


/**
 * Gets a byte from a ringbuffer
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of ring_buffer.h
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns a byte from the buffer, -1 if the buffer is empty
 */
int ringbuff_get(ringbuff_t *buffer)
{
    unsigned int  read_pos;
    int           ret;

    if (buffer->fill == 0)
    {
        // Buffer empty
        return -1;
    }
    else
    {
        read_pos = buffer->read_pos;

        ret = (int)buffer->memory[read_pos++];
        if (read_pos >= buffer->size)
            read_pos = 0;

        buffer->read_pos = read_pos;
        buffer->fill--;

        return ret;
    }
}


/**
 * Peeks at the next byte in a ringbuffer but doesn't remove it
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of ring_buffer.h
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns a byte from the buffer, -1 if the buffer is empty
 */
int ringbuff_peek(ringbuff_t *buffer)
{
    int  ret;

    if (buffer->fill == 0)
        // Buffer empty
        ret = -1;
    else
        ret = buffer->memory[buffer->read_pos];

    return ret;
}


/**
 * Deletes the last byte put in a ringbuffer if there is one.
 * Useful for implementing the backspace function for UARTs.
 * Note: This function needs interrupt and/or mutex protection,
 *       see explanation at the top of ring_buffer.h
 *
 * @param[in]   buffer    Pointer to the ringbuffer descriptor
 *
 * @returns number of bytes in the buffer after deletion, -1 if the buffer
 *          is empty before we try to delete a byte
 */
int ringbuff_delete_last(ringbuff_t *buffer)
{
    if (buffer->fill == 0)
    {
        // Buffer empty
        return -1;
    }
    else
    {
        // Work out the position of the last byte written
        if (buffer->write_pos == 0)
            buffer->write_pos = buffer->size;

        buffer->write_pos--;

        return (int)(--buffer->fill);
    }
}

