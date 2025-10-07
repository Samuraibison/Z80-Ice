/*
 * Byte packing/unpacking library functions
 *
 * These pack/unpack functions are designed to simplify storing variables in byte arrays
 * before writing them to flash/disk or sending messages over a communications interface.
 * Data is packed in little-endian order regardless of the endianness of the processor
 * it is running on i.e. the code is portable and endian-neutral.
 * pack_data() and unpack_data() have additional usage constraints to guarantee this.
 *
 * Copyright Debug Innovations Ltd. 2017-2023
 */

#include <stdint.h>
#include <string.h>
#include <math.h>
#include "pack.h"


/* Put a byte into a buffer and increment the buffer pointer */
void pack8(uint8_t **buff, uint8_t data)
{
    (*buff)[0] = data;
    (*buff) += 1;
}


/* Pack a 16-bit value into 2 bytes of a buffer and increment the buffer pointer by 2 */
void pack16(uint8_t **buff, uint16_t data)
{
    (*buff)[0] = (uint8_t)(data & 0xFF);
    (*buff)[1] = (uint8_t)((data >> 8) & 0xFF);
    (*buff) += 2;
}


/* Pack a 32-bit value into 4 bytes of a buffer and increment the buffer pointer by 4 */
void pack32(uint8_t **buff, uint32_t data)
{
    (*buff)[0] = (uint8_t)(data & 0xFF);
    (*buff)[1] = (uint8_t)((data >> 8) & 0xFF);
    (*buff)[2] = (uint8_t)((data >> 16) & 0xFF);
    (*buff)[3] = (uint8_t)((data >> 24) & 0xFF);
    (*buff) += 4;
}


/* Pack a 64-bit value into 8 bytes of a buffer and increment the buffer pointer by 8 */
void pack64(uint8_t **buff, uint64_t data)
{
    pack32(buff, (uint32_t)(data & 0xFFFFFFFF));
    pack32(buff, (uint32_t)((data >> 32) & 0xFFFFFFFF));
}


/* Pack a floating point double value into 12 bytes of a buffer and increment the
 * buffer pointer by 12.  This is a processor-neutral implementation so it is safe to
 * transfer doubles between different processor types and/or use different compilers.
 */
void pack_double(uint8_t **buff, double data)
{
    double   mantissa;
    int      exponent;
    int64_t  man64;
    int32_t  exp32;

    /* Break the double into mantissa and exponent */
    mantissa = frexp(data, &exponent);

    /* Convert the mantissa to a 64-bit signed integer
     * and the exponent to a 32-bit signed integer.
     * The range of the mantissa is -1 to +1.
     */
    man64 = (int64_t)(mantissa * INT64_MAX);
    exp32 = exponent;

    /* Pack them in the buffer as unsigned values */
    pack64(buff, (uint64_t)man64);
    pack32(buff, (uint32_t)exp32);
}


/* Put a block of data into a buffer and increment the buffer pointer */
void pack_data(uint8_t **buff, void *data, uint32_t num_bytes)
{
    memcpy((*buff), data, (size_t)num_bytes);
    (*buff) += num_bytes;
}


/* Pack a null terminated string into a buffer and move the buffer pointer.
 * The null terminator is included in the packed string.
 */
void pack_string(uint8_t **buff, char *str)
{
    char  *s;

    s = str;

    /* Copy the string to buff */
    while (*s != '\0')
    {
        **buff = (uint8_t)(*s++);
        (*buff) += 1;
    }

    /* Include the null terminator */
    **buff = '\0';
    (*buff) += 1;
}


/* Read a byte from a buffer and increment the buffer pointer */
uint8_t unpack8(uint8_t **buff)
{
    uint8_t  data;

    data = (*buff)[0];
    (*buff) += 1;

    return data;
}


/* Unpack a 16-bit value from 2 bytes of a buffer and increment the buffer pointer by 2 */
uint16_t unpack16(uint8_t **buff)
{
    uint16_t  data;

    data = (uint16_t)((*buff)[0]) | ((uint16_t)((*buff)[1]) << 8);
    (*buff) += 2;

    return data;
}


/* Unpack a 32-bit value from 4 bytes of a buffer and increment the buffer pointer by 4 */
uint32_t unpack32(uint8_t **buff)
{
    uint32_t  data;

    data = (uint32_t)((*buff)[0]) | ((uint32_t)((*buff)[1]) << 8) |
           ((uint32_t)((*buff)[2]) << 16) | ((uint32_t)((*buff)[3]) << 24);
    (*buff) += 4;

    return data;
}


/* Unpack a 64-bit value from 8 bytes of a buffer and increment the buffer pointer by 8 */
uint64_t unpack64(uint8_t **buff)
{
    return (uint64_t)unpack32(buff) | ((uint64_t)unpack32(buff) << 32);
}


/* Unpack a floating point double value from 12 bytes of a buffer and increment the
 * buffer pointer by 12.  This is a processor-neutral implementation so it is safe to
 * transfer doubles between different processor types and/or use different compilers.
 */
double unpack_double(uint8_t **buff)
{
    double   mantissa;
    int64_t  man64;
    int32_t  exp32;

    /* Unpack the mantissa and exponent from the buffer as signed values */
    man64 = (int64_t)unpack64(buff);
    exp32 = (int32_t)unpack32(buff);

    /* Convert the mantissa to a double with a range of -1 to +1 */
    mantissa = (double)man64 / INT64_MAX;

    /* Re-combine the mantissa and exponent to get a regular double value */
    return (mantissa * pow(2.0, (int)exp32));
}


/* Unpack a block of data from a buffer and increment the buffer pointer */
void unpack_data(uint8_t **buff, void *data, uint32_t num_bytes)
{
    memcpy(data, (*buff), (size_t)num_bytes);
    (*buff) += num_bytes;
}


/* Unpack a null terminated string from a buffer and move the buffer pointer.
 * Characters are read until a null terminator is reached or (str_size - 1)
 * chars have been read.  str is always null-terminated.  After the string is
 * read, the buffer pointer will point to the next packed item.
 */
void unpack_string(uint8_t **buff, char *str, uint32_t str_size)
{
    uint32_t  char_count;

    char_count = 0;

    /* Copy the string from buff to str */
    while ((**buff != '\0') && (char_count < (str_size - 1)))
    {
        str[char_count++] = (char)(**buff);
        (*buff) += 1;
    }

    str[char_count] = '\0';

    /* If we ran out of characters in the str buffer, move the pointer
     * up to the stored null terminator
     */
    while (**buff != '\0')
        (*buff) += 1;

    /* Point to the next item */
    (*buff) += 1;
}


/* @brief Inserts a block of data into another block of data at a specified position by
 *        pushing up the remaining bytes.  The result will contain (num_bytes + ins_bytes)
 *        of data.  For strings, num_bytes should be strlen() + 1 and ins_bytes should be
 *        strlen() for the null terminator to be handled correctly.
 * @param[in/out]  data       The data to be manipulated
 * @param[in]      num_bytes  The number of valid bytes of input data
 * @param[in]      ins_pos    The insert position
 * @param[in]      ins_data   The data to be inserted
 * @param[in]      ins_bytes  The number of valid bytes of in_data
 */
void buff_insert(uint8_t *data, uint32_t num_bytes, uint32_t ins_pos, uint8_t *ins_data, uint32_t ins_bytes)
{
    uint32_t  b, block_size;
    uint8_t   *s, *d;

    if ((data == NULL) || (ins_data == NULL))
        return;

    // Check for nothing to insert or insertion past the end
    if ((ins_bytes == 0) || (ins_pos > num_bytes))
        return;

    // Starting at the end...
    s = &data[num_bytes - 1];

    // ...move the data up ins_bytes...
    d = s + ins_bytes;

    // ...this many times
    block_size = num_bytes - ins_pos;

    // Work backward to the insert point
    for (b = 0; b < block_size; b++)
        *d-- = *s--;

    // Then insert the data into the gap
    memcpy(&data[ins_pos], ins_data, (size_t)ins_bytes);
}


/* @brief Removes bytes from a block of data at a specified position by copying down the
 *        remaining bytes.  The result will contain (num_bytes - del_bytes) of data.
 *        For strings, num_bytes should be strlen() + 1 for the null terminator to be
 *        handled correctly.
 * @param[in/out]  data       The data to be manipulated
 * @param[in]      num_bytes  The number of valid bytes of input data
 * @param[in]      del_pos    The start position of the deleted data
 * @param[in]      del_bytes  The number of bytes to delete
 */
void buff_delete(uint8_t *data, uint32_t num_bytes, uint32_t del_pos, uint32_t del_bytes)
{
    uint32_t  b, block_size;
    uint8_t   *s, *d;

    if (data == NULL)
        return;

    // Check for nothing to delete or deletion past the end
    if ((del_bytes == 0) || (del_pos >= num_bytes))
        return;

    // Constrain the delete block so it doesn't go past the end
    if ((del_pos + del_bytes) > num_bytes)
        del_bytes = num_bytes - del_pos;

    // Starting at the deletion point...
    d = &data[del_pos];

    // ...move the data down del_bytes...
    s = d + del_bytes;

    // ...this many times
    block_size = num_bytes - del_pos;

    // Work forward from the delete point to the end
    for (b = 0; b < block_size; b++)
        *d++ = *s++;
}


