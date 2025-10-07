/*
 * Interface to byte packing/unpacking library functions
 *
 * These pack/unpack functions are designed to simplify storing variables in byte arrays
 * before writing them to flash/disk or sending messages over a communications interface.
 * Data is packed in little-endian order regardless of the endianness of the processor
 * it is running on i.e. the code is portable and endian-neutral.
 * pack_data() and unpack_data() have additional usage constraints to guarantee this.
 *
 * Copyright Debug Innovations Ltd. 2017-2018
 */

/* Make it usable with C and C++ */
#ifdef __cplusplus
extern "C" {
#endif


/* Put a byte into a buffer and increment the buffer pointer */
void pack8(uint8_t **buff, uint8_t data);

/* Pack a 16-bit value into 2 bytes of a buffer and increment the buffer pointer by 2 */
void pack16(uint8_t **buff, uint16_t data);

/* Pack a 32-bit value into 4 bytes of a buffer and increment the buffer pointer by 4 */
void pack32(uint8_t **buff, uint32_t data);

/* Pack a 64-bit value into 8 bytes of a buffer and increment the buffer pointer by 8 */
void pack64(uint8_t **buff, uint64_t data);

/* Pack a floating point double value into 12 bytes of a buffer and increment the
 * buffer pointer by 12.  This is a processor-neutral implementation so it is safe to
 * transfer doubles between different processor types and/or use different compilers.
 */
void pack_double(uint8_t **buff, double data);

/* Put a block of data into a buffer and increment the buffer pointer.
 * WARNING: This function is designed to pack byte arrays.  Using it to pack structures
 * or multi-byte values will result in endian-sensitive behaviour.  Use the pack8-64
 * functions on each structure element or multi-byte value if endian-neutral behaviour
 * is required.
 */
void pack_data(uint8_t **buff, void *data, uint32_t num_bytes);

/* Pack a null terminated string into a buffer and move the buffer pointer.
 * The null terminator is included in the packed string.
 */
void pack_string(uint8_t **buff, char *str);


/* Read a byte from a buffer and increment the buffer pointer */
uint8_t unpack8(uint8_t **buff);

/* Unpack a 16-bit value from 2 bytes of a buffer and increment the buffer pointer by 2 */
uint16_t unpack16(uint8_t **buff);

/* Unpack a 32-bit value from 4 bytes of a buffer and increment the buffer pointer by 4 */
uint32_t unpack32(uint8_t **buff);

/* Unpack a 64-bit value from 8 bytes of a buffer and increment the buffer pointer by 8 */
uint64_t unpack64(uint8_t **buff);

/* Unpack a floating point double value from 12 bytes of a buffer and increment the
 * buffer pointer by 12.  This is a processor-neutral implementation so it is safe to
 * transfer doubles between different processor types and/or use different compilers.
 */
double unpack_double(uint8_t **buff);

/* Unpack a block of data from a buffer and increment the buffer pointer.
 * WARNING: This function is designed to unpack byte arrays.  Using it to unpack structures
 * or multi-byte values will result in endian-sensitive behaviour.  Use the unpack8-64
 * functions on each structure element or multi-byte value if endian-neutral behaviour
 * is required.
 */
void unpack_data(uint8_t **buff, void *data, uint32_t num_bytes);

/* Unpack a null terminated string from a buffer and move the buffer pointer.
 * Characters are read until a null terminator is reached or (str_size - 1)
 * chars have been read.  str is always null-terminated.  After the string is
 * read, the buffer pointer will point to the next packed item.
 */
void unpack_string(uint8_t **buff, char *str, uint32_t str_size);


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
void buff_insert(uint8_t *data, uint32_t num_bytes, uint32_t ins_pos, uint8_t *ins_data, uint32_t ins_bytes);

/* @brief Removes bytes from a block of data at a specified position by copying down the
 *        remaining bytes.  The result will contain (num_bytes - del_bytes) of data.
 *        For strings, num_bytes should be strlen() + 1 for the null terminator to be
 *        handled correctly.
 * @param[in/out]  data       The data to be manipulated
 * @param[in]      num_bytes  The number of valid bytes of input data
 * @param[in]      del_pos    The start position of the deleted data
 * @param[in]      del_bytes  The number of bytes to delete
 */
void buff_delete(uint8_t *data, uint32_t num_bytes, uint32_t del_pos, uint32_t del_bytes);


#ifdef __cplusplus
}
#endif

