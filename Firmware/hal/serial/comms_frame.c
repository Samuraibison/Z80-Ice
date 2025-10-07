/*
 * Frame construct/extract functions for simple serial comms links
 *
 *  Frame protocol:
 *
 *   Byte   Contents
 *    0     Frame start (0xAA)
 *    1     Frame type
 *    2     Sequence number - increments each frame
 *    3     Data length
 *    4     Control block checksum (bytes 0-3)
 *    .     Data
 *    .     Data
 *  Last    Data checksum inverted
 *
 * Copyright Debug Innovations Ltd. 2017-2023
 */

#include <stdint.h>
#include <string.h>
#include "frame_types.h"
#include "comms_frame.h"
#include "trace.h"


static uint8_t  tx_seq_num, rx_seq_num;


static uint8_t checksum8(uint8_t *data, uint8_t num_bytes)
{
    uint8_t  b, chksum;

    chksum = 0;

    for (b = 0; b < num_bytes; b++)
        chksum += *data++;

    return chksum;
}


/* @brief Initialises the frame module.
 *        Don't call this directly, the comms manager will do it.
 */
void frame_init(void)
{
    tx_seq_num = 0;
    rx_seq_num = 0;
}


/* @brief Constructs a frame in buff[] from the parameters supplied.
 *        The maximum possible size of buff[] is (255 + 6) bytes.
 *
 * @param[in]   frame_type   The frame type to be sent
 * @param[in]   data         The data to be sent, NULL if not required
 * @param[in]   num_bytes    The number of bytes of input data
 * @param[out]  buff         The constructed frame ready to be sent
 *
 * @returns The number of bytes in the constructed frame buff[]
 */
uint8_t frame_construct(frame_type_t frame_type, uint8_t *data, uint8_t num_bytes, uint8_t *buff)
{
    uint8_t  chksum;

    buff[0] = FRAME_START;
    buff[1] = (uint8_t)frame_type;
    buff[2] = ++tx_seq_num;
    buff[3] = num_bytes;

    chksum = checksum8(buff, FRAME_HEADER_LEN - 1);
    buff[FRAME_HEADER_LEN - 1] = chksum;

    if (data != NULL)
        memcpy(&buff[FRAME_HEADER_LEN], data, num_bytes);

    chksum = checksum8(&buff[FRAME_HEADER_LEN], num_bytes);
    buff[num_bytes + FRAME_HEADER_LEN] = (uint8_t)(~chksum);

    return num_bytes + FRAME_HEADER_LEN + 1;
}


/* @brief Extracts the frame header from buff[]
 *
 * @param[in]   buff         The received frame to be processed
 * @param[out]  frame_type   The frame type received
 * @param[out]  num_bytes    The number of bytes in data
 *
 * @returns:  0 : Frame header extracted successfully
 *            1 : Header checksum is bad
 *            2 : Out of sequence frame
 */
uint8_t frame_extract_header(uint8_t *buff, frame_type_t *frame_type, uint8_t *num_bytes)
{
    uint8_t  seq_num, chksum;

    *frame_type = FRAME_UNKNOWN;
    *num_bytes = 0;

    // Check the header checksum
    chksum = checksum8(buff, FRAME_HEADER_LEN - 1);
    if (buff[FRAME_HEADER_LEN - 1] != chksum)
    {
        // Bad header checksum
        trace(TRACE_ERROR, "Comms: Header checksum error");
        return 1;
    }

    *frame_type = (frame_type_t)buff[1];
    seq_num = buff[2];
    *num_bytes = buff[3];

    if (seq_num != ++rx_seq_num)
    {
        // Out of sequence frame
        if (*frame_type != FRAME_RESET)
            trace(TRACE_ERROR, "Comms: Sequence error");
        rx_seq_num = 0;
        return 2;
    }

    return 0;
}


/* @brief Extracts the frame data from buff[]
 *
 * @param[in]   buff       The received frame to be processed
 * @param[in]   num_bytes  The number of bytes in buff
 * @param[out]  data       The data extracted
 *
 * @returns:  0 : Frame data extracted successfully
 *            1 : Data checksum is bad
 */
uint8_t frame_extract_data(uint8_t *buff, uint8_t num_bytes, uint8_t *data)
{
    uint8_t  chksum;

    // Check the data checksum
    chksum = checksum8(buff, num_bytes);
    if (buff[num_bytes] != (uint8_t)(~chksum))
    {
        // Bad payload checksum
        trace(TRACE_ERROR, "Comms: Payload checksum error");
        return 1;
    }

    memcpy(data, buff, num_bytes);

    return 0;
}

