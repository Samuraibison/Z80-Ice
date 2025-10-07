/*
 * Interface to frame construct/extract functions for simple serial comms links
 *
 * Copyright Debug Innovations Ltd. 2017-2023
 */

// Make it usable with C and C++
#ifdef __cplusplus
extern "C" {
#endif


// Start of frame character
#define FRAME_START         0xAA

// Size of the frame header up to the header checksum
#define FRAME_HEADER_LEN    5


/* @brief Initialises the frame module.
 *        Don't call this directly, the comms manager will do it.
 */
void frame_init(void);


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
uint8_t frame_construct(frame_type_t frame_type, uint8_t *data, uint8_t num_bytes, uint8_t *buff);


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
uint8_t frame_extract_header(uint8_t *buff, frame_type_t *frame_type, uint8_t *num_bytes);


/* @brief Extracts the frame data from buff[]
 *
 * @param[in]   buff       The received frame to be processed
 * @param[in]   num_bytes  The number of bytes in buff
 * @param[out]  data       The data extracted
 *
 * @returns:  0 : Frame data extracted successfully
 *            1 : Data checksum is bad
 */
uint8_t frame_extract_data(uint8_t *buff, uint8_t num_bytes, uint8_t *data);


#ifdef __cplusplus
}
#endif

