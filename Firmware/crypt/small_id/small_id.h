/*
 * Interface to small_id system
 *
 * Copyright Debug Innovations Ltd. 2023
 */


/* Make it usable with C and C++ */
#ifdef __cplusplus
extern "C" {
#endif


/* @brief Generates a 32-bit random number.
 *        srand() must have been called first to seed the RNG.
 * @returns a 32-bit random number
 */
uint32_t small_id_rand32(void);


/* @brief Generates an output response value from an input value and a hash table.
 *        See the top of small_id.c for more information.
 * @param[in]   value         Input value to be hashed
 * @param[in]   table         Small ID table to use
 * @param[in]   num_entries   Number of table entries
 * @returns the output response value
 */
uint32_t small_id_hash(uint32_t value, const uint8_t *table, int num_entries);


#ifdef __cplusplus
}
#endif

