/*
 * Small ID
 * ========
 *
 * A simple ID verification system for devices that don't have enough
 * memory for a full AES/SHA encrypted ID soultion.
 *
 * Problem to be solved
 * --------------------
 * Making sure that our software only works with our hardware and that
 * clones of our hardware can't be sold at lower cost and used with
 * copies of our software.
 *
 * The software communicates with the firmware built into the hardware.
 * The software needs to authenticate that the firmware is genuine.
 *
 * Procedure
 * ---------
 * 1. A list of random numbers is shared between the two pieces of
 *    software.  The small_id_gen utility can be used to generate the
 *    list which by default is in small_id_table.h.  This is called
 *    the table.
 * 2. One end generates a 32-bit random number and sends it to the
 *    other end.  The function small_id_hash() picks a number of
 *    table values using the random number and combines them into a
 *    32-bit number which is sent back to the requestor.
 * 3. The requestor checks the returned value using the same procedure.
 *
 * Why it works
 * ------------
 * 1. For software to positively ID a device, the device must reply
 *    correctly to a random request.  A device that responds incorrectly
 *    will not pass the ID test.
 * 2. There are 4 billion possible random numbers so it is impractical
 *    for an observer to record all possible cases.  They don't know
 *    that the table is small and there are far less possible replies.
 * 3. Because the reply is a hashed version of the table contents, it
 *    is difficult to observe any pattern which may reveal the table.
 * 4. Replay attacks are not possible as the requestor generates a new
 *    request each time and having a box in the comms link that changes
 *    the request to a known one and replays the correct respose will
 *    not pass because the originator checks for a different respose.
 *
 * Caveats
 * -------
 * 1. Small ID is not as secure as a full AES/SHA ID soultion.
 * 2. The source code has to be kept secret.  In the case of a full
 *    AES/SHA solution, the source code can be public but the key is
 *    secret.  Small ID requires everything to be secret to be secure:
 *    The key used to generate the table, the file with the table,
 *    this source code and the knowledge of how it works.
 * 3. At least one side must not be copyable.  Two pieces of software
 *    running on a PC can be copied and will always pass this test.
 *    With a PC application talking to an embedded device, the device
 *    must be copy-protected and the code for that device must not be
 *    available for programming or download unless it is encrypted.
 *
 * Copyright Debug Innovations Ltd. 2023
 */

#include <stdint.h>
#include <stdlib.h>
#include "small_id.h"


/* @brief Generates a 32-bit random number.
 *        srand() must have been called first to seed the RNG.
 * @returns a 32-bit random number
 */
uint32_t small_id_rand32(void)
{
    uint64_t  u64;

    // rand() returns a value 0-32767 which is 15 bits so use
    // 3 x 15-bit values and pack them into a 64-bit variable
    u64 = (uint64_t)(rand() & 3) << 30;
    u64 |= (uint64_t)(rand() & 0x7FFF) << 15;
    u64 |= (uint64_t)(rand() & 0x7FFF);

    return (uint32_t)u64;
}


/* @brief Generates an output response value from an input value and a hash table.
 *        See the top of the file for more information.
 * @param[in]   value         Input value to be hashed
 * @param[in]   table         Small ID table to use
 * @param[in]   num_entries   Number of table entries
 * @returns the output response value
 */
uint32_t small_id_hash(uint32_t value, const uint8_t *table, int num_entries)
{
    uint32_t  hash, mash;
    uint8_t   i;
    int       index;

    hash = 0;

    for (i = 0; i < 4; i++)
    {
        hash <<= 8;

        // Take part of the input value as a table index
        index = (int)(value & 0x7FFF) % num_entries;

        // Add the table value into the hash
        hash |= table[index];

        value >>= 4;
    }

    // We have used half the input value to create a whole hash
    // but the table values are visible in the hash, so now use
    // the second half of the value to obfuscate the result
    mash = 0;

    for (i = 0; i < 4; i++)
    {
        mash <<= 8;

        // Take part of the input value as a table index
        index = (int)(value & 0x7FFF) % num_entries;

        // Add the table value into the mash
        mash |= table[index];

        value >>= 4;
    }

    // Obfuscate the returned hash
    hash ^= mash;

    return hash;
}

