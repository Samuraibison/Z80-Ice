/**
 * @brief  Interface to PIC internal EEPROM read/write functions
 *
 * Copyright Debug Innovations Ltd. 2010-2021
 */

// Read a byte from location 'addr'
uint8_t ee_read(uint16_t addr);

// Write a byte 'data' to location 'addr'
void ee_write(uint16_t addr, uint8_t data);

