/**
 * @file cs1237.h
 * @brief CS1237 24-bit ADC driver using bit-banging with shared SCLK.
 */
#ifndef CS1237_H
#define CS1237_H

#include <stdint.h>
#include <stdbool.h>

#define CS1237_SPEED_10    0b00000000
#define CS1237_SPEED_40    0b00010000
#define CS1237_SPEED_640   0b00100000
#define CS1237_SPEED_1280  0b00110000

#define CS1237_PGA_1       0b00000000
#define CS1237_PGA_2       0b00000100
#define CS1237_PGA_64      0b00001000
#define CS1237_PGA_128     0b00001100

#define CS1237_CHANNEL_A    0b00000000
#define CS1237_CHANNEL_TEMP 0b00000010

#define CS1237_REF_INTERNAL 0b00000000
#define CS1237_REF_EXTERNAL 0b01000000

/**
 * @brief Initialize GPIOs for SCLK and DOUTs.
 */
void cs1237_init(void);

/**
 * @brief Configure one CS1237 chip (blocks for its DRDY).
 * @param ch Index of the chip (0..7)
 * @param gain PGA gain macro
 * @param speed Speed macro
 * @param channel Channel and Ref macro
 * @return true if successful, false on timeout.
 */
bool cs1237_configure_one(uint8_t ch, uint8_t gain, uint8_t speed, uint8_t channel);

/**
 * @brief Sequentially scan all chips. Blocks until all are read or timeout.
 * @param raw_out Array of 8 int32_t to store the raw 24-bit readings.
 * @return true if all chips were read, false if any timed out.
 */
bool cs1237_scan_all(int32_t raw_out[]);

#endif /* CS1237_H */
