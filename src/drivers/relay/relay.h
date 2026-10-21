/**
 * @file    relay.h
 * @brief   16-channel relay bank (heater / blower) via ULN2803 drivers.
 *
 * Channels are 0..15. The physical port/pin for each is held in a table in
 * relay.c, keyed off board_config.h.
 */
#ifndef RELAY_H
#define RELAY_H

#include <stdint.h>
#include <stdbool.h>

/* Mask for odd-numbered physical relays: Relay 1, 3, 5, 7, 9, 11, 13, 15
 * (Logical channels: 0, 2, 4, 6, 8, 10, 12, 14) -> 0b0101010101010101 */
#define RELAY_MASK_ODD_RELAYS 0x5555U

/* Mask for even-numbered physical relays: Relay 2, 4, 6, 8, 10, 12, 14, 16
 * (Logical channels: 1, 3, 5, 7, 9, 11, 13, 15) -> 0b1010101010101010 */
#define RELAY_MASK_EVEN_RELAYS 0xAAAAU

/** Early initialization: releases JTAG remap and forces all relays off immediately. */
void relay_early_init(void);

/** Configure all relay output pins (de-energized). */
void relay_init(void);

/** Set a single relay (channel 0..15). */
void relay_set(uint8_t channel, bool on);

/** Read back the cached state of a relay. */
bool relay_get(uint8_t channel);

/** Apply a 16-bit mask in one call (bit n -> relay n). */
void relay_set_mask(uint16_t mask);

/** Current 16-bit state mask. */
uint16_t relay_get_mask(void);

/** Force all relays off. */
void relay_all_off(void);

#endif /* RELAY_H */
