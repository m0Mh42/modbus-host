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
