/**
 * @file    thermocouple.h
 * @brief   8-channel thermocouple temperature service.
 */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize CS1237 devices. */
void thermocouple_init(void);

/** Periodic task: scan CS1237 channels. */
void thermocouple_task(void);

/** Latest temperature in 0.1 C units for channel 0..7. */
int16_t thermocouple_get_temp(uint8_t channel);

/** True if the channel reading is valid (probe present, no fault). */
bool thermocouple_is_valid(uint8_t channel);

/** DC offset in uV for CS1237 channels. */
int16_t thermocouple_get_offset(uint8_t channel);
void thermocouple_set_offset(uint8_t channel, int16_t offset_uv);

#endif /* THERMOCOUPLE_H */
