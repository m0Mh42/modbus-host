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

/** True if the channel reading is valid (probe present, no fault, enabled). */
bool thermocouple_is_valid(uint8_t channel);

/** DC offset in uV for CS1237 channels. */
int16_t thermocouple_get_offset(uint8_t channel);
void thermocouple_set_offset(uint8_t channel, int16_t offset_uv);

/** Latest cold-junction compensation temperature in 0.1 C (from DS18B20). */
int16_t thermocouple_get_cjc_temp_c10(void);

/** Channel enable bitmask (bits 0..7 for TC1..TC8). */
void thermocouple_set_enable_mask(uint16_t mask);
uint16_t thermocouple_get_enable_mask(void);

/** Get TC fault bitmask: bits 0..7 = TC1..8 open/invalid, bit 8 = CJC DS18B20 fault. */
uint16_t thermocouple_get_faults(void);

/** Clear latched TC faults. */
void thermocouple_clear_faults(void);

#endif /* THERMOCOUPLE_H */
