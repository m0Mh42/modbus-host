/**
 * @file    thermocouple.h
 * @brief   8-channel thermocouple temperature service.
 *
 * Sits on top of the two ADS1220 drivers: scans channels round-robin,
 * applies cold-junction compensation and the thermocouple polynomial to
 * produce a temperature per channel.
 */
#ifndef THERMOCOUPLE_H
#define THERMOCOUPLE_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize both ADS1220 devices and internal state. */
void thermocouple_init(void);

/** Periodic task: advance the channel scan, read ready conversions. */
void thermocouple_task(void);

/** Latest temperature in 0.1 C units for channel 0..7. */
int16_t thermocouple_get_temp(uint8_t channel);

/** True if the channel reading is valid (probe present, no fault). */
bool thermocouple_is_valid(uint8_t channel);

#endif /* THERMOCOUPLE_H */
