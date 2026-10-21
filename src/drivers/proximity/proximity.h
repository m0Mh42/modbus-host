/**
 * @file    proximity.h
 * @brief   Two proximity-sensor inputs used for meterage counting and speed.
 */
#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>

typedef enum
{
    PROX_CHANNEL_1 = 0,
    PROX_CHANNEL_2 = 1,
} prox_channel_t;

/** Configure both proximity inputs (and their EXTI lines). */
void proximity_init(void);

/** ISR hook: call on each detected edge for the given channel. */
void proximity_on_edge(prox_channel_t ch);

/** Periodic task: ages out speed to zero when pulses stop. */
void proximity_task(void);

/** Total accumulated pulse count (meterage). */
uint32_t proximity_get_count(prox_channel_t ch);

/** Most recent measured speed (pulses/sec or device units). */
uint32_t proximity_get_speed(prox_channel_t ch);

/** Reset the meterage counter. */
void proximity_reset_count(prox_channel_t ch);

/** Get accumulated line meterage in 0.1 m resolution (decimeters). */
uint32_t proximity_get_meterage_c10(void);

/** Get accumulated line meterage in whole meters. */
uint32_t proximity_get_meterage_m(void);

/** Reset the meterage counter (and pulse counts). */
void proximity_reset_meterage(void);

/** Pulses per meter calibration factor (default 100). */
void proximity_set_pulses_per_meter(uint16_t ppm);
uint16_t proximity_get_pulses_per_meter(void);

/** Set/get proximity channel enable bitmask (bit 0 = PROX1, bit 1 = PROX2). */
void proximity_set_enable_mask(uint16_t mask);
uint16_t proximity_get_enable_mask(void);

/** Speed input source: 0=PROX1, 1=PROX2, 2=BOTH (average). */
void proximity_set_speed_src(uint16_t src);
uint16_t proximity_get_speed_src(void);

/** Set speed input (target line speed, e.g. 0.1 m/min). */
void proximity_set_speed_setpoint(uint16_t sp);
uint16_t proximity_get_speed_setpoint(void);

/** Get effective frequency in Hz based on selected source. */
uint32_t proximity_get_effective_speed_hz(void);

/** Get converted line speed in 0.1 m/min: ((uint64_t)hz * 600ULL) / (uint64_t)ppm. */
uint16_t proximity_get_line_speed_c10(void);

/** Get proximity fault bitmask: bit 0 = PROX1 overspeed, bit 1 = PROX2 overspeed. */
uint16_t proximity_get_faults(void);

/** Clear latched proximity faults. */
void proximity_clear_faults(void);

#endif /* PROXIMITY_H */
