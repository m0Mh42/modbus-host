/**
 * @file    proximity.h
 * @brief   Two proximity-sensor inputs used for meterage counting and speed.
 *
 * Each edge increments a pulse counter; speed is derived from pulse interval.
 * Intended to be fed by GPIO EXTI interrupts.
 */
#ifndef PROXIMITY_H
#define PROXIMITY_H

#include <stdint.h>

typedef enum {
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

#endif /* PROXIMITY_H */
