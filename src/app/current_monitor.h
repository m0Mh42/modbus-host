/**
 * @file    current_monitor.h
 * @brief   8-channel heater-current monitoring via current transformers (True RMS).
 */
#ifndef CURRENT_MONITOR_H
#define CURRENT_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize the ADC channels for the 8 CT inputs. */
void current_monitor_init(void);

/** Periodic task: sample channels over AC mains cycle and calculate True RMS. */
void current_monitor_task(void);

/** Latest True RMS current for channel 0..7, in milliamps. */
uint16_t current_monitor_get(uint8_t channel);

/** Set/get CT voltage bias in millivolts (nominally 2495 mV from TL431). */
void current_monitor_set_bias_mv(uint16_t bias_mv);
uint16_t current_monitor_get_bias_mv(void);

/** Set/get CT turns ratio (e.g. 2375). */
void current_monitor_set_turns_ratio(uint16_t ratio);
uint16_t current_monitor_get_turns_ratio(void);

/** Set/get CT burden resistance in 0.1 ohm (e.g. 620 for 62.0 ohm). */
void current_monitor_set_burden_ohm_x10(uint16_t burden_x10);
uint16_t current_monitor_get_burden_ohm_x10(void);

/** Set/get CT channel enable mask (bits 0..7 enable CT1..CT8). */
void current_monitor_set_enable_mask(uint16_t mask);
uint16_t current_monitor_get_enable_mask(void);

/** Get CT fault bitmask: bits 0..7 = CT1..8 overcurrent/saturation, bit 8 = bias fault. */
uint16_t current_monitor_get_faults(void);

/** Clear latched CT faults. */
void current_monitor_clear_faults(void);

#endif /* CURRENT_MONITOR_H */
