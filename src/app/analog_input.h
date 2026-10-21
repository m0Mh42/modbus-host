/**
 * @file    analog_input.h
 * @brief   2 divided analog inputs -> millivolts, exposed to Modbus.
 */
#ifndef ANALOG_INPUT_H
#define ANALOG_INPUT_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize ADC GPIO pins for AIN1 and AIN2. */
void analog_input_init(void);

/** Periodic task: sample ADC channels and calculate mV using divider ratio. */
void analog_input_task(void);

/** Latest divider-corrected voltage for channel 0 or 1, in millivolts. */
uint16_t analog_input_get_mv(uint8_t channel);

/** Raw ADC code (0..4095). */
uint16_t analog_input_get_raw(uint8_t channel);

/** Set/get voltage divider ratio scaled by 1000 (e.g. 4917 for 4.917). */
void analog_input_set_divider_ratio_x1000(uint8_t channel, uint16_t ratio_x1000);
uint16_t analog_input_get_divider_ratio_x1000(uint8_t channel);

/** Set/get channel enable bitmask (bit 0 = AIN1, bit 1 = AIN2). */
void analog_input_set_enable_mask(uint16_t mask);
uint16_t analog_input_get_enable_mask(void);

/** Get AIN fault bitmask (bit 0 = AIN1 overvoltage, bit 1 = AIN2 overvoltage). */
uint16_t analog_input_get_faults(void);

/** Clear latched AIN faults. */
void analog_input_clear_faults(void);

#endif /* ANALOG_INPUT_H */
