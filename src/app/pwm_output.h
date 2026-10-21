/**
 * @file    pwm_output.h
 * @brief   2 PWM outputs whose duty is driven directly from Modbus registers.
 */
#ifndef PWM_OUTPUT_H
#define PWM_OUTPUT_H

#include <stdint.h>

/** Initialize the PWM timer/channels (both outputs at 0%). */
void pwm_output_init(void);

/** Set duty for channel 0..1, value 0..PWM_MAX_DUTY (clamped). */
void pwm_output_set(uint8_t channel, uint16_t duty);

/** Read back the last duty set for channel 0..1. */
uint16_t pwm_output_get(uint8_t channel);

/** Set/get PWM channel enable bitmask (bit 0 = PWM1, bit 1 = PWM2). */
void pwm_output_set_enable_mask(uint16_t mask);
uint16_t pwm_output_get_enable_mask(void);

#endif /* PWM_OUTPUT_H */
