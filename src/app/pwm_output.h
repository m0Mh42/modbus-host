/**
 * @file    pwm_output.h
 * @brief   2 PWM outputs whose duty is driven directly from Modbus registers.
 *
 * Duty is expressed as 0..PWM_MAX_DUTY (board_config) and rescaled onto the
 * timer's compare range. Writing a holding register calls pwm_output_set().
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

#endif /* PWM_OUTPUT_H */
