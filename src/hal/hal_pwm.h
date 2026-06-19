/**
 * @file    hal_pwm.h
 * @brief   Timer-PWM abstraction for the 2 PWM outputs.
 */
#ifndef HAL_PWM_H
#define HAL_PWM_H

#include <stdint.h>

/** Initialize the PWM timer and both output channels (duty = 0). */
void hal_pwm_init(void);

/** Set compare value (0..PWM_PERIOD) for a timer channel. */
void hal_pwm_set_compare(uint32_t timer, uint16_t channel, uint16_t compare);

#endif /* HAL_PWM_H */
