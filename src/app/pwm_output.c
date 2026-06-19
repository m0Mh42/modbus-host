/**
 * @file pwm_output.c
 * @brief 2 Modbus-driven PWM outputs.
 *
 * compare = duty * PWM_PERIOD / PWM_MAX_DUTY
 */
#include "pwm_output.h"
#include "app_config.h"
#include "board_config.h"
#include "hal_pwm.h"

static const uint16_t pwm_channel[NUM_PWM_OUTPUTS] = {
    PWM1_TIMER_CHANNEL,
    PWM2_TIMER_CHANNEL,
};

static uint16_t pwm_duty[NUM_PWM_OUTPUTS];

void pwm_output_init(void)
{
    hal_pwm_init();
    for (uint8_t i = 0; i < NUM_PWM_OUTPUTS; i++) {
        pwm_duty[i] = 0;
        hal_pwm_set_compare(PWM_TIMER, pwm_channel[i], 0);
    }
}

void pwm_output_set(uint8_t channel, uint16_t duty)
{
    if (channel >= NUM_PWM_OUTPUTS) return;
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    pwm_duty[channel] = duty;

    /* Rescale 0..PWM_MAX_DUTY -> 0..PWM_PERIOD compare value. */
    uint32_t compare = (uint32_t)duty * PWM_PERIOD / PWM_MAX_DUTY;
    hal_pwm_set_compare(PWM_TIMER, pwm_channel[channel], (uint16_t)compare);
}

uint16_t pwm_output_get(uint8_t channel)
{
    if (channel >= NUM_PWM_OUTPUTS) return 0;
    return pwm_duty[channel];
}
