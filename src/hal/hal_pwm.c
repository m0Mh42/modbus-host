/**
 * @file hal_pwm.c
 * @brief Timer-PWM abstraction implementation (GD32F30x SPL).
 *
 * Sets up PWM_TIMER in edge-aligned PWM mode with period PWM_PERIOD on the
 * two channels routed to PWM1/PWM2. Duty is changed at runtime via the
 * compare value. Output frequency = timer_clock / ((prescaler+1)*(period+1)).
 */
#include "hal_pwm.h"
#include "hal_gpio.h"
#include "board_config.h"
#include "gd32f30x.h"

/* Map the configured PWM_TIMER to its RCU clock. Extend if you change timers. */
static rcu_periph_enum pwm_timer_rcu(void)
{
    switch (PWM_TIMER)
    {
    case TIMER0:
        return RCU_TIMER0;
    case TIMER1:
        return RCU_TIMER1;
    case TIMER2:
        return RCU_TIMER2;
    case TIMER3:
        return RCU_TIMER3;
    default:
        return RCU_TIMER2;
    }
}

static void config_channel(uint16_t channel)
{
    timer_oc_parameter_struct oc;
    timer_channel_output_struct_para_init(&oc);
    oc.outputstate = TIMER_CCX_ENABLE;
    oc.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_channel_output_config(PWM_TIMER, channel, &oc);

    timer_channel_output_pulse_value_config(PWM_TIMER, channel, 0U); /* duty 0 */
    timer_channel_output_mode_config(PWM_TIMER, channel, TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(PWM_TIMER, channel, TIMER_OC_SHADOW_DISABLE);
}

void hal_pwm_init(void)
{
    rcu_periph_clock_enable(pwm_timer_rcu());
    rcu_periph_clock_enable(RCU_AF);

    /* PWM output pins as alternate-function push-pull. */
    hal_gpio_init(PWM1_PORT, PWM1_PIN, HAL_GPIO_ALTERNATE);
    hal_gpio_init(PWM2_PORT, PWM2_PIN, HAL_GPIO_ALTERNATE);

    /* Time base. Prescaler chosen so the counter ticks at ~1 MHz; with
     * PWM_PERIOD=999 that yields ~1 kHz PWM. Tune prescaler for your clock. */
    timer_parameter_struct tp;
    timer_struct_para_init(&tp);
    tp.prescaler = (SystemCoreClock / 1000000U) - 1U; /* -> 1 MHz tick */
    tp.alignedmode = TIMER_COUNTER_EDGE;
    tp.counterdirection = TIMER_COUNTER_UP;
    tp.period = PWM_PERIOD;
    tp.clockdivision = TIMER_CKDIV_DIV1;
    tp.repetitioncounter = 0U;
    timer_init(PWM_TIMER, &tp);

    config_channel(PWM1_TIMER_CHANNEL);
    config_channel(PWM2_TIMER_CHANNEL);

    timer_auto_reload_shadow_enable(PWM_TIMER);
    timer_enable(PWM_TIMER);
}

void hal_pwm_set_compare(uint32_t timer, uint16_t channel, uint16_t compare)
{
    if (compare > PWM_PERIOD)
        compare = PWM_PERIOD;
    timer_channel_output_pulse_value_config(timer, channel, compare);
}
