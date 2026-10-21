/**
 * @file    pwm_pid.c
 * @brief   Closed-loop PID controller for PWM outputs.
 */
#include "pwm_pid.h"
#include "app_config.h"
#include "pwm_output.h"
#include "proximity.h"
#include "analog_input.h"
#include "thermocouple.h"
#include "register_map.h"
#include "flash_eeprom.h"
#include <string.h>

typedef struct
{
    nv_pwm_pid_cfg_t cfg;
    int32_t integral;
    int32_t last_error;
    int32_t last_pv;
} pwm_pid_channel_t;

static pwm_pid_channel_t channels[NUM_PWM_PID_CHANNELS];

void pwm_pid_init(void)
{
    memset(channels, 0, sizeof(channels));
    const nv_config_t *nv = flash_eeprom_get();
    bool has_cfg = (nv->magic == NV_CONFIG_MAGIC);

    for (uint8_t i = 0; i < NUM_PWM_PID_CHANNELS; i++)
    {
        if (has_cfg)
        {
            channels[i].cfg = nv->pwm_pid[i];
            if (channels[i].cfg.max_duty == 0 && channels[i].cfg.kp == 0)
            {
                channels[i].cfg.max_duty = 1000;
                channels[i].cfg.kp = 100;
                channels[i].cfg.ki = 10;
            }
            if (channels[i].cfg.scale_mul == 0)
            {
                channels[i].cfg.scale_mul = 1000;
            }
            if (channels[i].cfg.scale_div == 0)
            {
                channels[i].cfg.scale_div = 1000;
            }
        }
        else
        {
            channels[i].cfg.mode = 0;
            channels[i].cfg.feedback_src = 0;
            channels[i].cfg.setpoint = 0;
            channels[i].cfg.kp = 100;
            channels[i].cfg.ki = 10;
            channels[i].cfg.kd = 0;
            channels[i].cfg.min_duty = 0;
            channels[i].cfg.max_duty = 1000;
            channels[i].cfg.scale_mul = 1000;
            channels[i].cfg.scale_div = 1000;
        }
        channels[i].integral = 0;
        channels[i].last_error = 0;
        channels[i].last_pv = 0;
    }
}

void pwm_pid_process(void)
{
    bool running = register_map_is_machine_running();

    for (uint8_t ch = 0; ch < NUM_PWM_PID_CHANNELS; ch++)
    {
        pwm_pid_channel_t *p = &channels[ch];

        if (p->cfg.mode != 1)
        {
            /* Manual mode: duty is written directly to REG_HOLD_PWMx_DUTY */
            p->integral = 0;
            p->last_error = 0;
            continue;
        }

        if (!running)
        {
            /* Machine stopped: safe zero output */
            pwm_output_set(ch, 0);
            p->integral = 0;
            p->last_error = 0;
            continue;
        }

        /* 1. Get raw process variable (PV raw) from selected feedback source */
        int32_t pv_raw = 0;
        switch (p->cfg.feedback_src)
        {
        case PWM_PID_SRC_LINE_SPEED:
            pv_raw = (int32_t)proximity_get_line_speed_c10();
            break;
        case PWM_PID_SRC_AIN1:
            pv_raw = (int32_t)analog_input_get_mv(0);
            break;
        case PWM_PID_SRC_AIN2:
            pv_raw = (int32_t)analog_input_get_mv(1);
            break;
        case PWM_PID_SRC_PROX1_HZ:
            pv_raw = (int32_t)proximity_get_speed(PROX_CHANNEL_1);
            break;
        case PWM_PID_SRC_PROX2_HZ:
            pv_raw = (int32_t)proximity_get_speed(PROX_CHANNEL_2);
            break;
        default:
            if (p->cfg.feedback_src >= PWM_PID_SRC_TC1 && p->cfg.feedback_src <= PWM_PID_SRC_TC8)
            {
                uint8_t tc_idx = (uint8_t)(p->cfg.feedback_src - PWM_PID_SRC_TC1);
                pv_raw = (int32_t)thermocouple_get_temp(tc_idx);
            }
            else
            {
                pv_raw = (int32_t)proximity_get_line_speed_c10();
            }
            break;
        }

        /* 2. Scale raw PV to user engineering units */
        uint32_t mul = (p->cfg.scale_mul > 0) ? p->cfg.scale_mul : 1000;
        uint32_t div = (p->cfg.scale_div > 0) ? p->cfg.scale_div : 1000;
        int32_t pv;
        if (mul == div)
        {
            pv = pv_raw;
        }
        else
        {
            int64_t num = (int64_t)pv_raw * (int64_t)mul;
            int64_t den = (int64_t)div;
            if (num >= 0)
            {
                pv = (int32_t)((num + (den / 2)) / den);
            }
            else
            {
                pv = (int32_t)((num - (den / 2)) / den);
            }
        }
        p->last_pv = pv;

        /* 3. Determine target setpoint (SP) in user engineering units */
        int32_t sp = (int32_t)p->cfg.setpoint;
        if (sp == 0 && p->cfg.feedback_src == PWM_PID_SRC_LINE_SPEED)
        {
            /* 0 indicates auto-tracking master machine line speed setpoint */
            int32_t line_sp_raw = (int32_t)proximity_get_speed_setpoint();
            if (mul == div)
            {
                sp = line_sp_raw;
            }
            else
            {
                int64_t num = (int64_t)line_sp_raw * (int64_t)mul;
                int64_t den = (int64_t)div;
                sp = (int32_t)((num + (den / 2)) / den);
            }
        }

        /* 4. Error calculation in engineering units */
        int32_t error = sp - pv;

        /* 5. PID loop computation */
        int32_t p_term = (error * (int32_t)p->cfg.kp) / 100;

        p->integral += (error * CONTROL_LOOP_PERIOD_MS) / 1000;
        int32_t max_i = (1000 * 100) / (p->cfg.ki > 0 ? (int32_t)p->cfg.ki : 1);
        if (p->integral > max_i)
            p->integral = max_i;
        if (p->integral < -max_i)
            p->integral = -max_i;
        int32_t i_term = (p->integral * (int32_t)p->cfg.ki) / 100;

        int32_t d_term = 0;
        if (CONTROL_LOOP_PERIOD_MS > 0 && p->cfg.kd > 0)
        {
            int32_t derivative = (error - p->last_error) * 1000 / CONTROL_LOOP_PERIOD_MS;
            d_term = (derivative * (int32_t)p->cfg.kd) / 100;
        }
        p->last_error = error;

        int32_t out = p_term + i_term + d_term;

        /* 6. Clamp to min/max output duty */
        int32_t min_lim = (int32_t)p->cfg.min_duty;
        int32_t max_lim = (int32_t)p->cfg.max_duty;
        if (max_lim > 1000)
            max_lim = 1000;
        if (min_lim < 0)
            min_lim = 0;
        if (out > max_lim)
            out = max_lim;
        if (out < min_lim)
            out = min_lim;

        pwm_output_set(ch, (uint16_t)out);
    }
}

int32_t pwm_pid_get_pv(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].last_pv : 0;
}

/* Modbus Getters */
uint16_t pwm_pid_get_mode(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.mode : 0;
}
uint16_t pwm_pid_get_src(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.feedback_src : 0;
}
uint16_t pwm_pid_get_setpoint(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.setpoint : 0;
}
uint16_t pwm_pid_get_kp(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.kp : 0;
}
uint16_t pwm_pid_get_ki(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.ki : 0;
}
uint16_t pwm_pid_get_kd(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.kd : 0;
}
uint16_t pwm_pid_get_min_duty(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.min_duty : 0;
}
uint16_t pwm_pid_get_max_duty(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.max_duty : 1000;
}
uint16_t pwm_pid_get_scale_mul(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.scale_mul : 1000;
}
uint16_t pwm_pid_get_scale_div(uint8_t ch)
{
    return (ch < NUM_PWM_PID_CHANNELS) ? channels[ch].cfg.scale_div : 1000;
}

/* Modbus Setters */
void pwm_pid_set_mode(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.mode = (val > 0) ? 1 : 0;
        flash_eeprom_get_mutable()->pwm_pid[ch].mode = channels[ch].cfg.mode;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_src(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.feedback_src = (val > 12) ? 0 : val;
        flash_eeprom_get_mutable()->pwm_pid[ch].feedback_src = channels[ch].cfg.feedback_src;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_setpoint(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.setpoint = val;
        flash_eeprom_get_mutable()->pwm_pid[ch].setpoint = val;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_kp(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.kp = val;
        flash_eeprom_get_mutable()->pwm_pid[ch].kp = val;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_ki(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.ki = val;
        flash_eeprom_get_mutable()->pwm_pid[ch].ki = val;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_kd(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.kd = val;
        flash_eeprom_get_mutable()->pwm_pid[ch].kd = val;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_min_duty(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.min_duty = (val > 1000) ? 1000 : val;
        flash_eeprom_get_mutable()->pwm_pid[ch].min_duty = channels[ch].cfg.min_duty;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_max_duty(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.max_duty = (val > 1000) ? 1000 : val;
        flash_eeprom_get_mutable()->pwm_pid[ch].max_duty = channels[ch].cfg.max_duty;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_scale_mul(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.scale_mul = (val == 0) ? 1000 : val;
        flash_eeprom_get_mutable()->pwm_pid[ch].scale_mul = channels[ch].cfg.scale_mul;
        flash_eeprom_mark_dirty();
    }
}
void pwm_pid_set_scale_div(uint8_t ch, uint16_t val)
{
    if (ch < NUM_PWM_PID_CHANNELS)
    {
        channels[ch].cfg.scale_div = (val == 0) ? 1000 : val;
        flash_eeprom_get_mutable()->pwm_pid[ch].scale_div = channels[ch].cfg.scale_div;
        flash_eeprom_mark_dirty();
    }
}
