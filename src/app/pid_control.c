/**
 * @file    pid_control.c
 * @brief   PID temperature control and software PWM relay driver.
 */
#include "pid_control.h"
#include "app_config.h"
#include "thermocouple.h"
#include "relay.h"
#include "pwm_output.h"
#include "flash_eeprom.h"
#include "hal_time.h"
#include <string.h>

#define SOFT_PWM_PERIOD_MS 2000 /* 2 seconds default software PWM period */
#define PID_OUT_MAX 1000        /* 100% duty cycle */
#define PID_OUT_MIN -1000       /* -100% duty cycle (blower) */

typedef struct
{
    uint16_t mode;         /* 0 = Manual, 1 = Auto (PID), 2 = ON/OFF */
    int16_t setpoint;      /* 0.1 C */
    uint16_t kp;           /* Kp * 100 */
    uint16_t ki;           /* Ki * 100 */
    uint16_t kd;           /* Kd * 100 */
    uint16_t deadzone;     /* 0.1 C */
    uint16_t heater_relay; /* 0..15 (Relay), 0x0020 (PWM1), 0x0021 (PWM2), 0xFFFF (None) */
    uint16_t blower_relay; /* 0..15, or 0xFFFF for none */
    int16_t manual_duty;   /* -1000..1000 */

    /* Extended Thermocontroller Parameters */
    uint16_t action;         /* 0 = Heating, 1 = Cooling */
    uint16_t hysteresis;     /* 0.1 C */
    uint16_t cycle_time_s;   /* Proportional cycle time in seconds (1..100s) */
    int16_t sv_high_limit;   /* 0.1 C */
    int16_t sv_low_limit;    /* 0.1 C */
    int16_t pv_offset;       /* 0.1 C */
    int16_t alarm_high;      /* 0.1 C */
    int16_t alarm_low;       /* 0.1 C */
    uint16_t fail_safe_duty; /* 0..1000 */

    /* Internal State */
    int32_t integral;
    int16_t last_error;
    int32_t current_output; /* -1000..1000 */
    bool on_off_state;      /* true = energized, false = de-energized */
} pid_channel_t;

static pid_channel_t pid_channels[NUM_THERMOCOUPLES];

void pid_control_init(void)
{
    memset(pid_channels, 0, sizeof(pid_channels));
    const nv_config_t *cfg = flash_eeprom_get();
    bool has_cfg = (cfg->magic == NV_CONFIG_MAGIC);

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        if (has_cfg)
        {
            pid_channels[i].mode = cfg->pid[i].mode;
            pid_channels[i].setpoint = cfg->pid[i].setpoint;
            pid_channels[i].kp = cfg->pid[i].kp;
            pid_channels[i].ki = cfg->pid[i].ki;
            pid_channels[i].kd = cfg->pid[i].kd;
            pid_channels[i].deadzone = cfg->pid[i].deadzone;
            uint16_t hr = cfg->pid[i].heater_relay;
            if (hr >= NUM_RELAYS && hr != PID_ACTUATOR_NONE)
            {
                hr = PID_ACTUATOR_NONE; /* Disallow legacy PWM in thermal zones */
            }
            pid_channels[i].heater_relay = hr;

            uint16_t br = cfg->pid[i].blower_relay;
            if (br >= NUM_RELAYS && br != PID_ACTUATOR_NONE)
            {
                br = PID_ACTUATOR_NONE;
            }
            pid_channels[i].blower_relay = br;
            pid_channels[i].manual_duty = cfg->pid[i].manual_duty;

            pid_channels[i].action = cfg->pid[i].action;
            pid_channels[i].hysteresis = (cfg->pid[i].hysteresis > 0) ? cfg->pid[i].hysteresis : 20;
            pid_channels[i].cycle_time_s = (cfg->pid[i].cycle_time_s > 0) ? cfg->pid[i].cycle_time_s : 2;
            pid_channels[i].sv_high_limit = (cfg->pid[i].sv_high_limit != 0) ? cfg->pid[i].sv_high_limit : 13700;
            pid_channels[i].sv_low_limit = (cfg->pid[i].sv_low_limit != 0) ? cfg->pid[i].sv_low_limit : -2000;
            pid_channels[i].pv_offset = cfg->pid[i].pv_offset;
            pid_channels[i].alarm_high = (cfg->pid[i].alarm_high != 0) ? cfg->pid[i].alarm_high : 13700;
            pid_channels[i].alarm_low = (cfg->pid[i].alarm_low != 0) ? cfg->pid[i].alarm_low : -2000;
            pid_channels[i].fail_safe_duty = cfg->pid[i].fail_safe_duty;
        }
        else
        {
            pid_channels[i].mode = 0;
            pid_channels[i].setpoint = 250;
            pid_channels[i].kp = 1000;
            pid_channels[i].ki = 50;
            pid_channels[i].kd = 100;
            pid_channels[i].deadzone = 20;
            pid_channels[i].heater_relay = i;
            pid_channels[i].blower_relay = (i < 8) ? (i + 8) : 0xFFFF;
            pid_channels[i].manual_duty = 0;

            pid_channels[i].action = PID_ACTION_HEATING;
            pid_channels[i].hysteresis = 20;
            pid_channels[i].cycle_time_s = 2;
            pid_channels[i].sv_high_limit = 13700;
            pid_channels[i].sv_low_limit = -2000;
            pid_channels[i].pv_offset = 0;
            pid_channels[i].alarm_high = 13700;
            pid_channels[i].alarm_low = -2000;
            pid_channels[i].fail_safe_duty = 0;
        }
        pid_channels[i].integral = 0;
        pid_channels[i].last_error = 0;
        pid_channels[i].current_output = 0;
        pid_channels[i].on_off_state = false;
    }
}

void pid_control_process(void)
{
    uint32_t now = hal_millis();

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        pid_channel_t *ch = &pid_channels[i];

        if (ch->mode == PID_MODE_MANUAL)
        {
            /* Manual Mode */
            ch->current_output = ch->manual_duty;
            ch->integral = 0;
        }
        else if (ch->mode == PID_MODE_ON_OFF)
        {
            /* ON/OFF Mode with Hysteresis (Bang-Bang) */
            if (!thermocouple_is_valid(i))
            {
                ch->current_output = ch->fail_safe_duty;
            }
            else
            {
                int16_t current_temp = thermocouple_get_temp(i) + ch->pv_offset;
                int16_t hyst = (ch->hysteresis > 0) ? (int16_t)ch->hysteresis : (int16_t)ch->deadzone;
                if (hyst < 1)
                    hyst = 10; /* At least 1.0 C default */

                if (ch->action == PID_ACTION_HEATING)
                {
                    /* Reverse acting: Heating */
                    if (current_temp <= (ch->setpoint - hyst / 2))
                    {
                        ch->on_off_state = true;
                    }
                    else if (current_temp >= (ch->setpoint + hyst / 2))
                    {
                        ch->on_off_state = false;
                    }
                }
                else
                {
                    /* Direct acting: Cooling */
                    if (current_temp >= (ch->setpoint + hyst / 2))
                    {
                        ch->on_off_state = true;
                    }
                    else if (current_temp <= (ch->setpoint - hyst / 2))
                    {
                        ch->on_off_state = false;
                    }
                }
                ch->current_output = ch->on_off_state ? PID_OUT_MAX : 0;
            }
        }
        else
        {
            /* Auto Mode (PID) */
            if (!thermocouple_is_valid(i))
            {
                /* Sensor fault - fail safe output */
                ch->current_output = ch->fail_safe_duty;
                ch->integral = 0;
            }
            else
            {
                int16_t current_temp = thermocouple_get_temp(i) + ch->pv_offset;
                int16_t error;
                if (ch->action == PID_ACTION_HEATING)
                {
                    error = ch->setpoint - current_temp;
                }
                else
                {
                    error = current_temp - ch->setpoint;
                }

                /* Apply Deadzone */
                if (error > 0 && error < ch->deadzone)
                {
                    error = 0;
                }
                else if (error < 0 && error > -(int16_t)ch->deadzone)
                {
                    error = 0;
                }

                int32_t p_term = ((int32_t)error * ch->kp) / 100;

                ch->integral += ((int32_t)error * CONTROL_LOOP_PERIOD_MS) / 1000;

                /* Anti-windup */
                int32_t max_integral = (PID_OUT_MAX * 100) / (ch->ki > 0 ? ch->ki : 1);
                if (ch->integral > max_integral)
                    ch->integral = max_integral;
                if (ch->integral < -max_integral)
                    ch->integral = -max_integral;

                int32_t i_term = (ch->integral * ch->ki) / 100;

                int32_t derivative = (error - ch->last_error) * 1000 / CONTROL_LOOP_PERIOD_MS;
                int32_t d_term = (derivative * ch->kd) / 100;

                ch->last_error = error;

                int32_t output = p_term + i_term + d_term;

                if (output > PID_OUT_MAX)
                    output = PID_OUT_MAX;
                if (output < PID_OUT_MIN)
                    output = PID_OUT_MIN;

                ch->current_output = output;
            }
        }

        /* Time-proportioning Software PWM strictly on Relays */
        uint32_t period_ms = (ch->cycle_time_s > 0) ? ((uint32_t)ch->cycle_time_s * 1000u) : SOFT_PWM_PERIOD_MS;
        uint32_t phase = now % period_ms;

        bool heater_on = false;
        bool blower_on = false;

        if (ch->current_output > 0)
        {
            uint32_t threshold = ((uint32_t)ch->current_output * period_ms) / PID_OUT_MAX;
            if (phase < threshold)
            {
                heater_on = true;
            }
        }
        else if (ch->current_output < 0)
        {
            uint32_t threshold = ((uint32_t)(-ch->current_output) * period_ms) / PID_OUT_MAX;
            if (phase < threshold)
            {
                blower_on = true;
            }
        }

        if (ch->heater_relay < NUM_RELAYS)
        {
            relay_set(ch->heater_relay, heater_on);
        }
        if (ch->blower_relay < NUM_RELAYS)
        {
            relay_set(ch->blower_relay, blower_on);
        }
    }
}

uint16_t pid_get_mode(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].mode : 0; }
uint16_t pid_get_setpoint(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? (uint16_t)pid_channels[ch].setpoint : 0; }
uint16_t pid_get_kp(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].kp : 0; }
uint16_t pid_get_ki(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].ki : 0; }
uint16_t pid_get_kd(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].kd : 0; }
uint16_t pid_get_deadzone(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].deadzone : 0; }
uint16_t pid_get_heater_relay(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].heater_relay : 0xFFFF; }
uint16_t pid_get_blower_relay(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].blower_relay : 0xFFFF; }
uint16_t pid_get_manual_duty(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? (uint16_t)pid_channels[ch].manual_duty : 0; }

void pid_set_mode(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].mode = val;
        flash_eeprom_get_mutable()->pid[ch].mode = val;
        flash_eeprom_mark_dirty();
    }
}
void pid_set_setpoint(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        int16_t sp = (int16_t)val;
        if (sp > pid_channels[ch].sv_high_limit)
            sp = pid_channels[ch].sv_high_limit;
        if (sp < pid_channels[ch].sv_low_limit)
            sp = pid_channels[ch].sv_low_limit;
        pid_channels[ch].setpoint = sp;
        flash_eeprom_get_mutable()->pid[ch].setpoint = sp;
        flash_eeprom_mark_dirty();
    }
}
void pid_set_kp(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].kp = val;
        flash_eeprom_get_mutable()->pid[ch].kp = val;
        flash_eeprom_mark_dirty();
    }
}
void pid_set_ki(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].ki = val;
        flash_eeprom_get_mutable()->pid[ch].ki = val;
        flash_eeprom_mark_dirty();
    }
}
void pid_set_kd(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].kd = val;
        flash_eeprom_get_mutable()->pid[ch].kd = val;
        flash_eeprom_mark_dirty();
    }
}
void pid_set_deadzone(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].deadzone = val;
        flash_eeprom_get_mutable()->pid[ch].deadzone = val;
        flash_eeprom_mark_dirty();
    }
}
bool pid_is_relay_assigned(uint8_t exclude_ch, bool exclude_is_blower, uint16_t relay_id)
{
    if (relay_id >= NUM_RELAYS)
    {
        return false;
    }
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        if (!(exclude_is_blower == false && i == exclude_ch))
        {
            if (pid_channels[i].heater_relay == relay_id)
            {
                return true;
            }
        }
        if (!(exclude_is_blower == true && i == exclude_ch))
        {
            if (pid_channels[i].blower_relay == relay_id)
            {
                return true;
            }
        }
    }
    return false;
}

bool pid_set_heater_relay(uint8_t ch, uint16_t val)
{
    if (ch >= NUM_THERMOCOUPLES)
    {
        return false;
    }
    if (val == PID_ACTUATOR_NONE)
    {
        pid_channels[ch].heater_relay = val;
        flash_eeprom_get_mutable()->pid[ch].heater_relay = val;
        flash_eeprom_mark_dirty();
        return true;
    }
    if (val >= NUM_RELAYS)
    {
        return false; /* Disallow invalid relays and PWM actuators */
    }
    if (pid_is_relay_assigned(ch, false, val))
    {
        return false; /* Mutual exclusivity: relay already in use */
    }
    pid_channels[ch].heater_relay = val;
    flash_eeprom_get_mutable()->pid[ch].heater_relay = val;
    flash_eeprom_mark_dirty();
    return true;
}

bool pid_set_blower_relay(uint8_t ch, uint16_t val)
{
    if (ch >= NUM_THERMOCOUPLES)
    {
        return false;
    }
    if (val == PID_ACTUATOR_NONE)
    {
        pid_channels[ch].blower_relay = val;
        flash_eeprom_get_mutable()->pid[ch].blower_relay = val;
        flash_eeprom_mark_dirty();
        return true;
    }
    if (val >= NUM_RELAYS)
    {
        return false; /* Disallow invalid relays */
    }
    if (pid_is_relay_assigned(ch, true, val))
    {
        return false; /* Mutual exclusivity: relay already in use */
    }
    pid_channels[ch].blower_relay = val;
    flash_eeprom_get_mutable()->pid[ch].blower_relay = val;
    flash_eeprom_mark_dirty();
    return true;
}
void pid_set_manual_duty(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].manual_duty = (int16_t)val;
        flash_eeprom_get_mutable()->pid[ch].manual_duty = (int16_t)val;
        flash_eeprom_mark_dirty();
    }
}

uint16_t pid_get_action(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].action : 0; }
uint16_t pid_get_hysteresis(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].hysteresis : 0; }
uint16_t pid_get_cycle_time(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].cycle_time_s : 0; }
int16_t pid_get_sv_high_limit(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].sv_high_limit : 13700; }
int16_t pid_get_sv_low_limit(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].sv_low_limit : -2000; }
int16_t pid_get_pv_offset(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].pv_offset : 0; }
uint16_t pid_get_fail_safe_duty(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].fail_safe_duty : 0; }
int16_t pid_get_alarm_high(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].alarm_high : 13700; }
int16_t pid_get_alarm_low(uint8_t ch) { return (ch < NUM_THERMOCOUPLES) ? pid_channels[ch].alarm_low : -2000; }

void pid_set_action(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].action = val;
        flash_eeprom_get_mutable()->pid[ch].action = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_hysteresis(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].hysteresis = val;
        flash_eeprom_get_mutable()->pid[ch].hysteresis = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_cycle_time(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        if (val < 1)
            val = 1;
        if (val > 100)
            val = 100;
        pid_channels[ch].cycle_time_s = val;
        flash_eeprom_get_mutable()->pid[ch].cycle_time_s = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_sv_high_limit(uint8_t ch, int16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].sv_high_limit = val;
        flash_eeprom_get_mutable()->pid[ch].sv_high_limit = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_sv_low_limit(uint8_t ch, int16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].sv_low_limit = val;
        flash_eeprom_get_mutable()->pid[ch].sv_low_limit = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_pv_offset(uint8_t ch, int16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].pv_offset = val;
        flash_eeprom_get_mutable()->pid[ch].pv_offset = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_fail_safe_duty(uint8_t ch, uint16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        if (val > 1000)
            val = 1000;
        pid_channels[ch].fail_safe_duty = val;
        flash_eeprom_get_mutable()->pid[ch].fail_safe_duty = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_alarm_high(uint8_t ch, int16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].alarm_high = val;
        flash_eeprom_get_mutable()->pid[ch].alarm_high = val;
        flash_eeprom_mark_dirty();
    }
}

void pid_set_alarm_low(uint8_t ch, int16_t val)
{
    if (ch < NUM_THERMOCOUPLES)
    {
        pid_channels[ch].alarm_low = val;
        flash_eeprom_get_mutable()->pid[ch].alarm_low = val;
        flash_eeprom_mark_dirty();
    }
}
