/**
 * @file    pid_control.c
 * @brief   PID temperature control and software PWM relay driver.
 */
#include "pid_control.h"
#include "app_config.h"
#include "thermocouple.h"
#include "relay.h"
#include <string.h>

#define SOFT_PWM_PERIOD_MS   2000   /* 2 seconds software PWM period */
#define PID_OUT_MAX          1000   /* 100% duty cycle */
#define PID_OUT_MIN         -1000   /* -100% duty cycle (blower) */

typedef struct {
    uint16_t mode;          /* 0 = Manual, 1 = Auto */
    int16_t  setpoint;      /* 0.1 C */
    uint16_t kp;            /* Kp * 100 */
    uint16_t ki;            /* Ki * 100 */
    uint16_t kd;            /* Kd * 100 */
    uint16_t deadzone;      /* 0.1 C */
    uint16_t heater_relay;  /* 0..15, or 0xFFFF for none */
    uint16_t blower_relay;  /* 0..15, or 0xFFFF for none */
    int16_t  manual_duty;   /* -1000..1000 */
    
    /* Internal State */
    int32_t  integral;
    int16_t  last_error;
    int32_t  current_output; /* -1000..1000 */
} pid_channel_t;

static pid_channel_t pid_channels[NUM_THERMOCOUPLES];
static uint32_t soft_pwm_counter_ms = 0;

void pid_control_init(void)
{
    memset(pid_channels, 0, sizeof(pid_channels));
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
        pid_channels[i].mode = 0;          /* Manual by default for safety */
        pid_channels[i].setpoint = 250;    /* 25.0 C */
        pid_channels[i].kp = 1000;         /* Default 10.0 */
        pid_channels[i].ki = 50;           /* Default 0.5 */
        pid_channels[i].kd = 100;          /* Default 1.0 */
        pid_channels[i].deadzone = 20;     /* 2.0 C deadzone */
        pid_channels[i].heater_relay = i;  /* Map TC[i] -> Relay[i] */
        pid_channels[i].blower_relay = i + 8; /* Map TC[i] -> Relay[i+8] */
        pid_channels[i].manual_duty = 0;
        pid_channels[i].integral = 0;
        pid_channels[i].last_error = 0;
        pid_channels[i].current_output = 0;
    }
}

void pid_control_process(void)
{
    soft_pwm_counter_ms += CONTROL_LOOP_PERIOD_MS;
    if (soft_pwm_counter_ms >= SOFT_PWM_PERIOD_MS) {
        soft_pwm_counter_ms = 0;
    }

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
        pid_channel_t *ch = &pid_channels[i];
        
        if (ch->mode == 0) {
            /* Manual Mode */
            ch->current_output = ch->manual_duty;
            ch->integral = 0; /* Reset integral to avoid windup on switch */
        } else {
            /* Auto Mode (PID) */
            if (!thermocouple_is_valid(i)) {
                /* Sensor fault - fail safe to 0 output */
                ch->current_output = 0;
                ch->integral = 0;
            } else {
                int16_t current_temp = thermocouple_get_temp(i);
                int16_t error = ch->setpoint - current_temp;

                /* Apply Deadzone */
                if (error > 0 && error < ch->deadzone) {
                    error = 0;
                } else if (error < 0 && error > -(int16_t)ch->deadzone) {
                    error = 0;
                }
                
                int32_t p_term = ((int32_t)error * ch->kp) / 100;
                
                ch->integral += ((int32_t)error * CONTROL_LOOP_PERIOD_MS) / 1000;
                
                /* Anti-windup */
                int32_t max_integral = (PID_OUT_MAX * 100) / (ch->ki > 0 ? ch->ki : 1);
                if (ch->integral > max_integral) ch->integral = max_integral;
                if (ch->integral < -max_integral) ch->integral = -max_integral;

                int32_t i_term = (ch->integral * ch->ki) / 100;
                
                int32_t derivative = (error - ch->last_error) * 1000 / CONTROL_LOOP_PERIOD_MS;
                int32_t d_term = (derivative * ch->kd) / 100;
                
                ch->last_error = error;
                
                int32_t output = p_term + i_term + d_term;
                
                if (output > PID_OUT_MAX) output = PID_OUT_MAX;
                if (output < PID_OUT_MIN) output = PID_OUT_MIN;
                
                ch->current_output = output;
            }
        }
        
        /* Apply Output to Relays via Software PWM */
        bool heater_on = false;
        bool blower_on = false;
        
        if (ch->current_output > 0) {
            /* Heating */
            uint32_t threshold = (ch->current_output * SOFT_PWM_PERIOD_MS) / PID_OUT_MAX;
            if (soft_pwm_counter_ms < threshold) {
                heater_on = true;
            }
        } else if (ch->current_output < 0) {
            /* Blowing */
            uint32_t threshold = ((-ch->current_output) * SOFT_PWM_PERIOD_MS) / PID_OUT_MAX;
            if (soft_pwm_counter_ms < threshold) {
                blower_on = true;
            }
        }
        
        if (ch->heater_relay < NUM_RELAYS) {
            relay_set(ch->heater_relay, heater_on);
        }
        if (ch->blower_relay < NUM_RELAYS) {
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

void pid_set_mode(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].mode = val; }
void pid_set_setpoint(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].setpoint = (int16_t)val; }
void pid_set_kp(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].kp = val; }
void pid_set_ki(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].ki = val; }
void pid_set_kd(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].kd = val; }
void pid_set_deadzone(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].deadzone = val; }
void pid_set_heater_relay(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].heater_relay = val; }
void pid_set_blower_relay(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].blower_relay = val; }
void pid_set_manual_duty(uint8_t ch, uint16_t val) { if (ch < NUM_THERMOCOUPLES) pid_channels[ch].manual_duty = (int16_t)val; }
