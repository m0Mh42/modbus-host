/**
 * @file    pid_control.h
 * @brief   PID temperature control and software PWM relay driver.
 */
#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#define PID_ACTUATOR_NONE 0xFFFF

#define PID_MODE_MANUAL 0
#define PID_MODE_AUTO 1
#define PID_MODE_ON_OFF 2

#define PID_ACTION_HEATING 0
#define PID_ACTION_COOLING 1

/** Initialize the PID controllers to default safe values. */
void pid_control_init(void);

/**
 * Process the control loop. Should be called periodically (e.g. every 100ms).
 * Calculates PID output and applies software PWM to relays or direct duty to PWM.
 */
void pid_control_process(void);

/* Modbus Getters */
uint16_t pid_get_mode(uint8_t ch);
uint16_t pid_get_setpoint(uint8_t ch);
uint16_t pid_get_kp(uint8_t ch);
uint16_t pid_get_ki(uint8_t ch);
uint16_t pid_get_kd(uint8_t ch);
uint16_t pid_get_deadzone(uint8_t ch);
uint16_t pid_get_heater_relay(uint8_t ch);
uint16_t pid_get_blower_relay(uint8_t ch);
uint16_t pid_get_manual_duty(uint8_t ch);

/* Extended Thermocontroller Getters */
uint16_t pid_get_action(uint8_t ch);
uint16_t pid_get_hysteresis(uint8_t ch);
uint16_t pid_get_cycle_time(uint8_t ch);
int16_t pid_get_sv_high_limit(uint8_t ch);
int16_t pid_get_sv_low_limit(uint8_t ch);
int16_t pid_get_pv_offset(uint8_t ch);
uint16_t pid_get_fail_safe_duty(uint8_t ch);
int16_t pid_get_alarm_high(uint8_t ch);
int16_t pid_get_alarm_low(uint8_t ch);

/* Modbus Setters */
void pid_set_mode(uint8_t ch, uint16_t val);
void pid_set_setpoint(uint8_t ch, uint16_t val);
void pid_set_kp(uint8_t ch, uint16_t val);
void pid_set_ki(uint8_t ch, uint16_t val);
void pid_set_kd(uint8_t ch, uint16_t val);
void pid_set_deadzone(uint8_t ch, uint16_t val);
bool pid_set_heater_relay(uint8_t ch, uint16_t val);
bool pid_set_blower_relay(uint8_t ch, uint16_t val);
void pid_set_manual_duty(uint8_t ch, uint16_t val);

/** Check whether a physical relay (0..15) is already assigned to any zone. */
bool pid_is_relay_assigned(uint8_t exclude_ch, bool exclude_is_blower, uint16_t relay_id);

/* Extended Thermocontroller Setters */
void pid_set_action(uint8_t ch, uint16_t val);
void pid_set_hysteresis(uint8_t ch, uint16_t val);
void pid_set_cycle_time(uint8_t ch, uint16_t val);
void pid_set_sv_high_limit(uint8_t ch, int16_t val);
void pid_set_sv_low_limit(uint8_t ch, int16_t val);
void pid_set_pv_offset(uint8_t ch, int16_t val);
void pid_set_fail_safe_duty(uint8_t ch, uint16_t val);
void pid_set_alarm_high(uint8_t ch, int16_t val);
void pid_set_alarm_low(uint8_t ch, int16_t val);

#endif /* PID_CONTROL_H */
