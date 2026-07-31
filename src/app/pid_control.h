/**
 * @file    pid_control.h
 * @brief   PID temperature control and software PWM relay driver.
 */
#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/** Initialize the PID controllers to default safe values. */
void pid_control_init(void);

/** 
 * Process the control loop. Should be called periodically (e.g. every 100ms).
 * Calculates PID output and applies software PWM to the assigned relays.
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

/* Modbus Setters */
void pid_set_mode(uint8_t ch, uint16_t val);
void pid_set_setpoint(uint8_t ch, uint16_t val);
void pid_set_kp(uint8_t ch, uint16_t val);
void pid_set_ki(uint8_t ch, uint16_t val);
void pid_set_kd(uint8_t ch, uint16_t val);
void pid_set_deadzone(uint8_t ch, uint16_t val);
void pid_set_heater_relay(uint8_t ch, uint16_t val);
void pid_set_blower_relay(uint8_t ch, uint16_t val);
void pid_set_manual_duty(uint8_t ch, uint16_t val);

#endif /* PID_CONTROL_H */
