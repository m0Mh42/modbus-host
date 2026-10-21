/**
 * @file    pwm_pid.h
 * @brief   Closed-loop PID controller for PWM outputs (speed / analog).
 */
#ifndef PWM_PID_H
#define PWM_PID_H

#include <stdint.h>
#include <stdbool.h>

#define PWM_PID_SRC_LINE_SPEED 0
#define PWM_PID_SRC_AIN1       1
#define PWM_PID_SRC_AIN2       2
#define PWM_PID_SRC_PROX1_HZ   3
#define PWM_PID_SRC_PROX2_HZ   4
#define PWM_PID_SRC_TC1        5
#define PWM_PID_SRC_TC2        6
#define PWM_PID_SRC_TC3        7
#define PWM_PID_SRC_TC4        8
#define PWM_PID_SRC_TC5        9
#define PWM_PID_SRC_TC6        10
#define PWM_PID_SRC_TC7        11
#define PWM_PID_SRC_TC8        12

#define NUM_PWM_PID_CHANNELS   2

void pwm_pid_init(void);
void pwm_pid_process(void);

/* Live Scaled Process Variable */
int32_t pwm_pid_get_pv(uint8_t ch);

/* Modbus Getters */
uint16_t pwm_pid_get_mode(uint8_t ch);
uint16_t pwm_pid_get_src(uint8_t ch);
uint16_t pwm_pid_get_setpoint(uint8_t ch);
uint16_t pwm_pid_get_kp(uint8_t ch);
uint16_t pwm_pid_get_ki(uint8_t ch);
uint16_t pwm_pid_get_kd(uint8_t ch);
uint16_t pwm_pid_get_min_duty(uint8_t ch);
uint16_t pwm_pid_get_max_duty(uint8_t ch);
uint16_t pwm_pid_get_scale_mul(uint8_t ch);
uint16_t pwm_pid_get_scale_div(uint8_t ch);

/* Modbus Setters */
void pwm_pid_set_mode(uint8_t ch, uint16_t val);
void pwm_pid_set_src(uint8_t ch, uint16_t val);
void pwm_pid_set_setpoint(uint8_t ch, uint16_t val);
void pwm_pid_set_kp(uint8_t ch, uint16_t val);
void pwm_pid_set_ki(uint8_t ch, uint16_t val);
void pwm_pid_set_kd(uint8_t ch, uint16_t val);
void pwm_pid_set_min_duty(uint8_t ch, uint16_t val);
void pwm_pid_set_max_duty(uint8_t ch, uint16_t val);
void pwm_pid_set_scale_mul(uint8_t ch, uint16_t val);
void pwm_pid_set_scale_div(uint8_t ch, uint16_t val);

#endif /* PWM_PID_H */
