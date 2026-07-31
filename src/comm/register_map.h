/**
 * @file    register_map.h
 * @brief   Modbus data model: bridges the protocol layer to the application's
 *          sensors and actuators. The Modbus stack only ever calls these
 *          accessors - it never touches the drivers directly.
 */
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#include <stdint.h>
#include <stdbool.h>

/* ============================ ADDRESS PLAN ============================ */
/* INPUT REGISTERS (FC04, read-only): */
#define REG_IN_TC_TEMP_BASE     0x0000 /* 8 ch, 0.1 C, signed */
#define REG_IN_TC_VALID_BASE    0x0008 /* 8 ch, 0/1 */
#define REG_IN_CURRENT_BASE     0x0010 /* 8 ch, mA */
#define REG_IN_AIN_BASE         0x0018 /* 2 ch, mV, divider-corrected */
#define REG_IN_PROX1_COUNT_HI   0x0020 /* 32-bit pulses */
#define REG_IN_PROX1_COUNT_LO   0x0021
#define REG_IN_PROX2_COUNT_HI   0x0022
#define REG_IN_PROX2_COUNT_LO   0x0023
#define REG_IN_PROX1_SPEED      0x0024 /* pulses/sec */
#define REG_IN_PROX2_SPEED      0x0025

/* HOLDING REGISTERS (FC03 read / FC06,FC16 write): */
#define REG_HOLD_RELAY_MASK     0x0100 /* bit n -> relay n */
#define REG_HOLD_PWM1_DUTY      0x0101 /* 0..PWM_MAX_DUTY */
#define REG_HOLD_PWM2_DUTY      0x0102

/* PID & Configuration Holding Registers (8 channels each) */
#define REG_HOLD_PID_MODE_BASE       0x0110 /* 0=Manual, 1=Auto */
#define REG_HOLD_PID_SETPOINT_BASE   0x0120 /* 0.1 C */
#define REG_HOLD_PID_KP_BASE         0x0130 /* Kp * 100 */
#define REG_HOLD_PID_KI_BASE         0x0140 /* Ki * 100 */
#define REG_HOLD_PID_KD_BASE         0x0150 /* Kd * 100 */
#define REG_HOLD_PID_DEADZONE_BASE   0x0160 /* 0.1 C */
#define REG_HOLD_TC_OFFSET_BASE      0x0170 /* DC offset uV (signed 16-bit) */
#define REG_HOLD_HEATER_RELAY_BASE   0x0180 /* 0..15 or 0xFFFF for none */
#define REG_HOLD_BLOWER_RELAY_BASE   0x0190 /* 0..15 or 0xFFFF for none */
#define REG_HOLD_MANUAL_DUTY_BASE    0x01A0 /* -1000..1000 (signed) */

/* COILS (FC01 read / FC05,FC15 write): */
#define COIL_RELAY_BASE         0x0000 /* relay 0..15 */
/* ===================================================================== */

/** Refresh the cached input-register snapshot from the services. */
void register_map_update_inputs(void);

/** Read one input register (FC04). Returns false if address is unmapped. */
bool register_map_read_input(uint16_t addr, uint16_t *out);

/** Read one holding register (FC03). Returns false if unmapped. */
bool register_map_read_holding(uint16_t addr, uint16_t *out);

/** Write one holding register (FC06/FC16). Returns false if read-only/unmapped. */
bool register_map_write_holding(uint16_t addr, uint16_t value);

/** Coil access (relays). */
bool register_map_read_coil(uint16_t addr, bool *out);
bool register_map_write_coil(uint16_t addr, bool value);

#endif /* REGISTER_MAP_H */
