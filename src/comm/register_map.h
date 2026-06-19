/**
 * @file    register_map.h
 * @brief   Modbus data model: bridges the protocol layer to the application's
 *          sensors and actuators. The Modbus stack only ever calls these
 *          accessors - it never touches the drivers directly.
 *
 * ============================ ADDRESS PLAN ============================
 * INPUT REGISTERS (FC04, read-only):
 *   0x0000..0x0007  thermocouple temperature ch0..7   (0.1 C, signed)
 *   0x0008..0x000F  thermocouple valid flag   ch0..7   (0/1)
 *   0x0010..0x0017  heater current            ch0..7   (mA)
 *   0x0018..0x0019  analog input AIN1, AIN2            (mV, divider-corrected)
 *   0x0020..0x0021  proximity 1 count (hi, lo)         (32-bit pulses)
 *   0x0022..0x0023  proximity 2 count (hi, lo)
 *   0x0024          proximity 1 speed                  (pulses/sec)
 *   0x0025          proximity 2 speed
 *
 * HOLDING REGISTERS (FC03 read / FC06,FC16 write):
 *   0x0100          relay output mask (bit n -> relay n)
 *   0x0101          PWM1 duty (0..PWM_MAX_DUTY)
 *   0x0102          PWM2 duty (0..PWM_MAX_DUTY)
 *
 * COILS (FC01 read / FC05,FC15 write):
 *   0x0000..0x000F  relay 0..15
 * =====================================================================
 */
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#include <stdint.h>
#include <stdbool.h>

/* Input-register addresses */
#define REG_IN_TC_TEMP_BASE     0x0000
#define REG_IN_TC_VALID_BASE    0x0008
#define REG_IN_CURRENT_BASE     0x0010
#define REG_IN_AIN_BASE         0x0018
#define REG_IN_PROX1_COUNT_HI   0x0020
#define REG_IN_PROX1_COUNT_LO   0x0021
#define REG_IN_PROX2_COUNT_HI   0x0022
#define REG_IN_PROX2_COUNT_LO   0x0023
#define REG_IN_PROX1_SPEED      0x0024
#define REG_IN_PROX2_SPEED      0x0025

/* Holding-register addresses */
#define REG_HOLD_RELAY_MASK     0x0100
#define REG_HOLD_PWM1_DUTY      0x0101
#define REG_HOLD_PWM2_DUTY      0x0102

/* Coil base for relays */
#define COIL_RELAY_BASE         0x0000

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
