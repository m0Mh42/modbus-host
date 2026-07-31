/**
 * @file register_map.c
 * @brief Modbus data model implementation.
 */
#include "register_map.h"
#include "app_config.h"

#include "thermocouple.h"
#include "current_monitor.h"
#include "analog_input.h"
#include "proximity.h"
#include "relay.h"
#include "pwm_output.h"
#include "pid_control.h"

/* Snapshot of input registers, refreshed each control loop. */
static uint16_t input_snapshot[0x0026];

void register_map_update_inputs(void)
{
    /* Thermocouples: temperature + valid flag. */
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
        input_snapshot[REG_IN_TC_TEMP_BASE + i]  = (uint16_t)thermocouple_get_temp(i);
        input_snapshot[REG_IN_TC_VALID_BASE + i] = thermocouple_is_valid(i) ? 1 : 0;
    }

    /* Heater currents. */
    for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++) {
        input_snapshot[REG_IN_CURRENT_BASE + i] = current_monitor_get(i);
    }

    /* General analog inputs (mV). */
    for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++) {
        input_snapshot[REG_IN_AIN_BASE + i] = analog_input_get_mv(i);
    }

    /* Proximity counts (32-bit, split hi/lo) and speeds. */
    uint32_t c1 = proximity_get_count(PROX_CHANNEL_1);
    uint32_t c2 = proximity_get_count(PROX_CHANNEL_2);
    input_snapshot[REG_IN_PROX1_COUNT_HI] = (uint16_t)(c1 >> 16);
    input_snapshot[REG_IN_PROX1_COUNT_LO] = (uint16_t)(c1 & 0xFFFF);
    input_snapshot[REG_IN_PROX2_COUNT_HI] = (uint16_t)(c2 >> 16);
    input_snapshot[REG_IN_PROX2_COUNT_LO] = (uint16_t)(c2 & 0xFFFF);
    input_snapshot[REG_IN_PROX1_SPEED] = (uint16_t)proximity_get_speed(PROX_CHANNEL_1);
    input_snapshot[REG_IN_PROX2_SPEED] = (uint16_t)proximity_get_speed(PROX_CHANNEL_2);
}

bool register_map_read_input(uint16_t addr, uint16_t *out)
{
    if (addr >= (sizeof(input_snapshot) / sizeof(input_snapshot[0]))) {
        return false;
    }
    *out = input_snapshot[addr];
    return true;
}

bool register_map_read_holding(uint16_t addr, uint16_t *out)
{
    if (addr == REG_HOLD_RELAY_MASK) { *out = relay_get_mask(); return true; }
    if (addr == REG_HOLD_PWM1_DUTY)  { *out = pwm_output_get(0); return true; }
    if (addr == REG_HOLD_PWM2_DUTY)  { *out = pwm_output_get(1); return true; }

    if (addr >= REG_HOLD_PID_MODE_BASE && addr < REG_HOLD_PID_MODE_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_mode(addr - REG_HOLD_PID_MODE_BASE); return true;
    }
    if (addr >= REG_HOLD_PID_SETPOINT_BASE && addr < REG_HOLD_PID_SETPOINT_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_setpoint(addr - REG_HOLD_PID_SETPOINT_BASE); return true;
    }
    if (addr >= REG_HOLD_PID_KP_BASE && addr < REG_HOLD_PID_KP_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_kp(addr - REG_HOLD_PID_KP_BASE); return true;
    }
    if (addr >= REG_HOLD_PID_KI_BASE && addr < REG_HOLD_PID_KI_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_ki(addr - REG_HOLD_PID_KI_BASE); return true;
    }
    if (addr >= REG_HOLD_PID_KD_BASE && addr < REG_HOLD_PID_KD_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_kd(addr - REG_HOLD_PID_KD_BASE); return true;
    }
    if (addr >= REG_HOLD_PID_DEADZONE_BASE && addr < REG_HOLD_PID_DEADZONE_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_deadzone(addr - REG_HOLD_PID_DEADZONE_BASE); return true;
    }
    if (addr >= REG_HOLD_TC_OFFSET_BASE && addr < REG_HOLD_TC_OFFSET_BASE + NUM_THERMOCOUPLES) {
        *out = (uint16_t)thermocouple_get_offset(addr - REG_HOLD_TC_OFFSET_BASE); return true;
    }
    if (addr >= REG_HOLD_HEATER_RELAY_BASE && addr < REG_HOLD_HEATER_RELAY_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_heater_relay(addr - REG_HOLD_HEATER_RELAY_BASE); return true;
    }
    if (addr >= REG_HOLD_BLOWER_RELAY_BASE && addr < REG_HOLD_BLOWER_RELAY_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_blower_relay(addr - REG_HOLD_BLOWER_RELAY_BASE); return true;
    }
    if (addr >= REG_HOLD_MANUAL_DUTY_BASE && addr < REG_HOLD_MANUAL_DUTY_BASE + NUM_THERMOCOUPLES) {
        *out = pid_get_manual_duty(addr - REG_HOLD_MANUAL_DUTY_BASE); return true;
    }
    
    return false;
}

bool register_map_write_holding(uint16_t addr, uint16_t value)
{
    if (addr == REG_HOLD_RELAY_MASK) { relay_set_mask(value); return true; }
    if (addr == REG_HOLD_PWM1_DUTY)  { pwm_output_set(0, value); return true; }
    if (addr == REG_HOLD_PWM2_DUTY)  { pwm_output_set(1, value); return true; }

    if (addr >= REG_HOLD_PID_MODE_BASE && addr < REG_HOLD_PID_MODE_BASE + NUM_THERMOCOUPLES) {
        pid_set_mode(addr - REG_HOLD_PID_MODE_BASE, value); return true;
    }
    if (addr >= REG_HOLD_PID_SETPOINT_BASE && addr < REG_HOLD_PID_SETPOINT_BASE + NUM_THERMOCOUPLES) {
        pid_set_setpoint(addr - REG_HOLD_PID_SETPOINT_BASE, value); return true;
    }
    if (addr >= REG_HOLD_PID_KP_BASE && addr < REG_HOLD_PID_KP_BASE + NUM_THERMOCOUPLES) {
        pid_set_kp(addr - REG_HOLD_PID_KP_BASE, value); return true;
    }
    if (addr >= REG_HOLD_PID_KI_BASE && addr < REG_HOLD_PID_KI_BASE + NUM_THERMOCOUPLES) {
        pid_set_ki(addr - REG_HOLD_PID_KI_BASE, value); return true;
    }
    if (addr >= REG_HOLD_PID_KD_BASE && addr < REG_HOLD_PID_KD_BASE + NUM_THERMOCOUPLES) {
        pid_set_kd(addr - REG_HOLD_PID_KD_BASE, value); return true;
    }
    if (addr >= REG_HOLD_PID_DEADZONE_BASE && addr < REG_HOLD_PID_DEADZONE_BASE + NUM_THERMOCOUPLES) {
        pid_set_deadzone(addr - REG_HOLD_PID_DEADZONE_BASE, value); return true;
    }
    if (addr >= REG_HOLD_TC_OFFSET_BASE && addr < REG_HOLD_TC_OFFSET_BASE + NUM_THERMOCOUPLES) {
        thermocouple_set_offset(addr - REG_HOLD_TC_OFFSET_BASE, (int16_t)value); return true;
    }
    if (addr >= REG_HOLD_HEATER_RELAY_BASE && addr < REG_HOLD_HEATER_RELAY_BASE + NUM_THERMOCOUPLES) {
        pid_set_heater_relay(addr - REG_HOLD_HEATER_RELAY_BASE, value); return true;
    }
    if (addr >= REG_HOLD_BLOWER_RELAY_BASE && addr < REG_HOLD_BLOWER_RELAY_BASE + NUM_THERMOCOUPLES) {
        pid_set_blower_relay(addr - REG_HOLD_BLOWER_RELAY_BASE, value); return true;
    }
    if (addr >= REG_HOLD_MANUAL_DUTY_BASE && addr < REG_HOLD_MANUAL_DUTY_BASE + NUM_THERMOCOUPLES) {
        pid_set_manual_duty(addr - REG_HOLD_MANUAL_DUTY_BASE, value); return true;
    }

    return false;
}

bool register_map_read_coil(uint16_t addr, bool *out)
{
    if (addr >= COIL_RELAY_BASE && addr < COIL_RELAY_BASE + NUM_RELAYS) {
        *out = relay_get((uint8_t)(addr - COIL_RELAY_BASE));
        return true;
    }
    return false;
}

bool register_map_write_coil(uint16_t addr, bool value)
{
    if (addr >= COIL_RELAY_BASE && addr < COIL_RELAY_BASE + NUM_RELAYS) {
        relay_set((uint8_t)(addr - COIL_RELAY_BASE), value);
        return true;
    }
    return false;
}
