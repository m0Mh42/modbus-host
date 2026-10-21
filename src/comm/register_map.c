/**
 * @file register_map.c
 * @brief Modbus data model implementation.
 */
#include "register_map.h"
#include "app_config.h"
#include "board_config.h"
#include "hal_gpio.h"
#include "thermocouple.h"
#include "current_monitor.h"
#include "analog_input.h"
#include "proximity.h"
#include "relay.h"
#include "pwm_output.h"
#include "pwm_pid.h"
#include "pid_control.h"
#include "flash_eeprom.h"
#include "hal_clock.h"
#include <string.h>

static uint16_t pack_string_reg(const char *str, uint8_t reg_idx)
{
    if (!str)
        return 0;
    size_t len = strlen(str);
    size_t idx0 = (size_t)reg_idx * 2;
    size_t idx1 = (size_t)reg_idx * 2 + 1;
    uint8_t b0 = (idx0 < len) ? (uint8_t)str[idx0] : 0;
    uint8_t b1 = (idx1 < len) ? (uint8_t)str[idx1] : 0;
    return ((uint16_t)b0 << 8) | (uint16_t)b1;
}

static void unpack_string_reg(char *buf, size_t max_buf_len, uint8_t reg_idx, uint16_t value)
{
    if (!buf || max_buf_len < 2)
        return;
    if (reg_idx == 0)
    {
        memset(buf, 0, max_buf_len);
    }
    size_t idx0 = reg_idx * 2;
    size_t idx1 = reg_idx * 2 + 1;
    if (idx0 < max_buf_len - 1)
    {
        buf[idx0] = (char)((value >> 8) & 0xFF);
    }
    if (idx1 < max_buf_len - 1)
    {
        buf[idx1] = (char)(value & 0xFF);
    }
    buf[max_buf_len - 1] = '\0';
    flash_eeprom_mark_dirty();
}

/* Snapshot of input registers, refreshed each control loop. */
static uint16_t input_snapshot[0x0040];
static uint16_t machine_run_state = 0; /* 0=Stop, 1=Run */
static bool fault_acknowledged = false;

void register_map_update_inputs(void)
{
    /* Thermocouples: temperature + valid flag. */
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        input_snapshot[REG_IN_TC_TEMP_BASE + i] = (uint16_t)thermocouple_get_temp(i);
        input_snapshot[REG_IN_TC_VALID_BASE + i] = thermocouple_is_valid(i) ? 1 : 0;
    }

    /* Heater currents. */
    for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++)
    {
        input_snapshot[REG_IN_CURRENT_BASE + i] = current_monitor_get(i);
    }

    /* General analog inputs (mV). */
    for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++)
    {
        input_snapshot[REG_IN_AIN_BASE + i] = analog_input_get_mv(i);
    }

    /* Cold Junction Compensation temperature (0.1 C) */
    input_snapshot[REG_IN_CJC_TEMP] = (uint16_t)thermocouple_get_cjc_temp_c10();

    /* Proximity counts (32-bit, split hi/lo) and speeds. */
    uint32_t c1 = proximity_get_count(PROX_CHANNEL_1);
    uint32_t c2 = proximity_get_count(PROX_CHANNEL_2);
    input_snapshot[REG_IN_PROX1_COUNT_HI] = (uint16_t)(c1 >> 16);
    input_snapshot[REG_IN_PROX1_COUNT_LO] = (uint16_t)(c1 & 0xFFFF);
    input_snapshot[REG_IN_PROX2_COUNT_HI] = (uint16_t)(c2 >> 16);
    input_snapshot[REG_IN_PROX2_COUNT_LO] = (uint16_t)(c2 & 0xFFFF);
    input_snapshot[REG_IN_PROX1_SPEED] = (uint16_t)proximity_get_speed(PROX_CHANNEL_1);
    input_snapshot[REG_IN_PROX2_SPEED] = (uint16_t)proximity_get_speed(PROX_CHANNEL_2);
    input_snapshot[REG_IN_LINE_SPEED] = proximity_get_line_speed_c10();
    input_snapshot[REG_IN_SPEED_HZ] = (uint16_t)proximity_get_effective_speed_hz();

    /* Fault registers */
    uint16_t f_tc = thermocouple_get_faults();
    uint16_t f_ct = current_monitor_get_faults();
    uint16_t f_ain = analog_input_get_faults();
    uint16_t f_prox = proximity_get_faults();
    uint16_t f_sys = flash_eeprom_get_faults();
    if (hal_clock_has_crystal_fault())
    {
        f_sys |= FAULT_SYS_CRYSTAL_ERROR;
    }

    uint16_t f_overall = 0;
    if (f_tc)
        f_overall |= FAULT_OVERALL_TC;
    if (f_ct)
        f_overall |= FAULT_OVERALL_CT;
    if (f_ain)
        f_overall |= FAULT_OVERALL_AIN;
    if (f_prox)
        f_overall |= FAULT_OVERALL_PROX;
    if (f_sys)
        f_overall |= FAULT_OVERALL_SYS;
    if (f_overall != 0)
    {
        f_overall |= FAULT_OVERALL_MASTER; /* Master fault flag */
        if (fault_acknowledged)
        {
            f_overall |= FAULT_OVERALL_ACK;
        }
    }
    else
    {
        fault_acknowledged = false;
    }

    input_snapshot[REG_IN_FAULT_OVERALL] = f_overall;
    input_snapshot[REG_IN_FAULT_TC] = f_tc;
    input_snapshot[REG_IN_FAULT_CT] = f_ct;
    input_snapshot[REG_IN_FAULT_AIN] = f_ain;
    input_snapshot[REG_IN_FAULT_PROX] = f_prox;
    input_snapshot[REG_IN_FAULT_SYS] = f_sys;
    input_snapshot[REG_IN_FAULT_ACK] = (fault_acknowledged && (f_overall & FAULT_OVERALL_MASTER)) ? 1 : 0;

    /* Zone status registers (0x0028..0x002F): 0=Disabled, 1=Working, 2=Fault */
    uint16_t tc_en = thermocouple_get_enable_mask();
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        if (!(tc_en & (1U << i)))
        {
            input_snapshot[REG_IN_ZONE_STATUS_BASE + i] = ZONE_STATUS_DISABLED;
        }
        else if ((f_tc & (1U << i)) || !thermocouple_is_valid(i))
        {
            input_snapshot[REG_IN_ZONE_STATUS_BASE + i] = ZONE_STATUS_FAULT;
        }
        else
        {
            input_snapshot[REG_IN_ZONE_STATUS_BASE + i] = ZONE_STATUS_WORKING;
        }
    }

    /* Meterage registers (32-bit: 0.1 m and whole meters) */
    uint32_t met_c10 = proximity_get_meterage_c10();
    input_snapshot[REG_IN_METERAGE_HI] = (uint16_t)(met_c10 >> 16);
    input_snapshot[REG_IN_METERAGE_LO] = (uint16_t)(met_c10 & 0xFFFF);
    uint32_t met_m = proximity_get_meterage_m();
    input_snapshot[REG_IN_METERAGE_M_HI] = (uint16_t)(met_m >> 16);
    input_snapshot[REG_IN_METERAGE_M_LO] = (uint16_t)(met_m & 0xFFFF);
}

bool register_map_read_input(uint16_t addr, uint16_t *out)
{
    if (addr >= (sizeof(input_snapshot) / sizeof(input_snapshot[0])))
    {
        return false;
    }
    *out = input_snapshot[addr];
    return true;
}

bool register_map_read_holding(uint16_t addr, uint16_t *out)
{
    if (addr == REG_HOLD_RELAY_MASK)
    {
        *out = relay_get_mask();
        return true;
    }
    if (addr == REG_HOLD_PWM1_DUTY)
    {
        *out = pwm_output_get(0);
        return true;
    }
    if (addr == REG_HOLD_PWM2_DUTY)
    {
        *out = pwm_output_get(1);
        return true;
    }

    if (addr == REG_HOLD_FAULT_CLEAR)
    {
        *out = 0;
        return true;
    }
    if (addr == REG_HOLD_EEPROM_CMD)
    {
        *out = 0;
        return true;
    }
    if (addr == REG_HOLD_EEPROM_STATUS)
    {
        *out = flash_eeprom_get_status();
        return true;
    }
    if (addr == REG_HOLD_FAULT_ACK)
    {
        *out = fault_acknowledged ? 1 : 0;
        return true;
    }
    if (addr == 0x0107)
    {
        *out = 0;
        return true;
    }

    if (addr == REG_HOLD_TC_ENABLE_MASK)
    {
        *out = thermocouple_get_enable_mask();
        return true;
    }
    if (addr == REG_HOLD_CT_ENABLE_MASK)
    {
        *out = current_monitor_get_enable_mask();
        return true;
    }
    if (addr == REG_HOLD_AIN_ENABLE_MASK)
    {
        *out = analog_input_get_enable_mask();
        return true;
    }
    if (addr == REG_HOLD_PWM_ENABLE_MASK)
    {
        *out = pwm_output_get_enable_mask();
        return true;
    }
    if (addr == REG_HOLD_PROX_ENABLE_MASK)
    {
        *out = proximity_get_enable_mask();
        return true;
    }

    if (addr == REG_HOLD_CT_BIAS_MV)
    {
        *out = current_monitor_get_bias_mv();
        return true;
    }
    if (addr == REG_HOLD_CT_TURNS_RATIO)
    {
        *out = current_monitor_get_turns_ratio();
        return true;
    }
    if (addr == REG_HOLD_CT_BURDEN_OHM_X10)
    {
        *out = current_monitor_get_burden_ohm_x10();
        return true;
    }

    if (addr >= REG_HOLD_PID_MODE_BASE && addr < REG_HOLD_PID_MODE_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_mode(addr - REG_HOLD_PID_MODE_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_SETPOINT_BASE && addr < REG_HOLD_PID_SETPOINT_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_setpoint(addr - REG_HOLD_PID_SETPOINT_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_KP_BASE && addr < REG_HOLD_PID_KP_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_kp(addr - REG_HOLD_PID_KP_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_KI_BASE && addr < REG_HOLD_PID_KI_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_ki(addr - REG_HOLD_PID_KI_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_KD_BASE && addr < REG_HOLD_PID_KD_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_kd(addr - REG_HOLD_PID_KD_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_DEADZONE_BASE && addr < REG_HOLD_PID_DEADZONE_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_deadzone(addr - REG_HOLD_PID_DEADZONE_BASE);
        return true;
    }
    if (addr >= REG_HOLD_TC_OFFSET_BASE && addr < REG_HOLD_TC_OFFSET_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)thermocouple_get_offset(addr - REG_HOLD_TC_OFFSET_BASE);
        return true;
    }
    if (addr >= REG_HOLD_HEATER_RELAY_BASE && addr < REG_HOLD_HEATER_RELAY_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_heater_relay(addr - REG_HOLD_HEATER_RELAY_BASE);
        return true;
    }
    if (addr >= REG_HOLD_BLOWER_RELAY_BASE && addr < REG_HOLD_BLOWER_RELAY_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_blower_relay(addr - REG_HOLD_BLOWER_RELAY_BASE);
        return true;
    }
    if (addr >= REG_HOLD_MANUAL_DUTY_BASE && addr < REG_HOLD_MANUAL_DUTY_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_manual_duty(addr - REG_HOLD_MANUAL_DUTY_BASE);
        return true;
    }

    if (addr == REG_HOLD_AIN1_DIVIDER)
    {
        *out = analog_input_get_divider_ratio_x1000(0);
        return true;
    }
    if (addr == REG_HOLD_AIN2_DIVIDER)
    {
        *out = analog_input_get_divider_ratio_x1000(1);
        return true;
    }
    if (addr == REG_HOLD_SPEED_PROX_SRC)
    {
        *out = proximity_get_speed_src();
        return true;
    }
    if (addr == REG_HOLD_SPEED_SETPOINT)
    {
        *out = proximity_get_speed_setpoint();
        return true;
    }
    if (addr == REG_HOLD_MACHINE_RUN)
    {
        *out = machine_run_state;
        return true;
    }
    if (addr == REG_HOLD_PULSES_PER_METER)
    {
        *out = proximity_get_pulses_per_meter();
        return true;
    }
    if (addr == REG_HOLD_RESET_METERAGE)
    {
        *out = 0;
        return true;
    }

    /* Extended Thermocontroller Parameters */
    if (addr >= REG_HOLD_PID_ACTION_BASE && addr < REG_HOLD_PID_ACTION_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_action(addr - REG_HOLD_PID_ACTION_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_HYSTERESIS_BASE && addr < REG_HOLD_PID_HYSTERESIS_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_hysteresis(addr - REG_HOLD_PID_HYSTERESIS_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_CYCLE_TIME_BASE && addr < REG_HOLD_PID_CYCLE_TIME_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_cycle_time(addr - REG_HOLD_PID_CYCLE_TIME_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_SV_HIGH_BASE && addr < REG_HOLD_PID_SV_HIGH_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)pid_get_sv_high_limit(addr - REG_HOLD_PID_SV_HIGH_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_SV_LOW_BASE && addr < REG_HOLD_PID_SV_LOW_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)pid_get_sv_low_limit(addr - REG_HOLD_PID_SV_LOW_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_PV_OFFSET_BASE && addr < REG_HOLD_PID_PV_OFFSET_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)pid_get_pv_offset(addr - REG_HOLD_PID_PV_OFFSET_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_FAIL_SAFE_BASE && addr < REG_HOLD_PID_FAIL_SAFE_BASE + NUM_THERMOCOUPLES)
    {
        *out = pid_get_fail_safe_duty(addr - REG_HOLD_PID_FAIL_SAFE_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_ALARM_HIGH_BASE && addr < REG_HOLD_PID_ALARM_HIGH_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)pid_get_alarm_high(addr - REG_HOLD_PID_ALARM_HIGH_BASE);
        return true;
    }
    if (addr >= REG_HOLD_PID_ALARM_LOW_BASE && addr < REG_HOLD_PID_ALARM_LOW_BASE + NUM_THERMOCOUPLES)
    {
        *out = (uint16_t)pid_get_alarm_low(addr - REG_HOLD_PID_ALARM_LOW_BASE);
        return true;
    }

    /* Names (UTF-8 / Persian supported) */
    if (addr >= REG_HOLD_NAME_NODE_BASE && addr < REG_HOLD_NAME_NODE_BASE + REG_HOLD_NAME_REGS_PER_NAME)
    {
        *out = pack_string_reg(flash_eeprom_get_node_name(), addr - REG_HOLD_NAME_NODE_BASE);
        return true;
    }
    if (addr >= REG_HOLD_NAME_ZONE_BASE && addr < REG_HOLD_NAME_ZONE_BASE + (NUM_THERMOCOUPLES * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_ZONE_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_ZONE_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        *out = pack_string_reg(flash_eeprom_get_zone_name(ch), reg_idx);
        return true;
    }
    if (addr >= REG_HOLD_NAME_AIN_BASE && addr < REG_HOLD_NAME_AIN_BASE + (2 * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_AIN_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_AIN_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        *out = pack_string_reg(flash_eeprom_get_ain_name(ch), reg_idx);
        return true;
    }
    if (addr >= REG_HOLD_NAME_PWM_BASE && addr < REG_HOLD_NAME_PWM_BASE + (2 * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_PWM_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_PWM_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        *out = pack_string_reg(flash_eeprom_get_pwm_name(ch), reg_idx);
        return true;
    }

    /* PWM PID Configuration Registers */
    if (addr == REG_HOLD_PWM1_PID_MODE) { *out = pwm_pid_get_mode(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_SRC) { *out = pwm_pid_get_src(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_SETPOINT) { *out = pwm_pid_get_setpoint(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_KP) { *out = pwm_pid_get_kp(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_KI) { *out = pwm_pid_get_ki(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_KD) { *out = pwm_pid_get_kd(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_MIN_DUTY) { *out = pwm_pid_get_min_duty(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_MAX_DUTY) { *out = pwm_pid_get_max_duty(0); return true; }

    if (addr == REG_HOLD_PWM2_PID_MODE) { *out = pwm_pid_get_mode(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_SRC) { *out = pwm_pid_get_src(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_SETPOINT) { *out = pwm_pid_get_setpoint(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_KP) { *out = pwm_pid_get_kp(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_KI) { *out = pwm_pid_get_ki(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_KD) { *out = pwm_pid_get_kd(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_MIN_DUTY) { *out = pwm_pid_get_min_duty(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_MAX_DUTY) { *out = pwm_pid_get_max_duty(1); return true; }

    if (addr == REG_HOLD_PWM1_PID_SCALE_MUL) { *out = pwm_pid_get_scale_mul(0); return true; }
    if (addr == REG_HOLD_PWM1_PID_SCALE_DIV) { *out = pwm_pid_get_scale_div(0); return true; }
    if (addr == REG_HOLD_PWM2_PID_SCALE_MUL) { *out = pwm_pid_get_scale_mul(1); return true; }
    if (addr == REG_HOLD_PWM2_PID_SCALE_DIV) { *out = pwm_pid_get_scale_div(1); return true; }

    return false;
}

bool register_map_write_holding(uint16_t addr, uint16_t value)
{
    if (addr == REG_HOLD_RELAY_MASK)
    {
        relay_set_mask(value);
        return true;
    }
    if (addr == REG_HOLD_PWM1_DUTY)
    {
        pwm_output_set(0, value);
        return true;
    }
    if (addr == REG_HOLD_PWM2_DUTY)
    {
        pwm_output_set(1, value);
        return true;
    }

    if (addr == REG_HOLD_FAULT_CLEAR)
    {
        if (value != 0)
        {
            register_map_clear_all_faults();
        }
        return true;
    }

    if (addr == REG_HOLD_EEPROM_CMD)
    {
        if (value == EEPROM_CMD_SAVE)
        {
            flash_eeprom_save();
        }
        else if (value == EEPROM_CMD_RESET)
        {
            flash_eeprom_factory_reset();
            /* Refresh in-memory services from newly loaded defaults */
            thermocouple_init();
            current_monitor_init();
            analog_input_init();
            pwm_output_init();
            proximity_init();
            pid_control_init();
            pwm_pid_init();
        }
        return true;
    }

    if (addr == REG_HOLD_FAULT_ACK)
    {
        if (value != 0)
        {
            register_map_ack_faults();
        }
        return true;
    }
    if (addr == 0x0107)
    {
        return true;
    }

    if (addr == REG_HOLD_TC_ENABLE_MASK)
    {
        thermocouple_set_enable_mask(value);
        return true;
    }
    if (addr == REG_HOLD_CT_ENABLE_MASK)
    {
        current_monitor_set_enable_mask(value);
        return true;
    }
    if (addr == REG_HOLD_AIN_ENABLE_MASK)
    {
        analog_input_set_enable_mask(value);
        return true;
    }
    if (addr == REG_HOLD_PWM_ENABLE_MASK)
    {
        pwm_output_set_enable_mask(value);
        return true;
    }
    if (addr == REG_HOLD_PROX_ENABLE_MASK)
    {
        proximity_set_enable_mask(value);
        return true;
    }

    if (addr == REG_HOLD_CT_BIAS_MV)
    {
        current_monitor_set_bias_mv(value);
        return true;
    }
    if (addr == REG_HOLD_CT_TURNS_RATIO)
    {
        current_monitor_set_turns_ratio(value);
        return true;
    }
    if (addr == REG_HOLD_CT_BURDEN_OHM_X10)
    {
        current_monitor_set_burden_ohm_x10(value);
        return true;
    }

    if (addr >= REG_HOLD_PID_MODE_BASE && addr < REG_HOLD_PID_MODE_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_mode(addr - REG_HOLD_PID_MODE_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_SETPOINT_BASE && addr < REG_HOLD_PID_SETPOINT_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_setpoint(addr - REG_HOLD_PID_SETPOINT_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_KP_BASE && addr < REG_HOLD_PID_KP_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_kp(addr - REG_HOLD_PID_KP_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_KI_BASE && addr < REG_HOLD_PID_KI_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_ki(addr - REG_HOLD_PID_KI_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_KD_BASE && addr < REG_HOLD_PID_KD_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_kd(addr - REG_HOLD_PID_KD_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_DEADZONE_BASE && addr < REG_HOLD_PID_DEADZONE_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_deadzone(addr - REG_HOLD_PID_DEADZONE_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_TC_OFFSET_BASE && addr < REG_HOLD_TC_OFFSET_BASE + NUM_THERMOCOUPLES)
    {
        thermocouple_set_offset(addr - REG_HOLD_TC_OFFSET_BASE, (int16_t)value);
        return true;
    }
    if (addr >= REG_HOLD_HEATER_RELAY_BASE && addr < REG_HOLD_HEATER_RELAY_BASE + NUM_THERMOCOUPLES)
    {
        return pid_set_heater_relay(addr - REG_HOLD_HEATER_RELAY_BASE, value);
    }
    if (addr >= REG_HOLD_BLOWER_RELAY_BASE && addr < REG_HOLD_BLOWER_RELAY_BASE + NUM_THERMOCOUPLES)
    {
        return pid_set_blower_relay(addr - REG_HOLD_BLOWER_RELAY_BASE, value);
    }
    if (addr >= REG_HOLD_MANUAL_DUTY_BASE && addr < REG_HOLD_MANUAL_DUTY_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_manual_duty(addr - REG_HOLD_MANUAL_DUTY_BASE, value);
        return true;
    }

    if (addr == REG_HOLD_AIN1_DIVIDER)
    {
        analog_input_set_divider_ratio_x1000(0, value);
        return true;
    }
    if (addr == REG_HOLD_AIN2_DIVIDER)
    {
        analog_input_set_divider_ratio_x1000(1, value);
        return true;
    }
    if (addr == REG_HOLD_SPEED_PROX_SRC)
    {
        proximity_set_speed_src(value);
        return true;
    }
    if (addr == REG_HOLD_SPEED_SETPOINT)
    {
        proximity_set_speed_setpoint(value);
        return true;
    }
    if (addr == REG_HOLD_MACHINE_RUN)
    {
        machine_run_state = (value != 0) ? 1 : 0;
        return true;
    }
    if (addr == REG_HOLD_PULSES_PER_METER)
    {
        proximity_set_pulses_per_meter(value);
        return true;
    }
    if (addr == REG_HOLD_RESET_METERAGE)
    {
        if (value != 0)
        {
            proximity_reset_meterage();
        }
        return true;
    }

    /* Extended Thermocontroller Parameters */
    if (addr >= REG_HOLD_PID_ACTION_BASE && addr < REG_HOLD_PID_ACTION_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_action(addr - REG_HOLD_PID_ACTION_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_HYSTERESIS_BASE && addr < REG_HOLD_PID_HYSTERESIS_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_hysteresis(addr - REG_HOLD_PID_HYSTERESIS_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_CYCLE_TIME_BASE && addr < REG_HOLD_PID_CYCLE_TIME_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_cycle_time(addr - REG_HOLD_PID_CYCLE_TIME_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_SV_HIGH_BASE && addr < REG_HOLD_PID_SV_HIGH_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_sv_high_limit(addr - REG_HOLD_PID_SV_HIGH_BASE, (int16_t)value);
        return true;
    }
    if (addr >= REG_HOLD_PID_SV_LOW_BASE && addr < REG_HOLD_PID_SV_LOW_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_sv_low_limit(addr - REG_HOLD_PID_SV_LOW_BASE, (int16_t)value);
        return true;
    }
    if (addr >= REG_HOLD_PID_PV_OFFSET_BASE && addr < REG_HOLD_PID_PV_OFFSET_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_pv_offset(addr - REG_HOLD_PID_PV_OFFSET_BASE, (int16_t)value);
        return true;
    }
    if (addr >= REG_HOLD_PID_FAIL_SAFE_BASE && addr < REG_HOLD_PID_FAIL_SAFE_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_fail_safe_duty(addr - REG_HOLD_PID_FAIL_SAFE_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_PID_ALARM_HIGH_BASE && addr < REG_HOLD_PID_ALARM_HIGH_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_alarm_high(addr - REG_HOLD_PID_ALARM_HIGH_BASE, (int16_t)value);
        return true;
    }
    if (addr >= REG_HOLD_PID_ALARM_LOW_BASE && addr < REG_HOLD_PID_ALARM_LOW_BASE + NUM_THERMOCOUPLES)
    {
        pid_set_alarm_low(addr - REG_HOLD_PID_ALARM_LOW_BASE, (int16_t)value);
        return true;
    }

    /* Names (UTF-8 / Persian supported) */
    if (addr >= REG_HOLD_NAME_NODE_BASE && addr < REG_HOLD_NAME_NODE_BASE + REG_HOLD_NAME_REGS_PER_NAME)
    {
        nv_config_t *cfg = flash_eeprom_get_mutable();
        unpack_string_reg(cfg->node_name, sizeof(cfg->node_name), addr - REG_HOLD_NAME_NODE_BASE, value);
        return true;
    }
    if (addr >= REG_HOLD_NAME_ZONE_BASE && addr < REG_HOLD_NAME_ZONE_BASE + (NUM_THERMOCOUPLES * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_ZONE_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_ZONE_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        nv_config_t *cfg = flash_eeprom_get_mutable();
        unpack_string_reg(cfg->zone_name[ch], sizeof(cfg->zone_name[ch]), reg_idx, value);
        return true;
    }
    if (addr >= REG_HOLD_NAME_AIN_BASE && addr < REG_HOLD_NAME_AIN_BASE + (2 * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_AIN_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_AIN_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        nv_config_t *cfg = flash_eeprom_get_mutable();
        unpack_string_reg(cfg->ain_name[ch], sizeof(cfg->ain_name[ch]), reg_idx, value);
        return true;
    }
    if (addr >= REG_HOLD_NAME_PWM_BASE && addr < REG_HOLD_NAME_PWM_BASE + (2 * REG_HOLD_NAME_REGS_PER_NAME))
    {
        uint8_t ch = (addr - REG_HOLD_NAME_PWM_BASE) / REG_HOLD_NAME_REGS_PER_NAME;
        uint8_t reg_idx = (addr - REG_HOLD_NAME_PWM_BASE) % REG_HOLD_NAME_REGS_PER_NAME;
        nv_config_t *cfg = flash_eeprom_get_mutable();
        unpack_string_reg(cfg->pwm_name[ch], sizeof(cfg->pwm_name[ch]), reg_idx, value);
        return true;
    }

    /* PWM PID Configuration Registers */
    if (addr == REG_HOLD_PWM1_PID_MODE) { pwm_pid_set_mode(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_SRC) { pwm_pid_set_src(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_SETPOINT) { pwm_pid_set_setpoint(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_KP) { pwm_pid_set_kp(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_KI) { pwm_pid_set_ki(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_KD) { pwm_pid_set_kd(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_MIN_DUTY) { pwm_pid_set_min_duty(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_MAX_DUTY) { pwm_pid_set_max_duty(0, value); return true; }

    if (addr == REG_HOLD_PWM2_PID_MODE) { pwm_pid_set_mode(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_SRC) { pwm_pid_set_src(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_SETPOINT) { pwm_pid_set_setpoint(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_KP) { pwm_pid_set_kp(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_KI) { pwm_pid_set_ki(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_KD) { pwm_pid_set_kd(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_MIN_DUTY) { pwm_pid_set_min_duty(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_MAX_DUTY) { pwm_pid_set_max_duty(1, value); return true; }

    if (addr == REG_HOLD_PWM1_PID_SCALE_MUL) { pwm_pid_set_scale_mul(0, value); return true; }
    if (addr == REG_HOLD_PWM1_PID_SCALE_DIV) { pwm_pid_set_scale_div(0, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_SCALE_MUL) { pwm_pid_set_scale_mul(1, value); return true; }
    if (addr == REG_HOLD_PWM2_PID_SCALE_DIV) { pwm_pid_set_scale_div(1, value); return true; }

    return false;
}

bool register_map_read_coil(uint16_t addr, bool *out)
{
    if (addr < NUM_RELAYS)
    {
        *out = relay_get((uint8_t)addr);
        return true;
    }
    if (addr == COIL_MACHINE_RUN)
    {
        *out = (machine_run_state != 0);
        return true;
    }
    if (addr == COIL_RESET_METERAGE)
    {
        *out = false;
        return true;
    }
    if (addr == COIL_FAULT_ACK)
    {
        *out = fault_acknowledged;
        return true;
    }
    if (addr == COIL_FAULT_CLEAR)
    {
        *out = false;
        return true;
    }
    return false;
}

bool register_map_write_coil(uint16_t addr, bool value)
{
    if (addr < NUM_RELAYS)
    {
        relay_set((uint8_t)addr, value);
        return true;
    }
    if (addr == COIL_MACHINE_RUN)
    {
        machine_run_state = value ? 1 : 0;
        return true;
    }
    if (addr == COIL_RESET_METERAGE)
    {
        if (value)
        {
            proximity_reset_meterage();
        }
        return true;
    }
    if (addr == COIL_FAULT_ACK)
    {
        if (value)
        {
            register_map_ack_faults();
        }
        return true;
    }
    if (addr == COIL_FAULT_CLEAR)
    {
        if (value)
        {
            register_map_clear_all_faults();
        }
        return true;
    }
    return false;
}

bool register_map_read_discrete_input(uint16_t addr, bool *out)
{
    if (addr == 0)
    {
        *out = hal_gpio_read(PROX1_PORT, PROX1_PIN);
        return true;
    }
    else if (addr == 1)
    {
        *out = hal_gpio_read(PROX2_PORT, PROX2_PIN);
        return true;
    }
    return false;
}

uint16_t register_map_get_fault_overall(void)
{
    return input_snapshot[REG_IN_FAULT_OVERALL];
}

bool register_map_is_machine_running(void)
{
    return (machine_run_state != 0);
}

void register_map_set_machine_running(bool running)
{
    machine_run_state = running ? 1 : 0;
}

void register_map_ack_faults(void)
{
    fault_acknowledged = true;
    input_snapshot[REG_IN_FAULT_ACK] = 1;
    if (input_snapshot[REG_IN_FAULT_OVERALL] & FAULT_OVERALL_MASTER)
    {
        input_snapshot[REG_IN_FAULT_OVERALL] |= FAULT_OVERALL_ACK;
    }
}

bool register_map_is_fault_acknowledged(void)
{
    return fault_acknowledged;
}

void register_map_clear_all_faults(void)
{
    thermocouple_clear_faults();
    current_monitor_clear_faults();
    analog_input_clear_faults();
    proximity_clear_faults();
    flash_eeprom_clear_faults();
    hal_clock_clear_fault();
    fault_acknowledged = false;
    input_snapshot[REG_IN_FAULT_ACK] = 0;
    if (input_snapshot[REG_IN_FAULT_OVERALL] & FAULT_OVERALL_ACK)
    {
        input_snapshot[REG_IN_FAULT_OVERALL] &= ~FAULT_OVERALL_ACK;
    }
}
