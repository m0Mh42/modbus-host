/**
 * @file    register_map.h
 * @brief   Modbus data model: bridges the protocol layer to the application's
 *          sensors and actuators.
 */
#ifndef REGISTER_MAP_H
#define REGISTER_MAP_H

#include <stdint.h>
#include <stdbool.h>

/* ============================ ADDRESS PLAN ============================ */
/* INPUT REGISTERS (FC04, read-only): */
#define REG_IN_TC_TEMP_BASE 0x0000   /* 8 ch, 0.1 C, signed */
#define REG_IN_TC_VALID_BASE 0x0008  /* 8 ch, 0/1 */
#define REG_IN_CURRENT_BASE 0x0010   /* 8 ch, mA (True RMS) */
#define REG_IN_AIN_BASE 0x0018       /* 2 ch, mV, divider-corrected */
#define REG_IN_CJC_TEMP 0x001A       /* Cold Junction (DS18B20) temp, 0.1 C, signed */
#define REG_IN_PROX1_COUNT_HI 0x0020 /* 32-bit pulses */
#define REG_IN_PROX1_COUNT_LO 0x0021
#define REG_IN_PROX2_COUNT_HI 0x0022
#define REG_IN_PROX2_COUNT_LO 0x0023
#define REG_IN_PROX1_SPEED 0x0024 /* pulses/sec */
#define REG_IN_PROX2_SPEED 0x0025
#define REG_IN_LINE_SPEED 0x0026  /* Line speed in 0.1 m/min */
#define REG_IN_SPEED_HZ 0x0027    /* Effective speed in Hz */
#define REG_IN_ZONE_STATUS_BASE 0x0028 /* 8 zones: 0=Disabled, 1=Working, 2=Fault */

#define ZONE_STATUS_DISABLED 0
#define ZONE_STATUS_WORKING 1
#define ZONE_STATUS_FAULT 2

/* Fault Input Registers (FC04): */
#define REG_IN_FAULT_OVERALL 0x0030 /* bit0=TC, bit1=CT, bit2=AIN, bit3=PROX, bit4=SYS, bit15=Any */
#define REG_IN_FAULT_TC 0x0031      /* bits 0..7: TC1..8 fault/open, bit 8: CJC DS18B20 fault */
#define REG_IN_FAULT_CT 0x0032      /* bits 0..7: CT1..8 overcurrent/saturation, bit 8: bias fault */
#define REG_IN_FAULT_AIN 0x0033     /* bit 0: AIN1 overvoltage, bit 1: AIN2 overvoltage */
#define REG_IN_FAULT_PROX 0x0034    /* bit 0: PROX1 overspeed, bit 1: PROX2 overspeed */
#define REG_IN_FAULT_SYS 0x0035     /* bit 0: EEPROM CRC, bit 1: Flash write, bit 2: Crystal error */
#define FAULT_SYS_EEPROM_CRC (1U << 0)
#define FAULT_SYS_FLASH_WRITE (1U << 1)
#define FAULT_SYS_CRYSTAL_ERROR (1U << 2)

/* Meterage Input Registers (FC04, 32-bit): */
#define REG_IN_METERAGE_HI 0x0036   /* Line meterage in 0.1 m resolution, high word */
#define REG_IN_METERAGE_LO 0x0037   /* low word */
#define REG_IN_METERAGE_M_HI 0x0038 /* Line meterage in whole meters, high word */
#define REG_IN_METERAGE_M_LO 0x0039 /* low word */
#define REG_IN_FAULT_ACK 0x003A     /* 0=Unacknowledged, 1=Acknowledged */

#define FAULT_OVERALL_TC (1U << 0)
#define FAULT_OVERALL_CT (1U << 1)
#define FAULT_OVERALL_AIN (1U << 2)
#define FAULT_OVERALL_PROX (1U << 3)
#define FAULT_OVERALL_SYS (1U << 4)
#define FAULT_OVERALL_ACK (1U << 14)
#define FAULT_OVERALL_MASTER (1U << 15)

/* HOLDING REGISTERS (FC03 read / FC06,FC16 write): */
#define REG_HOLD_RELAY_MASK 0x0100 /* bit n -> relay n */
#define REG_HOLD_PWM1_DUTY 0x0101  /* 0..PWM_MAX_DUTY */
#define REG_HOLD_PWM2_DUTY 0x0102
#define REG_HOLD_FAULT_CLEAR 0x0103   /* Write 1 or 0xFFFF to clear latched faults */
#define REG_HOLD_EEPROM_CMD 0x0104    /* 0x55AA = Save, 0xDEAD = Factory Reset */
#define REG_HOLD_EEPROM_STATUS 0x0105 /* 0=OK, 1=Saved, 2=Dirty/Pending, 0xFF=Error */
#define REG_HOLD_FAULT_ACK 0x0106     /* Write 1 or 0xFFFF to acknowledge active faults */

/* Module Processing Enable Bitmasks */
#define REG_HOLD_TC_ENABLE_MASK 0x0108   /* Bits 0..7 enable TC1..TC8 */
#define REG_HOLD_CT_ENABLE_MASK 0x0109   /* Bits 0..7 enable CT1..CT8 */
#define REG_HOLD_AIN_ENABLE_MASK 0x010A  /* Bits 0..1 enable AIN1..AIN2 */
#define REG_HOLD_PWM_ENABLE_MASK 0x010B  /* Bits 0..1 enable PWM1..PWM2 */
#define REG_HOLD_PROX_ENABLE_MASK 0x010C /* Bits 0..1 enable PROX1..PROX2 */

/* CT Calibration Holding Registers */
#define REG_HOLD_CT_BIAS_MV 0x010D        /* TL431 bias in mV (nominally 2495) */
#define REG_HOLD_CT_TURNS_RATIO 0x010E    /* CT turns ratio (default 2375) */
#define REG_HOLD_CT_BURDEN_OHM_X10 0x010F /* Burden resistance in 0.1 ohm (default 620) */

/* PID & Configuration Holding Registers (8 channels each) */
#define REG_HOLD_PID_MODE_BASE 0x0110     /* 0=Manual, 1=Auto */
#define REG_HOLD_PID_SETPOINT_BASE 0x0120 /* 0.1 C */
#define REG_HOLD_PID_KP_BASE 0x0130       /* Kp * 100 */
#define REG_HOLD_PID_KI_BASE 0x0140       /* Ki * 100 */
#define REG_HOLD_PID_KD_BASE 0x0150       /* Kd * 100 */
#define REG_HOLD_PID_DEADZONE_BASE 0x0160 /* 0.1 C */
#define REG_HOLD_TC_OFFSET_BASE 0x0170    /* DC offset uV (signed 16-bit) */
#define REG_HOLD_HEATER_RELAY_BASE 0x0180 /* 0..15 or 0xFFFF for none */
#define REG_HOLD_BLOWER_RELAY_BASE 0x0190 /* 0..15 or 0xFFFF for none */
#define REG_HOLD_MANUAL_DUTY_BASE 0x01A0  /* -1000..1000 (signed) */

/* Voltage Divider & Speed Holding Registers */
#define REG_HOLD_AIN1_DIVIDER 0x01B0 /* AIN1 divider ratio * 1000 (default 4917) */
#define REG_HOLD_AIN2_DIVIDER 0x01B1 /* AIN2 divider ratio * 1000 (default 4917) */
#define REG_HOLD_SPEED_PROX_SRC 0x01B2 /* Speed input: 0=PROX1, 1=PROX2, 2=BOTH (avg) */
#define REG_HOLD_SPEED_SETPOINT 0x01B4     /* Set speed input (0.1 m/min) */
#define REG_HOLD_MACHINE_RUN 0x01B5        /* Machine Run command: 0=Stop, 1=Run */
#define REG_HOLD_PULSES_PER_METER 0x01B6   /* Pulses per meter calibration (default 100) */
#define REG_HOLD_RESET_METERAGE 0x01B7     /* Write 1 to zero line meterage */

#define SPEED_SRC_PROX1 0
#define SPEED_SRC_PROX2 1
#define SPEED_SRC_BOTH  2

/* Extended Thermocontroller Holding Registers (8 channels each) */
#define REG_HOLD_PID_ACTION_BASE 0x01C0     /* 0=Heating, 1=Cooling */
#define REG_HOLD_PID_HYSTERESIS_BASE 0x01C8 /* 0.1 C */
#define REG_HOLD_PID_CYCLE_TIME_BASE 0x01D0 /* Proportional cycle time in seconds (1..100) */
#define REG_HOLD_PID_SV_HIGH_BASE 0x01D8    /* SV High limit, 0.1 C */
#define REG_HOLD_PID_SV_LOW_BASE 0x01E0     /* SV Low limit, 0.1 C */
#define REG_HOLD_PID_PV_OFFSET_BASE 0x01E8  /* PV Calibration offset / shift, 0.1 C (signed) */
#define REG_HOLD_PID_FAIL_SAFE_BASE 0x01F0  /* Fail safe output on sensor open (0..1000) */
#define REG_HOLD_PID_ALARM_HIGH_BASE 0x01F8 /* High alarm threshold, 0.1 C */
#define REG_HOLD_PID_ALARM_LOW_BASE 0x0200  /* Low alarm threshold, 0.1 C */

/* PWM PID Configuration Holding Registers (PWM1 & PWM2) */
#define REG_HOLD_PWM1_PID_MODE 0x02E0     /* 0=Manual, 1=Auto PID */
#define REG_HOLD_PWM1_PID_SRC 0x02E1      /* Feedback src: 0=Line Speed, 1=AIN1, 2=AIN2, 3=PROX1, 4=PROX2, 5..12=TC1..TC8 */
#define REG_HOLD_PWM1_PID_SETPOINT 0x02E2 /* Target setpoint in scaled units (0 = track speed setpoint if Line Speed) */
#define REG_HOLD_PWM1_PID_KP 0x02E3       /* Kp * 100 */
#define REG_HOLD_PWM1_PID_KI 0x02E4       /* Ki * 100 */
#define REG_HOLD_PWM1_PID_KD 0x02E5       /* Kd * 100 */
#define REG_HOLD_PWM1_PID_MIN_DUTY 0x02E6 /* 0..1000 (0.0% - 100.0%) */
#define REG_HOLD_PWM1_PID_MAX_DUTY 0x02E7 /* 0..1000 (0.0% - 100.0%) */

#define REG_HOLD_PWM2_PID_MODE 0x02E8     /* 0=Manual, 1=Auto PID */
#define REG_HOLD_PWM2_PID_SRC 0x02E9      /* Feedback src: 0=Line Speed, 1=AIN1, 2=AIN2, 3=PROX1, 4=PROX2, 5..12=TC1..TC8 */
#define REG_HOLD_PWM2_PID_SETPOINT 0x02EA /* Target setpoint in scaled units (0 = track speed setpoint if Line Speed) */
#define REG_HOLD_PWM2_PID_KP 0x02EB       /* Kp * 100 */
#define REG_HOLD_PWM2_PID_KI 0x02EC       /* Ki * 100 */
#define REG_HOLD_PWM2_PID_KD 0x02ED       /* Kd * 100 */
#define REG_HOLD_PWM2_PID_MIN_DUTY 0x02EE /* 0..1000 (0.0% - 100.0%) */
#define REG_HOLD_PWM2_PID_MAX_DUTY 0x02EF /* 0..1000 (0.0% - 100.0%) */
#define REG_HOLD_PWM1_PID_SCALE_MUL 0x02F0 /* PWM1 feedback scale multiplier (default 1000) */
#define REG_HOLD_PWM1_PID_SCALE_DIV 0x02F1 /* PWM1 feedback scale divisor (default 1000) */
#define REG_HOLD_PWM2_PID_SCALE_MUL 0x02F2 /* PWM2 feedback scale multiplier (default 1000) */
#define REG_HOLD_PWM2_PID_SCALE_DIV 0x02F3 /* PWM2 feedback scale divisor (default 1000) */

/* COILS (FC01 read / FC05,FC15 write): */
#define COIL_RELAY_BASE 0x0000 /* relay 0..15 */
#define COIL_MACHINE_RUN 0x0010 /* coil 16: Machine Run (0=Stop, 1=Run) */
#define COIL_RESET_METERAGE 0x0011 /* coil 17: Reset Meterage Count */
#define COIL_FAULT_ACK 0x0012      /* coil 18: Acknowledge Active Faults */
#define COIL_FAULT_CLEAR 0x0013    /* coil 19: Clear Latched Faults */
/* ===================================================================== */

/** Refresh the cached input-register snapshot from the services. */
void register_map_update_inputs(void);

/** Read one input register (FC04). Returns false if address is unmapped. */
bool register_map_read_input(uint16_t addr, uint16_t *out);

/** Read one holding register (FC03). Returns false if unmapped. */
bool register_map_read_holding(uint16_t addr, uint16_t *out);

/** Write one holding register (FC06/FC16). Returns false if read-only/unmapped. */
bool register_map_write_holding(uint16_t addr, uint16_t value);

/** Coil access (relays + machine run). */
bool register_map_read_coil(uint16_t addr, bool *out);
bool register_map_write_coil(uint16_t addr, bool value);

/** Discrete input access (FC02). */
bool register_map_read_discrete_input(uint16_t addr, bool *out);

/** Machine run state getter/setter. */
bool register_map_is_machine_running(void);
void register_map_set_machine_running(bool running);

/** Get overall fault status bitmask (bit 15 = Master fault). */
uint16_t register_map_get_fault_overall(void);

/** Fault acknowledgment and clearing hooks. */
void register_map_ack_faults(void);
bool register_map_is_fault_acknowledged(void);
void register_map_clear_all_faults(void);

#endif /* REGISTER_MAP_H */
