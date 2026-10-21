/**
 * @file    flash_eeprom.h
 * @brief   Flash EEPROM emulation on GD32F303 on-chip Flash (Page 127).
 *
 * Stores PID settings, module enable masks, CT calibration (bias, turns, burden),
 * and AIN voltage divider factors.
 */
#ifndef FLASH_EEPROM_H
#define FLASH_EEPROM_H

#include <stdint.h>
#include <stdbool.h>

#define NV_CONFIG_MAGIC 0x54484552u /* ASCII 'THER' */
#define NV_CONFIG_VERSION 7

#define FLASH_EEPROM_PAGE_ADDR 0x0803F800u /* Page 127 of 256KB Flash */
#define FLASH_PAGE_SIZE 2048u              /* 2 KB */

#define NUM_PID_CHANNELS 8
#define NV_NAME_MAX_LEN 32 /* 32 bytes: supports up to 15-31 UTF-8 chars (Persian/Unicode) + null */

typedef struct
{
    uint16_t mode;         /* 0=Manual, 1=Auto PID */
    uint16_t feedback_src; /* 0=Line Speed, 1=AIN1, 2=AIN2, 3=PROX1, 4=PROX2, 5..12=TC1..TC8 */
    uint16_t setpoint;     /* Target setpoint in scaled engineering units */
    uint16_t kp;           /* Kp * 100 */
    uint16_t ki;           /* Ki * 100 */
    uint16_t kd;           /* Kd * 100 */
    uint16_t min_duty;     /* 0..1000 */
    uint16_t max_duty;     /* 0..1000 */
    uint16_t scale_mul;    /* Feedback scale multiplier (default 1000) */
    uint16_t scale_div;    /* Feedback scale divisor (default 1000) */
} __attribute__((packed)) nv_pwm_pid_cfg_t;

typedef struct
{
    uint16_t mode;         /* 0=Manual, 1=Auto, 2=ON/OFF */
    int16_t setpoint;      /* 0.1 C */
    uint16_t kp;           /* Kp * 100 */
    uint16_t ki;           /* Ki * 100 */
    uint16_t kd;           /* Kd * 100 */
    uint16_t deadzone;     /* 0.1 C */
    int16_t tc_offset_uv;  /* DC offset uV */
    uint16_t heater_relay; /* 0..15, 0x0020 (PWM1), 0x0021 (PWM2), or 0xFFFF */
    uint16_t blower_relay; /* 0..15 or 0xFFFF */
    int16_t manual_duty;   /* -1000..1000 */

    /* Extended thermocontroller parameters */
    uint16_t action;         /* 0=Heating (Reverse), 1=Cooling (Direct) */
    uint16_t hysteresis;     /* 0.1 C (ON/OFF and alarm deadband) */
    uint16_t cycle_time_s;   /* 1..100 s (default 2s for SSR, 20s for relay) */
    int16_t sv_high_limit;   /* 0.1 C (default 13700 = 1370.0 C) */
    int16_t sv_low_limit;    /* 0.1 C (default -2000 = -200.0 C) */
    int16_t pv_offset;       /* 0.1 C (sensor shift / bias calibration) */
    int16_t alarm_high;      /* 0.1 C */
    int16_t alarm_low;       /* 0.1 C */
    uint16_t fail_safe_duty; /* 0..1000 (power on sensor fault) */
} __attribute__((packed)) nv_pid_cfg_t;

typedef struct
{
    uint32_t magic;   /* NV_CONFIG_MAGIC */
    uint16_t version; /* NV_CONFIG_VERSION */
    uint16_t length;  /* Size of this struct */

    /* Module channel enable bitmasks */
    uint16_t tc_enable_mask;   /* Bits 0..7 for TC1..TC8 */
    uint16_t ct_enable_mask;   /* Bits 0..7 for CT1..CT8 */
    uint16_t ain_enable_mask;  /* Bits 0..1 for AIN1..AIN2 */
    uint16_t pwm_enable_mask;  /* Bits 0..1 for PWM1..PWM2 */
    uint16_t prox_enable_mask; /* Bits 0..1 for PROX1..PROX2 */

    /* CT calibration */
    uint16_t ct_bias_mv;        /* mV, default 2495 */
    uint16_t ct_turns_ratio;    /* e.g. 2375 */
    uint16_t ct_burden_ohm_x10; /* e.g. 620 for 62.0 ohm */

    /* AIN voltage divider calibration */
    uint16_t ain1_divider_x1000; /* e.g. 4917 for 4.917 */
    uint16_t ain2_divider_x1000; /* e.g. 4917 for 4.917 */

    /* Speed measurement & meterage configuration */
    uint16_t speed_prox_src;       /* 0=PROX1, 1=PROX2, 2=BOTH */
    uint16_t speed_setpoint;       /* Set speed input (0.1 m/min) */
    uint16_t pulses_per_meter;     /* Meterage calibration pulses/m (default 100) */

    /* PWM PID Configuration (2 channels) */
    nv_pwm_pid_cfg_t pwm_pid[2];

    /* PID configuration (8 channels) */
    nv_pid_cfg_t pid[NUM_PID_CHANNELS];

    /* Names (null-terminated UTF-8 / ASCII strings, max 32 bytes including null) */
    char node_name[NV_NAME_MAX_LEN];
    char zone_name[NUM_PID_CHANNELS][NV_NAME_MAX_LEN];
    char ain_name[2][NV_NAME_MAX_LEN];
    char pwm_name[2][NV_NAME_MAX_LEN];

    uint32_t crc32; /* CRC32 of preceding fields */
} __attribute__((packed)) nv_config_t;

/* Status codes for REG_HOLD_EEPROM_STATUS */
#define EEPROM_STATUS_OK 0
#define EEPROM_STATUS_SAVED 1
#define EEPROM_STATUS_DIRTY 2
#define EEPROM_STATUS_ERR 0xFF

/* Commands for REG_HOLD_EEPROM_CMD */
#define EEPROM_CMD_SAVE 0x55AA
#define EEPROM_CMD_RESET 0xDEAD

/** Initialize Flash EEPROM; load valid config or save defaults. */
bool flash_eeprom_init(void);

/** Get active read-only configuration. */
const nv_config_t *flash_eeprom_get(void);

/** Get active mutable configuration. */
nv_config_t *flash_eeprom_get_mutable(void);

/** Mark configuration dirty to trigger debounced auto-save. */
void flash_eeprom_mark_dirty(void);

/** Commit active configuration to Flash immediately. */
bool flash_eeprom_save(void);

/** Restore factory defaults and commit to Flash immediately. */
bool flash_eeprom_factory_reset(void);

/** Periodic housekeeping (handles 2-second debounce timer). */
void flash_eeprom_task(void);

/** Get current EEPROM status code. */
uint16_t flash_eeprom_get_status(void);

/** Get EEPROM fault bits: bit 0 = CRC/corrupt, bit 1 = flash write fail. */
uint16_t flash_eeprom_get_faults(void);

/** Clear EEPROM fault flags. */
void flash_eeprom_clear_faults(void);

/** Name getters (returns null-terminated string) */
const char *flash_eeprom_get_node_name(void);
const char *flash_eeprom_get_zone_name(uint8_t ch);
const char *flash_eeprom_get_ain_name(uint8_t ch);
const char *flash_eeprom_get_pwm_name(uint8_t ch);

/** Name setters (copies up to 31 bytes, null-terminated, marks dirty) */
void flash_eeprom_set_node_name(const char *name);
void flash_eeprom_set_zone_name(uint8_t ch, const char *name);
void flash_eeprom_set_ain_name(uint8_t ch, const char *name);
void flash_eeprom_set_pwm_name(uint8_t ch, const char *name);

#endif /* FLASH_EEPROM_H */
