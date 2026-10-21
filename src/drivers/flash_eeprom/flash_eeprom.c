/**
 * @file    flash_eeprom.c
 * @brief   Flash EEPROM emulation on GD32F303 Page 127.
 */
#include "flash_eeprom.h"
#include <string.h>
#include <stdio.h>

#ifndef UNIT_TEST
#include "gd32f30x.h"
#include "gd32f30x_fmc.h"
#endif
#include "hal_time.h"

#ifdef UNIT_TEST
static uint32_t mock_millis = 0;
uint32_t hal_millis(void)
{
    return mock_millis;
}
static uint8_t mock_flash_page[FLASH_PAGE_SIZE];
#define FLASH_CFG_PTR ((const nv_config_t *)mock_flash_page)
#else
#define FLASH_CFG_PTR ((const nv_config_t *)FLASH_EEPROM_PAGE_ADDR)
#endif

#define DEBOUNCE_DELAY_MS 2000u

static nv_config_t active_cfg;
static bool is_dirty = false;
static uint32_t dirty_timestamp = 0;
static uint16_t eeprom_status = EEPROM_STATUS_OK;
static uint16_t eeprom_faults = 0;

/* Simple software CRC-32 (IEEE 802.3 polynomial 0xEDB88320) */
static uint32_t calc_crc32(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

static void load_factory_defaults(nv_config_t *cfg)
{
    memset(cfg, 0, sizeof(nv_config_t));
    cfg->magic = NV_CONFIG_MAGIC;
    cfg->version = NV_CONFIG_VERSION;
    cfg->length = (uint16_t)sizeof(nv_config_t);

    cfg->tc_enable_mask = 0x00FFu;   /* All 8 TCs enabled */
    cfg->ct_enable_mask = 0x00FFu;   /* All 8 CTs enabled */
    cfg->ain_enable_mask = 0x0003u;  /* Both AINs enabled */
    cfg->pwm_enable_mask = 0x0003u;  /* Both PWMs enabled */
    cfg->prox_enable_mask = 0x0003u; /* Both PROXs enabled */

    cfg->ct_bias_mv = 2495u;       /* TL431 ~2.495V */
    cfg->ct_turns_ratio = 2375u;   /* 2375:1 */
    cfg->ct_burden_ohm_x10 = 620u; /* 62.0 ohm */

    cfg->ain1_divider_x1000 = 4917u; /* (4.7k + 1.2k) / 1.2k = 4.9167 */
    cfg->ain2_divider_x1000 = 4917u;

    cfg->speed_prox_src = 0;         /* 0=PROX1 */
    cfg->speed_setpoint = 0;
    cfg->pulses_per_meter = 100u;    /* default 100 pulses/m */

    for (uint8_t p = 0; p < 2; p++)
    {
        cfg->pwm_pid[p].mode = 0;         /* 0=Manual */
        cfg->pwm_pid[p].feedback_src = 0; /* 0=Line Speed */
        cfg->pwm_pid[p].setpoint = 0;     /* 0 (tracks global speed setpoint if Line Speed) */
        cfg->pwm_pid[p].kp = 100;         /* 1.00 */
        cfg->pwm_pid[p].ki = 10;          /* 0.10 */
        cfg->pwm_pid[p].kd = 0;           /* 0.00 */
        cfg->pwm_pid[p].min_duty = 0;
        cfg->pwm_pid[p].max_duty = 1000;  /* 100% */
        cfg->pwm_pid[p].scale_mul = 1000u; /* Multiplier (default 1000) */
        cfg->pwm_pid[p].scale_div = 1000u; /* Divisor (default 1000) */
    }

    for (uint8_t ch = 0; ch < NUM_PID_CHANNELS; ch++)
    {
        cfg->pid[ch].mode = 0;        /* Manual */
        cfg->pid[ch].setpoint = 2000; /* 200.0 C */
        cfg->pid[ch].kp = 500;        /* 5.00 */
        cfg->pid[ch].ki = 50;         /* 0.50 */
        cfg->pid[ch].kd = 100;        /* 1.00 */
        cfg->pid[ch].deadzone = 10;   /* 1.0 C */
        cfg->pid[ch].tc_offset_uv = 0;
        cfg->pid[ch].heater_relay = ch;      /* default 1:1 relay mapping */
        cfg->pid[ch].blower_relay = 0xFFFFu; /* none */
        cfg->pid[ch].manual_duty = 0;

        cfg->pid[ch].action = 0;            /* Heating (Reverse) */
        cfg->pid[ch].hysteresis = 20;       /* 2.0 C */
        cfg->pid[ch].cycle_time_s = 2;      /* 2 seconds */
        cfg->pid[ch].sv_high_limit = 13700; /* 1370.0 C */
        cfg->pid[ch].sv_low_limit = -2000;  /* -200.0 C */
        cfg->pid[ch].pv_offset = 0;
        cfg->pid[ch].alarm_high = 13700;
        cfg->pid[ch].alarm_low = -2000;
        cfg->pid[ch].fail_safe_duty = 0;
    }

    snprintf(cfg->node_name, sizeof(cfg->node_name), "Node-1");
    for (uint8_t ch = 0; ch < NUM_PID_CHANNELS; ch++)
    {
        snprintf(cfg->zone_name[ch], sizeof(cfg->zone_name[ch]), "Zone-%u", (unsigned)(ch + 1));
    }
    snprintf(cfg->ain_name[0], sizeof(cfg->ain_name[0]), "AIN-1");
    snprintf(cfg->ain_name[1], sizeof(cfg->ain_name[1]), "AIN-2");
    snprintf(cfg->pwm_name[0], sizeof(cfg->pwm_name[0]), "PWM-1");
    snprintf(cfg->pwm_name[1], sizeof(cfg->pwm_name[1]), "PWM-2");

    size_t payload_len = sizeof(nv_config_t) - sizeof(uint32_t);
    cfg->crc32 = calc_crc32((const uint8_t *)cfg, payload_len);
}

static bool flash_write_block(uint32_t page_addr, const void *src, size_t byte_count)
{
#ifndef UNIT_TEST
    fmc_unlock();

    fmc_flag_clear(FMC_FLAG_BANK0_END | FMC_FLAG_BANK0_WPERR | FMC_FLAG_BANK0_PGERR);

    fmc_state_enum state = fmc_page_erase(page_addr);
    if (state != FMC_READY)
    {
        fmc_lock();
        return false;
    }

    const uint32_t *p_data = (const uint32_t *)src;
    size_t words = (byte_count + 3) / 4;

    for (size_t i = 0; i < words; i++)
    {
        state = fmc_word_program(page_addr + (i * 4), p_data[i]);
        if (state != FMC_READY)
        {
            fmc_lock();
            return false;
        }
    }

    fmc_lock();
#else
    (void)page_addr;
    if (byte_count <= FLASH_PAGE_SIZE)
    {
        memcpy(mock_flash_page, src, byte_count);
    }
#endif
    return true;
}

bool flash_eeprom_init(void)
{
    const nv_config_t *flash_cfg = FLASH_CFG_PTR;

    /* Check magic, version, and length bounds */
    if (flash_cfg->magic == NV_CONFIG_MAGIC &&
        flash_cfg->version == NV_CONFIG_VERSION &&
        flash_cfg->length == sizeof(nv_config_t))
    {
        size_t payload_len = sizeof(nv_config_t) - sizeof(uint32_t);
        uint32_t expected_crc = calc_crc32((const uint8_t *)flash_cfg, payload_len);

        if (expected_crc == flash_cfg->crc32)
        {
            /* Valid configuration found */
            memcpy(&active_cfg, flash_cfg, sizeof(nv_config_t));
            eeprom_status = EEPROM_STATUS_OK;
            is_dirty = false;
            return true;
        }
    }

    /* Invalid, corrupted, or blank flash: restore defaults and persist */
    eeprom_faults |= 1u; /* Bit 0: CRC/corrupt fault */
    load_factory_defaults(&active_cfg);
    bool saved = flash_eeprom_save();
    eeprom_status = saved ? EEPROM_STATUS_SAVED : EEPROM_STATUS_ERR;
    return saved;
}

const nv_config_t *flash_eeprom_get(void)
{
    return &active_cfg;
}

nv_config_t *flash_eeprom_get_mutable(void)
{
    return &active_cfg;
}

void flash_eeprom_mark_dirty(void)
{
    is_dirty = true;
    dirty_timestamp = hal_millis();
    eeprom_status = EEPROM_STATUS_DIRTY;
}

bool flash_eeprom_save(void)
{
    active_cfg.magic = NV_CONFIG_MAGIC;
    active_cfg.version = NV_CONFIG_VERSION;
    active_cfg.length = (uint16_t)sizeof(nv_config_t);

    size_t payload_len = sizeof(nv_config_t) - sizeof(uint32_t);
    active_cfg.crc32 = calc_crc32((const uint8_t *)&active_cfg, payload_len);

    bool ok = flash_write_block(FLASH_EEPROM_PAGE_ADDR, &active_cfg, sizeof(nv_config_t));
    if (ok)
    {
        is_dirty = false;
        eeprom_status = EEPROM_STATUS_SAVED;
    }
    else
    {
        eeprom_faults |= 2u; /* Bit 1: Flash write error */
        eeprom_status = EEPROM_STATUS_ERR;
    }
    return ok;
}

bool flash_eeprom_factory_reset(void)
{
    load_factory_defaults(&active_cfg);
    return flash_eeprom_save();
}

void flash_eeprom_task(void)
{
    if (is_dirty)
    {
        uint32_t now = hal_millis();
        if ((now - dirty_timestamp) >= DEBOUNCE_DELAY_MS)
        {
            flash_eeprom_save();
        }
    }
}

uint16_t flash_eeprom_get_status(void)
{
    return eeprom_status;
}

uint16_t flash_eeprom_get_faults(void)
{
    return eeprom_faults;
}

void flash_eeprom_clear_faults(void)
{
    eeprom_faults = 0;
}

const char *flash_eeprom_get_node_name(void)
{
    return active_cfg.node_name;
}

const char *flash_eeprom_get_zone_name(uint8_t ch)
{
    if (ch < NUM_PID_CHANNELS)
    {
        return active_cfg.zone_name[ch];
    }
    return "";
}

const char *flash_eeprom_get_ain_name(uint8_t ch)
{
    if (ch < 2)
    {
        return active_cfg.ain_name[ch];
    }
    return "";
}

const char *flash_eeprom_get_pwm_name(uint8_t ch)
{
    if (ch < 2)
    {
        return active_cfg.pwm_name[ch];
    }
    return "";
}

void flash_eeprom_set_node_name(const char *name)
{
    if (!name)
        return;
    snprintf(active_cfg.node_name, sizeof(active_cfg.node_name), "%s", name);
    flash_eeprom_mark_dirty();
}

void flash_eeprom_set_zone_name(uint8_t ch, const char *name)
{
    if (ch < NUM_PID_CHANNELS && name)
    {
        snprintf(active_cfg.zone_name[ch], sizeof(active_cfg.zone_name[ch]), "%s", name);
        flash_eeprom_mark_dirty();
    }
}

void flash_eeprom_set_ain_name(uint8_t ch, const char *name)
{
    if (ch < 2 && name)
    {
        snprintf(active_cfg.ain_name[ch], sizeof(active_cfg.ain_name[ch]), "%s", name);
        flash_eeprom_mark_dirty();
    }
}

void flash_eeprom_set_pwm_name(uint8_t ch, const char *name)
{
    if (ch < 2 && name)
    {
        snprintf(active_cfg.pwm_name[ch], sizeof(active_cfg.pwm_name[ch]), "%s", name);
        flash_eeprom_mark_dirty();
    }
}
