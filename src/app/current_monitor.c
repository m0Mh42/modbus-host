/**
 * @file current_monitor.c
 * @brief Heater-current monitoring over the 8 CT analog inputs (True RMS).
 *
 * Accounts for configurable DC voltage bias on the CT return terminal (0..3300 mV),
 * samples across a mains cycle, subtracts the DC bias, and computes True RMS current in mA.
 */
#include "current_monitor.h"
#include "app_config.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "hal_time.h"
#include "board_config.h"
#include "flash_eeprom.h"
#include <math.h>

#define CT_ADC_FULL_SCALE 4095.0f
#define CT_ADC_VREF_MV 3300.0f
#define CT_RMS_SAMPLES 32
#define CT_SAMPLE_INTERVAL_US 625     /* 32 * 625 us = 20.0 ms (1 full 50Hz cycle) */
#define CT_NOISE_FLOOR_COUNTS 3.5f    /* ADC count noise floor threshold */
#define CT_OVERCURRENT_LIMIT_MA 32000 /* 32.0 A limit */

typedef struct
{
    uint32_t port;
    uint32_t pin;
    uint8_t adc_channel;
} ct_pin_cfg_t;

static const ct_pin_cfg_t ct_cfg[NUM_CURRENT_CHANNELS] = {
    {CT_ADC_1_PORT, CT_ADC_1_PIN, CT_ADC_1_CHANNEL},
    {CT_ADC_2_PORT, CT_ADC_2_PIN, CT_ADC_2_CHANNEL},
    {CT_ADC_3_PORT, CT_ADC_3_PIN, CT_ADC_3_CHANNEL},
    {CT_ADC_4_PORT, CT_ADC_4_PIN, CT_ADC_4_CHANNEL},
    {CT_ADC_5_PORT, CT_ADC_5_PIN, CT_ADC_5_CHANNEL},
    {CT_ADC_6_PORT, CT_ADC_6_PIN, CT_ADC_6_CHANNEL},
    {CT_ADC_7_PORT, CT_ADC_7_PIN, CT_ADC_7_CHANNEL},
    {CT_ADC_8_PORT, CT_ADC_8_PIN, CT_ADC_8_CHANNEL},
};

static uint16_t current_ma[NUM_CURRENT_CHANNELS];
static uint16_t ct_bias_mv = 2495;
static uint16_t ct_turns_ratio = 2375;
static uint16_t ct_burden_ohm_x10 = 620; /* 62.0 ohm */
static uint16_t ct_enable_mask = 0x00FF; /* All enabled by default */
static uint16_t ct_faults = 0;

static inline void delay_us(uint32_t us)
{
#ifndef UNIT_TEST
    volatile uint32_t count = us * 30;
    while (count--)
    {
        __NOP();
    }
#else
    (void)us;
#endif
}

void current_monitor_init(void)
{
    /* Load persisted calibration parameters if available */
    const nv_config_t *cfg = flash_eeprom_get();
    if (cfg->magic == NV_CONFIG_MAGIC)
    {
        ct_bias_mv = cfg->ct_bias_mv;
        ct_turns_ratio = cfg->ct_turns_ratio;
        ct_burden_ohm_x10 = cfg->ct_burden_ohm_x10;
        ct_enable_mask = cfg->ct_enable_mask;
    }

    for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++)
    {
        hal_gpio_init(ct_cfg[i].port, ct_cfg[i].pin, HAL_GPIO_ANALOG);
    }
}

void current_monitor_task(void)
{
    /* If no channels enabled, nothing to sample */
    if ((ct_enable_mask & 0x00FF) == 0)
    {
        for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++)
        {
            current_ma[i] = 0;
        }
        return;
    }

    /* Expected bias in raw ADC counts */
    float bias_counts = ((float)ct_bias_mv / CT_ADC_VREF_MV) * CT_ADC_FULL_SCALE;

    /* Accumulators for sum of squared deviations and DC average */
    float sum_sq_diff[NUM_CURRENT_CHANNELS] = {0};
    uint32_t sum_raw[NUM_CURRENT_CHANNELS] = {0};
    bool saturated[NUM_CURRENT_CHANNELS] = {false};

    /* Sample across one full mains cycle */
    for (uint8_t s = 0; s < CT_RMS_SAMPLES; s++)
    {
        for (uint8_t ch = 0; ch < NUM_CURRENT_CHANNELS; ch++)
        {
            if (!(ct_enable_mask & (1U << ch)))
            {
                continue;
            }

            uint16_t raw = hal_adc_read(ct_cfg[ch].adc_channel);
            sum_raw[ch] += raw;

            if (raw == 0 || raw >= 4095)
            {
                saturated[ch] = true;
            }

            float diff = (float)raw - bias_counts;
            sum_sq_diff[ch] += (diff * diff);
        }

        delay_us(CT_SAMPLE_INTERVAL_US);
    }

    float burden_ohm = (float)ct_burden_ohm_x10 / 10.0f;
    if (burden_ohm < 1.0f)
        burden_ohm = 62.0f; /* Safety guard against div by zero */

    for (uint8_t ch = 0; ch < NUM_CURRENT_CHANNELS; ch++)
    {
        if (!(ct_enable_mask & (1U << ch)))
        {
            current_ma[ch] = 0;
            continue;
        }

        /* RMS ADC counts */
        float ms = sum_sq_diff[ch] / (float)CT_RMS_SAMPLES;
        float rms_counts = sqrtf(ms);

        /* Average DC count for bias health check */
        float avg_dc = (float)sum_raw[ch] / (float)CT_RMS_SAMPLES;
        float avg_mv = (avg_dc / CT_ADC_FULL_SCALE) * CT_ADC_VREF_MV;

        /* Check bias health if DC offset is configured (> 0 mV, allow +/-800 mV tolerance) */
        if (ct_bias_mv > 0 && ct_bias_mv <= 3300)
        {
            if (fabsf(avg_mv - (float)ct_bias_mv) > 800.0f)
            {
                ct_faults |= (1U << 8); /* Bias reference failure */
            }
        }

        /* Suppress noise floor when idling */
        if (rms_counts < CT_NOISE_FLOOR_COUNTS)
        {
            current_ma[ch] = 0;
            continue;
        }

        /* Convert RMS counts to RMS millivolts */
        float v_rms_mv = (rms_counts / CT_ADC_FULL_SCALE) * CT_ADC_VREF_MV;

        /* Secondary current: I_sec = V_burden / R_burden */
        float i_sec_ma = v_rms_mv / burden_ohm;

        /* Primary current: I_pri = I_sec * turns_ratio */
        float i_pri_ma = i_sec_ma * (float)ct_turns_ratio;

        if (i_pri_ma > 65535.0f)
        {
            current_ma[ch] = 65535U;
        }
        else
        {
            current_ma[ch] = (uint16_t)i_pri_ma;
        }

        /* Overcurrent / saturation check */
        if (current_ma[ch] > CT_OVERCURRENT_LIMIT_MA || saturated[ch])
        {
            ct_faults |= (1U << ch);
        }
    }
}

uint16_t current_monitor_get(uint8_t channel)
{
    if (channel >= NUM_CURRENT_CHANNELS)
        return 0;
    if (!(ct_enable_mask & (1U << channel)))
        return 0;
    return current_ma[channel];
}

void current_monitor_set_bias_mv(uint16_t bias_mv)
{
    if (bias_mv > 3300)
    {
        bias_mv = 3300;
    }
    ct_bias_mv = bias_mv;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->ct_bias_mv = bias_mv;
    flash_eeprom_mark_dirty();
}

uint16_t current_monitor_get_bias_mv(void)
{
    return ct_bias_mv;
}

void current_monitor_set_turns_ratio(uint16_t ratio)
{
    if (ratio == 0)
        ratio = 2375;
    ct_turns_ratio = ratio;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->ct_turns_ratio = ratio;
    flash_eeprom_mark_dirty();
}

uint16_t current_monitor_get_turns_ratio(void)
{
    return ct_turns_ratio;
}

void current_monitor_set_burden_ohm_x10(uint16_t burden_x10)
{
    if (burden_x10 == 0)
        burden_x10 = 620;
    ct_burden_ohm_x10 = burden_x10;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->ct_burden_ohm_x10 = burden_x10;
    flash_eeprom_mark_dirty();
}

uint16_t current_monitor_get_burden_ohm_x10(void)
{
    return ct_burden_ohm_x10;
}

void current_monitor_set_enable_mask(uint16_t mask)
{
    ct_enable_mask = mask & 0x00FF;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->ct_enable_mask = ct_enable_mask;
    flash_eeprom_mark_dirty();
}

uint16_t current_monitor_get_enable_mask(void)
{
    return ct_enable_mask;
}

uint16_t current_monitor_get_faults(void)
{
    return ct_faults;
}

void current_monitor_clear_faults(void)
{
    ct_faults = 0;
}
