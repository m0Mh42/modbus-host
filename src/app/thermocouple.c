/**
 * @file thermocouple.c
 * @brief 8-channel thermocouple service using CS1237 with shared-SCLK opportunistic readout.
 */
#include "thermocouple.h"
#include "app_config.h"
#include "ads1220.h"
#include "cs1237.h"
#include "ds18b20.h"
#include "modbus_rtu.h"
#include "board_config.h"
#include "hal_time.h"
#include "flash_eeprom.h"
#include <math.h>
#include <stdlib.h>

static int16_t temp_c10[NUM_THERMOCOUPLES];
static bool temp_valid[NUM_THERMOCOUPLES];
static int16_t dc_offset_uv[NUM_THERMOCOUPLES];
static uint16_t tc_enable_mask = 0x00FF;
static uint16_t tc_faults = 0;
static float current_cjc_temp = 25.0f;
static uint32_t last_ds18b20_read_ms = 0;

/* 2.5V VREF from TL431, Gain 128 */
#define CS1237_VREF 2.5f
#define TC_GAIN_F 128.0f
#define TC_OPEN_THRESHOLD 8220000L /* ~98% of 2^23 full scale */

/* -------------------------------------------------------------------------
 * 3-Tap Median Filter for Outlier & Bus-Collision Rejection
 * ------------------------------------------------------------------------- */
static int32_t raw_history[NUM_THERMOCOUPLES][3];
static uint8_t history_count[NUM_THERMOCOUPLES];

static int32_t median3(int32_t a, int32_t b, int32_t c)
{
    if ((a <= b && b <= c) || (c <= b && b <= a))
        return b;
    if ((b <= a && a <= c) || (c <= a && a <= b))
        return a;
    return c;
}

/* -------------------------------------------------------------------------
 * NIST ITS-90 Type-K Thermocouple Polynomials
 * Valid from -200 C to 1372 C (-5.891 mV to 54.886 mV)
 * ------------------------------------------------------------------------- */

/* Forward: temperature (C) -> Type-K voltage (mV) */
static float typeK_tempToVoltage_mV(float t)
{
    static const float c[] = {
        -0.176004136860e-1f, 0.389212049750e-1f,
        0.185587700320e-4f, -0.994575928740e-7f,
        0.318409457190e-9f, -0.560728448890e-12f,
        0.560750590590e-15f, -0.320207200030e-18f,
        0.971511471520e-22f, -0.121047212750e-25f};
    const float a0 = 0.118597600000e0f;
    const float a1 = -0.118343200000e-3f;
    const float a2 = 0.126968600000e3f;
    float e = 0.0f, tp = 1.0f;
    for (uint8_t i = 0; i < 10; i++)
    {
        e += c[i] * tp;
        tp *= t;
    }
    e += a0 * expf(a1 * (t - a2) * (t - a2));
    return e;
}

/* Inverse: total Type-K voltage (mV) -> temperature (C) */
static float typeK_voltageToTemp_C(float mv)
{
    float t = 0.0f, v = 1.0f;
    if (mv < 0.0f)
    { /* -200..0 C (-5.891..0 mV) */
        static const float d[] = {
            0.0000000e0f, 2.5173462e1f, -1.1662878e0f,
            -1.0833638e0f, -8.9773540e-1f, -3.7342377e-1f,
            -8.6632643e-2f, -1.0450598e-2f, -5.1920577e-4f};
        for (uint8_t i = 0; i < 9; i++)
        {
            t += d[i] * v;
            v *= mv;
        }
    }
    else if (mv < 20.644f)
    { /* 0..500 C */
        static const float d[] = {
            0.000000e0f, 2.508355e1f, 7.860106e-2f, -2.503131e-1f,
            8.315270e-2f, -1.228034e-2f, 9.804036e-4f, -4.413030e-5f,
            1.057734e-6f, -1.052755e-8f};
        for (uint8_t i = 0; i < 10; i++)
        {
            t += d[i] * v;
            v *= mv;
        }
    }
    else
    { /* 500..1372 C (20.644..54.886 mV) */
        static const float d[] = {
            -1.318058e2f, 4.830222e1f, -1.646031e0f,
            5.464731e-2f, -9.650715e-4f, 8.802193e-6f,
            -3.110810e-8f};
        for (uint8_t i = 0; i < 7; i++)
        {
            t += d[i] * v;
            v *= mv;
        }
    }
    return t;
}

/* -------------------------------------------------------------------------
 * Adaptive One-Euro Filter for Jitter Elimination with Fast Step Response
 * ------------------------------------------------------------------------- */
#define FILTER_MIN_CUTOFF_HZ 0.15f
#define FILTER_SLOPE_RESPONSE 0.25f
#define FILTER_DERIVATIVE_CUTOFF_HZ 0.5f

typedef struct
{
    bool initialized;
    float previousInput;
    float filteredValue;
    float filteredDerivative;
    uint32_t previousTimeMs;
} one_euro_filter_t;

static one_euro_filter_t temp_filter[NUM_THERMOCOUPLES];

static inline float one_euro_alpha(float dt, float cutoffHz)
{
    const float tau = 1.0f / (6.2831853f * cutoffHz);
    return 1.0f / (1.0f + tau / dt);
}

static float one_euro_apply(one_euro_filter_t *f, float input, uint32_t nowMs)
{
    if (!f->initialized)
    {
        f->initialized = true;
        f->previousInput = input;
        f->filteredValue = input;
        f->filteredDerivative = 0.0f;
        f->previousTimeMs = nowMs;
        return input;
    }

    float dt = (nowMs - f->previousTimeMs) * 0.001f;
    if (dt < 0.001f)
    {
        dt = 0.001f;
    }

    float derivative = (input - f->previousInput) / dt;
    float dAlpha = one_euro_alpha(dt, FILTER_DERIVATIVE_CUTOFF_HZ);
    f->filteredDerivative += dAlpha * (derivative - f->filteredDerivative);

    float cutoff = FILTER_MIN_CUTOFF_HZ + FILTER_SLOPE_RESPONSE * fabsf(f->filteredDerivative);
    float valAlpha = one_euro_alpha(dt, cutoff);
    f->filteredValue += valAlpha * (input - f->filteredValue);

    f->previousInput = input;
    f->previousTimeMs = nowMs;
    return f->filteredValue;
}

static inline void one_euro_reset(one_euro_filter_t *f)
{
    f->initialized = false;
}

void thermocouple_init(void)
{
    /* Load persisted configuration */
    const nv_config_t *cfg = flash_eeprom_get();
    if (cfg->magic == NV_CONFIG_MAGIC)
    {
        tc_enable_mask = cfg->tc_enable_mask;
        for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
        {
            dc_offset_uv[i] = cfg->pid[i].tc_offset_uv;
        }
    }

    /* Register cooperative yield callback so Modbus is serviced during CS1237 wait loops */
    cs1237_set_yield_callback(modbus_task);

    cs1237_init();
    ds18b20_init();
    ds18b20_start_conversion();

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        temp_c10[i] = 0;
        temp_valid[i] = false;
        history_count[i] = 0;
        one_euro_reset(&temp_filter[i]);
    }
}

void thermocouple_task(void)
{
    int32_t raw[CS1237_CHANNELS];
    bool valid[CS1237_CHANNELS];

    if ((tc_enable_mask & 0x00FF) == 0)
    {
        for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
        {
            temp_c10[i] = 0;
            temp_valid[i] = false;
        }
        return;
    }

    /* Sequential wait-and-read across all 8 channels with generous per-channel timeout (250 ms) */
    uint8_t read_count = cs1237_scan_all(raw, valid, TC_SCAN_TIMEOUT_MS);
    if (read_count == 0)
    {
        /* All channels failed to respond */
        for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
        {
            if (tc_enable_mask & (1U << i))
            {
                tc_faults |= (1U << i);
            }
        }
        return;
    }

    uint32_t now = hal_millis();

    /* Poll DS18B20 cold junction asynchronously every ~1 second */
    if (now - last_ds18b20_read_ms >= 1000)
    {
        float temp;
        if (ds18b20_read_temp(&temp))
        {
            current_cjc_temp = temp;
            /* CJC temp valid range -40..85 C */
            if (temp < -40.0f || temp > 85.0f)
            {
                tc_faults |= (1U << 8);
            }
        }
        else
        {
            tc_faults |= (1U << 8); /* DS18B20 CJC failure */
        }
        ds18b20_start_conversion();
        last_ds18b20_read_ms = now;
    }

    float vCold_mV = typeK_tempToVoltage_mV(current_cjc_temp);

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++)
    {
        if (!(tc_enable_mask & (1U << i)))
        {
            temp_c10[i] = 0;
            temp_valid[i] = false;
            continue;
        }

        if (!valid[i])
        {
            temp_valid[i] = false;
            tc_faults |= (1U << i);
            continue;
        }

        /* Check raw limits for open thermocouple before filtering */
        if (cs1237_is_open(raw[i]))
        {
            temp_valid[i] = false;
            one_euro_reset(&temp_filter[i]);
            history_count[i] = 0;
            tc_faults |= (1U << i);
            continue;
        }

        /* 3-tap median filter to eliminate any outliers */
        int32_t raw_median;
        if (history_count[i] < 3)
        {
            raw_history[i][history_count[i]++] = raw[i];
            raw_median = raw[i];
        }
        else
        {
            raw_history[i][0] = raw_history[i][1];
            raw_history[i][1] = raw_history[i][2];
            raw_history[i][2] = raw[i];
            raw_median = median3(raw_history[i][0], raw_history[i][1], raw_history[i][2]);
        }

        /* Secondary check after median filtering */
        if (cs1237_is_open(raw_median))
        {
            temp_valid[i] = false;
            one_euro_reset(&temp_filter[i]);
            tc_faults |= (1U << i);
            continue;
        }

        /* Convert raw ADC code to microvolts at input */
        float uV = cs1237_raw_to_microvolts(raw_median) + (float)dc_offset_uv[i];

        /* Cold junction compensation: V_tot = V_tc + V_cjc */
        float vMeasured_mV = uV / 1000.0f;
        float vTotal_mV = vMeasured_mV + vCold_mV;

        /* ITS-90 inverse polynomial to temperature */
        float t = typeK_voltageToTemp_C(vTotal_mV);

        /* Adaptive One-Euro filter for jitter-free readings */
        t = one_euro_apply(&temp_filter[i], t, now);

        temp_c10[i] = (int16_t)(t * 10.0f);
        temp_valid[i] = true;
    }
}

int16_t thermocouple_get_temp(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES)
        return 0;
    if (!(tc_enable_mask & (1U << channel)))
        return 0;
    return temp_c10[channel];
}

bool thermocouple_is_valid(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES)
        return false;
    if (!(tc_enable_mask & (1U << channel)))
        return false;
    return temp_valid[channel];
}

int16_t thermocouple_get_cjc_temp_c10(void)
{
    return (int16_t)(current_cjc_temp * 10.0f);
}

int16_t thermocouple_get_offset(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES)
        return 0;
    return dc_offset_uv[channel];
}

void thermocouple_set_offset(uint8_t channel, int16_t offset_uv)
{
    if (channel >= NUM_THERMOCOUPLES)
        return;
    dc_offset_uv[channel] = offset_uv;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->pid[channel].tc_offset_uv = offset_uv;
    flash_eeprom_mark_dirty();
}

void thermocouple_set_enable_mask(uint16_t mask)
{
    tc_enable_mask = mask & 0x00FF;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->tc_enable_mask = tc_enable_mask;
    flash_eeprom_mark_dirty();
}

uint16_t thermocouple_get_enable_mask(void)
{
    return tc_enable_mask;
}

uint16_t thermocouple_get_faults(void)
{
    return tc_faults;
}

void thermocouple_clear_faults(void)
{
    tc_faults = 0;
}
