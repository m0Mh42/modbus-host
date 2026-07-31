/**
 * @file thermocouple.c
 * @brief 8-channel thermocouple service using CS1237.
 */
#include "thermocouple.h"
#include "app_config.h"
#include "ads1220.h" /* Kept per user request, but unused. */
#include "cs1237.h"
#include "board_config.h"
#include <math.h>

static int16_t   temp_c10[NUM_THERMOCOUPLES];
static bool      temp_valid[NUM_THERMOCOUPLES];
static int16_t   dc_offset_uv[NUM_THERMOCOUPLES];

/* 3.3V VREF, Gain 128 */
#define CS1237_VREF 3.3f
#define TC_GAIN_F   128.0f
#define TC_OPEN_THRESHOLD 8220000L

static float typeK_tempToVoltage_mV(float t) {
    static const float c[] = {-0.176004136860e-1, 0.389212049750e-1,
                              0.185587700320e-4,  -0.994575928740e-7,
                              0.318409457190e-9,  -0.560728448890e-12,
                              0.560750590590e-15, -0.320207200030e-18,
                              0.971511471520e-22, -0.121047212750e-25};
    const float a0 = 0.118597600000e0;
    const float a1 = -0.118343200000e-3;
    const float a2 = 0.126968600000e3;
    float e = 0.0f, tp = 1.0f;
    for (uint8_t i = 0; i < 10; i++) {
        e += c[i] * tp;
        tp *= t;
    }
    e += a0 * expf(a1 * (t - a2) * (t - a2));
    return e;
}

static float typeK_voltageToTemp_C(float mv) {
    float t = 0.0f, v = 1.0f;
    if (mv < 0.0f) {
        static const float d[] = {0.0f, 2.5173462e1, -1.1662878e0,
                                  -1.0833638e0, -8.9773540e-1, -3.7342377e-1,
                                  -8.6632643e-2, -1.0450598e-2, -5.1920577e-4};
        for (uint8_t i = 0; i < 9; i++) { t += d[i] * v; v *= mv; }
    } else if (mv < 20.644f) {
        static const float d[] = {0.0f, 2.508355e1, 7.860106e-2, -2.503131e-1,
                                  8.315270e-2, -1.228034e-2, 9.804036e-4, 
                                  -4.413030e-5, 1.057734e-6, -1.052755e-8};
        for (uint8_t i = 0; i < 10; i++) { t += d[i] * v; v *= mv; }
    } else {
        static const float d[] = {-1.318058e2, 4.830222e1, -1.646031e0,
                                  5.464731e-2, -9.650715e-4, 8.802193e-6,
                                  -3.110810e-8};
        for (uint8_t i = 0; i < 7; i++) { t += d[i] * v; v *= mv; }
    }
    return t;
}

void thermocouple_init(void)
{
    cs1237_init();
    
    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
        dc_offset_uv[i] = 0;
        temp_c10[i] = 0;
        temp_valid[i] = false;
        
        cs1237_configure_one(i, CS1237_PGA_128, CS1237_SPEED_40, CS1237_CHANNEL_A | CS1237_REF_EXTERNAL);
    }
}

void thermocouple_task(void)
{
    int32_t raw[8];
    if (!cs1237_scan_all(raw)) {
        return; /* timeout, stay non-blocking if possible (scan_all blocks up to timeout though) */
    }

    float cjTemp = 25.0f; /* TODO: Read real CJC from a DS18B20 or similar */
    float vCold_mV = typeK_tempToVoltage_mV(cjTemp);

    for (uint8_t i = 0; i < NUM_THERMOCOUPLES; i++) {
        if (raw[i] > TC_OPEN_THRESHOLD || raw[i] < -TC_OPEN_THRESHOLD) {
            temp_valid[i] = false;
            continue;
        }
        
        float uV = (float)raw[i] * (0.5f * CS1237_VREF / TC_GAIN_F) / 8388608.0f * 1e6f;
        uV += dc_offset_uv[i];
        
        float vMeasured_mV = uV / 1000.0f;
        float t = typeK_voltageToTemp_C(vMeasured_mV + vCold_mV);
        
        temp_c10[i] = (int16_t)(t * 10.0f);
        temp_valid[i] = true;
    }
}

int16_t thermocouple_get_temp(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES) return 0;
    return temp_c10[channel];
}

bool thermocouple_is_valid(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES) return false;
    return temp_valid[channel];
}

int16_t thermocouple_get_offset(uint8_t channel)
{
    if (channel >= NUM_THERMOCOUPLES) return 0;
    return dc_offset_uv[channel];
}

void thermocouple_set_offset(uint8_t channel, int16_t offset_uv)
{
    if (channel >= NUM_THERMOCOUPLES) return;
    dc_offset_uv[channel] = offset_uv;
}
