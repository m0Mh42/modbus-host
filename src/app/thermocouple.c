/**
 * @file thermocouple.c
 * @brief 8-channel thermocouple service over two ADS1220 devices.
 *
 * Strategy (single-shot, non-blocking round-robin):
 *   - Each task call checks whether the currently-selected channel's device
 *     has data ready. If so it reads the raw code, converts to temperature,
 *     advances to the next channel, points the mux, and starts a conversion.
 *   - Channels 0..3 live on ads[0], 4..7 on ads[1].
 *
 * The raw->temperature conversion here is a PLACEHOLDER linear map. Replace
 * convert_raw_to_c10() with: ADC code -> microvolts -> thermocouple lookup
 * (e.g. type-K polynomial) -> cold-junction compensation.
 */
#include "thermocouple.h"
#include "app_config.h"
#include "ads1220.h"
#include "board_config.h"

static ads1220_t ads[2];
static int16_t   temp_c10[NUM_THERMOCOUPLES];
static bool      temp_valid[NUM_THERMOCOUPLES];
static uint8_t   scan_idx;     /* 0..7, the channel currently converting */

/* Map a global thermocouple channel (0..7) to its device + local channel. */
static ads1220_t *dev_of(uint8_t ch) { return &ads[ch / 4]; }
static uint8_t    local_of(uint8_t ch) { return ch % 4; }

/* PLACEHOLDER conversion: ADS1220 full scale (2.048 V ref, gain 1) is
 * +/-2^23 codes. Scale to a temperature in 0.1 C. Replace with real math. */
static int16_t convert_raw_to_c10(int32_t raw)
{
    /* Example: treat code as microvolts (ref/2^23 ~= 0.244 uV/code) then a
     * naive 41 uV/C type-K slope. This is only to give a non-zero number. */
    float microvolts = (float)raw * (2048000.0f / 8388608.0f); /* uV */
    float celsius    = microvolts / 41.0f;                     /* ~type-K */
    return (int16_t)(celsius * 10.0f);
}

static void start_channel(uint8_t ch)
{
    ads1220_select_channel(dev_of(ch), local_of(ch));
    ads1220_start(dev_of(ch));
}

void thermocouple_init(void)
{
    /* Device 0 = thermocouples 0..3, device 1 = 4..7. */
    ads[0].cs_port   = ADS1_CS_PORT;   ads[0].cs_pin   = ADS1_CS_PIN;
    ads[0].drdy_port = ADS1_DRDY_PORT; ads[0].drdy_pin = ADS1_DRDY_PIN;
    ads[1].cs_port   = ADS2_CS_PORT;   ads[1].cs_pin   = ADS2_CS_PIN;
    ads[1].drdy_port = ADS2_DRDY_PORT; ads[1].drdy_pin = ADS2_DRDY_PIN;

    for (int i = 0; i < 2; i++) {
        bool ok = ads1220_init(&ads[i]);
        /* Mark this device's channels invalid until the first good read. */
        for (int c = 0; c < 4; c++) temp_valid[i * 4 + c] = ok ? false : false;
    }

    scan_idx = 0;
    start_channel(scan_idx);
}

void thermocouple_task(void)
{
    /* Only the active channel's device is converting; poll its DRDY. */
    if (!ads1220_data_ready(dev_of(scan_idx))) {
        return;     /* not ready yet - stay non-blocking */
    }

    int32_t raw = ads1220_read_data(dev_of(scan_idx));
    temp_c10[scan_idx]   = convert_raw_to_c10(raw);
    temp_valid[scan_idx] = true;   /* TODO: add open-thermocouple detection */

    /* Advance round-robin and kick off the next conversion. */
    scan_idx = (scan_idx + 1) % NUM_THERMOCOUPLES;
    start_channel(scan_idx);
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
