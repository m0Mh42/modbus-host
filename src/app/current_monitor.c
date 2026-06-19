/**
 * @file current_monitor.c
 * @brief Heater-current monitoring over the 8 CT analog inputs.
 *
 * Each current transformer feeds a burden + conditioning network into one
 * on-chip ADC channel. This sampler reads each channel once per task call and
 * converts the raw code to milliamps using a per-board scale factor.
 *
 * The conversion is a PLACEHOLDER linear map - replace CT_SCALE_MA_PER_COUNT
 * (and add RMS sampling if your conditioning is not already rectified/filtered)
 * to match the CT ratio + burden resistor in the schematic.
 */
#include "current_monitor.h"
#include "app_config.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "board_config.h"

/* mA per ADC count. TODO calibrate from CT ratio and burden resistor. */
#define CT_SCALE_MA_PER_COUNT   10.0f

static const uint8_t ct_adc_channel[NUM_CURRENT_CHANNELS] = {
    CT_ADC_CH_1, CT_ADC_CH_2, CT_ADC_CH_3, CT_ADC_CH_4,
    CT_ADC_CH_5, CT_ADC_CH_6, CT_ADC_CH_7, CT_ADC_CH_8,
};

static uint16_t current_ma[NUM_CURRENT_CHANNELS];

void current_monitor_init(void)
{
    /* ADC peripheral is brought up once in app_init via hal_adc_init();
     * here we just set the 8 CT pins to analog mode.
     * NOTE: the CT pins span the analog-capable port - confirm CT_ADC_PORT
     * and per-pin mapping in board_config.h. */
    for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++) {
        /* If each CT channel maps to a different pin, init them individually.
         * Using the channel number as the pin index is a placeholder. */
        hal_gpio_init(CT_ADC_PORT, (1u << ct_adc_channel[i]), HAL_GPIO_ANALOG);
    }
}

void current_monitor_task(void)
{
    for (uint8_t i = 0; i < NUM_CURRENT_CHANNELS; i++) {
        uint16_t raw = hal_adc_read(ct_adc_channel[i]);
        float ma = (float)raw * CT_SCALE_MA_PER_COUNT;
        current_ma[i] = (ma > 65535.0f) ? 65535u : (uint16_t)ma;
    }
}

uint16_t current_monitor_get(uint8_t channel)
{
    if (channel >= NUM_CURRENT_CHANNELS) return 0;
    return current_ma[channel];
}
