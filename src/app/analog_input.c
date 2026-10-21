/**
 * @file analog_input.c
 * @brief 2 divided analog inputs -> millivolts, exposed to Modbus.
 *
 * v_pin   = raw / FULL_SCALE * VREF_mV
 * v_input = v_pin * divider_ratio        (undo the resistor divider)
 */
#include "analog_input.h"
#include "app_config.h"
#include "hal_adc.h"
#include "hal_gpio.h"
#include "board_config.h"
#include "flash_eeprom.h"

#define AIN_OVERVOLTAGE_MV 12000u /* 12.0 V maximum on 10V input */

typedef struct
{
    uint32_t port;
    uint32_t pin;
    uint8_t adc_channel;
} ain_hw_cfg_t;

static const ain_hw_cfg_t ain_cfg[AIN_COUNT] = {
    {AIN1_PORT, AIN1_PIN, AIN1_ADC_CHANNEL},
    {AIN2_PORT, AIN2_PIN, AIN2_ADC_CHANNEL},
};

static uint16_t ain_raw[AIN_COUNT];
static uint16_t ain_mv[AIN_COUNT];
static uint16_t ain_divider_x1000[AIN_COUNT] = {4917, 4917};
static uint16_t ain_enable_mask = 0x0003;
static uint16_t ain_faults = 0;

void analog_input_init(void)
{
    const nv_config_t *cfg = flash_eeprom_get();
    if (cfg->magic == NV_CONFIG_MAGIC)
    {
        ain_divider_x1000[0] = cfg->ain1_divider_x1000;
        ain_divider_x1000[1] = cfg->ain2_divider_x1000;
        ain_enable_mask = cfg->ain_enable_mask;
    }

    for (uint8_t i = 0; i < AIN_COUNT; i++)
    {
        hal_gpio_init(ain_cfg[i].port, ain_cfg[i].pin, HAL_GPIO_ANALOG);
    }
}

void analog_input_task(void)
{
    for (uint8_t i = 0; i < AIN_COUNT; i++)
    {
        if (!(ain_enable_mask & (1U << i)))
        {
            ain_raw[i] = 0;
            ain_mv[i] = 0;
            continue;
        }

        uint16_t raw = hal_adc_read(ain_cfg[i].adc_channel);
        ain_raw[i] = raw;

        /* Voltage at the ADC pin, then apply configurable divider ratio */
        float v_pin = ((float)raw / (float)AIN_ADC_FULL_SCALE) * (float)AIN_VREF_MV;
        float ratio = (float)ain_divider_x1000[i] / 1000.0f;
        float v_input = v_pin * ratio;

        uint32_t mv = (uint32_t)(v_input + 0.5f);
        ain_mv[i] = (mv > 65535u) ? 65535u : (uint16_t)mv;

        if (ain_mv[i] > AIN_OVERVOLTAGE_MV)
        {
            ain_faults |= (1U << i);
        }
    }
}

uint16_t analog_input_get_mv(uint8_t channel)
{
    if (channel >= AIN_COUNT)
        return 0;
    if (!(ain_enable_mask & (1U << channel)))
        return 0;
    return ain_mv[channel];
}

uint16_t analog_input_get_raw(uint8_t channel)
{
    if (channel >= AIN_COUNT)
        return 0;
    if (!(ain_enable_mask & (1U << channel)))
        return 0;
    return ain_raw[channel];
}

void analog_input_set_divider_ratio_x1000(uint8_t channel, uint16_t ratio_x1000)
{
    if (channel >= AIN_COUNT)
        return;
    if (ratio_x1000 < 100)
        ratio_x1000 = 4917; /* Safety floor 0.1x */

    ain_divider_x1000[channel] = ratio_x1000;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    if (channel == 0)
    {
        cfg->ain1_divider_x1000 = ratio_x1000;
    }
    else
    {
        cfg->ain2_divider_x1000 = ratio_x1000;
    }
    flash_eeprom_mark_dirty();
}

uint16_t analog_input_get_divider_ratio_x1000(uint8_t channel)
{
    if (channel >= AIN_COUNT)
        return 4917;
    return ain_divider_x1000[channel];
}

void analog_input_set_enable_mask(uint16_t mask)
{
    ain_enable_mask = mask & 0x0003;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->ain_enable_mask = ain_enable_mask;
    flash_eeprom_mark_dirty();
}

uint16_t analog_input_get_enable_mask(void)
{
    return ain_enable_mask;
}

uint16_t analog_input_get_faults(void)
{
    return ain_faults;
}

void analog_input_clear_faults(void)
{
    ain_faults = 0;
}
