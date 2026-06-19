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

typedef struct {
    uint32_t port;
    uint32_t pin;
    uint8_t  adc_channel;
    float    divider_ratio;
} ain_cfg_t;

/* TODO confirm port/pin for AIN1/AIN2 against the schematic. */
static const ain_cfg_t ain_cfg[AIN_COUNT] = {
    { GPIOB, (1u << 0), AIN1_ADC_CHANNEL, AIN1_DIVIDER_RATIO },
    { GPIOB, (1u << 1), AIN2_ADC_CHANNEL, AIN2_DIVIDER_RATIO },
};

static uint16_t ain_raw[AIN_COUNT];
static uint16_t ain_mv[AIN_COUNT];

void analog_input_init(void)
{
    for (uint8_t i = 0; i < AIN_COUNT; i++) {
        hal_gpio_init(ain_cfg[i].port, ain_cfg[i].pin, HAL_GPIO_ANALOG);
    }
}

void analog_input_task(void)
{
    for (uint8_t i = 0; i < AIN_COUNT; i++) {
        uint16_t raw = hal_adc_read(ain_cfg[i].adc_channel);
        ain_raw[i] = raw;

        /* Voltage at the ADC pin, then undo the divider to get the real input. */
        float v_pin   = ((float)raw / (float)AIN_ADC_FULL_SCALE) * (float)AIN_VREF_MV;
        float v_input = v_pin * ain_cfg[i].divider_ratio;
        ain_mv[i] = (v_input > 65535.0f) ? 65535u : (uint16_t)v_input;
    }
}

uint16_t analog_input_get_mv(uint8_t channel)
{
    if (channel >= AIN_COUNT) return 0;
    return ain_mv[channel];
}

uint16_t analog_input_get_raw(uint8_t channel)
{
    if (channel >= AIN_COUNT) return 0;
    return ain_raw[channel];
}
