/**
 * @file proximity.c
 * @brief Proximity-sensor input implementation.
 */
#include "proximity.h"
#include "board_config.h"
#include "app_config.h"
#include "hal_gpio.h"
#include "hal_time.h"
#include "flash_eeprom.h"

#define PROX_SPEED_TIMEOUT_MS 2000 /* zero the speed after this idle gap */
#define PROX_MAX_SPEED_HZ 50000u   /* 50 kHz overspeed threshold */

static volatile uint32_t prox_count[NUM_PROXIMITY_INPUTS];
static volatile uint32_t prox_speed[NUM_PROXIMITY_INPUTS];
static volatile uint32_t prox_last_ms[NUM_PROXIMITY_INPUTS];
static uint16_t prox_enable_mask = 0x0003;
static uint16_t prox_faults = 0;
static uint16_t prox_speed_src = 0;           /* 0=PROX1, 1=PROX2, 2=BOTH */
static uint16_t prox_speed_setpoint = 0;       /* Set speed input (0.1 m/min) */
static uint16_t prox_pulses_per_meter = 100;   /* Pulses per meter (default 100) */

void proximity_init(void)
{
    const nv_config_t *cfg = flash_eeprom_get();
    if (cfg->magic == NV_CONFIG_MAGIC)
    {
        prox_enable_mask = cfg->prox_enable_mask;
        prox_speed_src = (cfg->speed_prox_src <= 2) ? cfg->speed_prox_src : 0;
        prox_speed_setpoint = cfg->speed_setpoint;
        prox_pulses_per_meter = (cfg->pulses_per_meter > 0) ? cfg->pulses_per_meter : 100;
    }

    hal_gpio_init(PROX1_PORT, PROX1_PIN, HAL_GPIO_INPUT_PULLUP);
    hal_gpio_init(PROX2_PORT, PROX2_PIN, HAL_GPIO_INPUT_PULLUP);

    for (int i = 0; i < NUM_PROXIMITY_INPUTS; i++)
    {
        prox_count[i] = prox_speed[i] = prox_last_ms[i] = 0;
    }

    /* Configure EXTI lines for COUNTER_1 (PA0) and COUNTER_2 (PA1) (falling edge per optocoupler output) */
#ifndef UNIT_TEST
    rcu_periph_clock_enable(RCU_AF);

    gpio_exti_source_select(PROX1_PORT_SOURCE, PROX1_PIN_SOURCE);
    gpio_exti_source_select(PROX2_PORT_SOURCE, PROX2_PIN_SOURCE);

    exti_init(PROX1_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_init(PROX2_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);

    nvic_irq_enable(PROX1_IRQn, 2, 0);
    nvic_irq_enable(PROX2_IRQn, 2, 0);
#endif
}

void EXTI0_IRQHandler(void)
{
#ifndef UNIT_TEST
    if (exti_interrupt_flag_get(PROX1_EXTI_LINE) != RESET)
    {
        exti_interrupt_flag_clear(PROX1_EXTI_LINE);
        proximity_on_edge(PROX_CHANNEL_1);
    }
#endif
}

void EXTI1_IRQHandler(void)
{
#ifndef UNIT_TEST
    if (exti_interrupt_flag_get(PROX2_EXTI_LINE) != RESET)
    {
        exti_interrupt_flag_clear(PROX2_EXTI_LINE);
        proximity_on_edge(PROX_CHANNEL_2);
    }
#endif
}

void proximity_on_edge(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS)
        return;
    if (!(prox_enable_mask & (1U << ch)))
        return;

    uint32_t now = hal_millis();
    uint32_t dt = now - prox_last_ms[ch];

    prox_count[ch]++;

    /* Instantaneous speed from the inter-pulse interval. */
    if (dt > 0)
    {
        prox_speed[ch] = 1000u / dt; /* pulses per second */
        if (prox_speed[ch] > PROX_MAX_SPEED_HZ)
        {
            prox_faults |= (1U << ch);
        }
    }
    prox_last_ms[ch] = now;
}

void proximity_task(void)
{
    /* Decay speed to zero if no pulses arrive for a while (object stopped). */
    uint32_t now = hal_millis();
    for (int i = 0; i < NUM_PROXIMITY_INPUTS; i++)
    {
        if (!(prox_enable_mask & (1U << i)))
        {
            prox_speed[i] = 0;
            continue;
        }

        if ((now - prox_last_ms[i]) > PROX_SPEED_TIMEOUT_MS)
        {
            prox_speed[i] = 0;
        }
    }
}

uint32_t proximity_get_count(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS)
        return 0;
    if (!(prox_enable_mask & (1U << ch)))
        return 0;
    return prox_count[ch];
}

uint32_t proximity_get_speed(prox_channel_t ch)
{
    if (ch >= NUM_PROXIMITY_INPUTS)
        return 0;
    if (!(prox_enable_mask & (1U << ch)))
        return 0;
    return prox_speed[ch];
}

void proximity_reset_count(prox_channel_t ch)
{
    if (ch < NUM_PROXIMITY_INPUTS)
        prox_count[ch] = 0;
}

static uint32_t proximity_get_meter_pulses(void)
{
    uint32_t c1 = proximity_get_count(PROX_CHANNEL_1);
    uint32_t c2 = proximity_get_count(PROX_CHANNEL_2);

    if (prox_speed_src == 0)
    {
        return c1;
    }
    else if (prox_speed_src == 1)
    {
        return c2;
    }
    else
    {
        return (c1 + c2) / 2;
    }
}

uint32_t proximity_get_meterage_c10(void)
{
    uint32_t pulses = proximity_get_meter_pulses();
    uint16_t ppm = (prox_pulses_per_meter > 0) ? prox_pulses_per_meter : 100;
    return (uint32_t)(((uint64_t)pulses * 10ULL) / (uint64_t)ppm);
}

uint32_t proximity_get_meterage_m(void)
{
    uint32_t pulses = proximity_get_meter_pulses();
    uint16_t ppm = (prox_pulses_per_meter > 0) ? prox_pulses_per_meter : 100;
    return pulses / (uint32_t)ppm;
}

void proximity_reset_meterage(void)
{
    prox_count[0] = 0;
    prox_count[1] = 0;
}

void proximity_set_pulses_per_meter(uint16_t ppm)
{
    prox_pulses_per_meter = (ppm > 0) ? ppm : 100;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->pulses_per_meter = prox_pulses_per_meter;
    flash_eeprom_mark_dirty();
}

uint16_t proximity_get_pulses_per_meter(void)
{
    return prox_pulses_per_meter;
}

void proximity_set_enable_mask(uint16_t mask)
{
    prox_enable_mask = mask & 0x0003;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->prox_enable_mask = prox_enable_mask;
    flash_eeprom_mark_dirty();
}

uint16_t proximity_get_enable_mask(void)
{
    return prox_enable_mask;
}

uint16_t proximity_get_faults(void)
{
    return prox_faults;
}

void proximity_clear_faults(void)
{
    prox_faults = 0;
}

void proximity_set_speed_src(uint16_t src)
{
    prox_speed_src = (src > 2) ? 2 : src;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->speed_prox_src = prox_speed_src;
    flash_eeprom_mark_dirty();
}

uint16_t proximity_get_speed_src(void)
{
    return prox_speed_src;
}

void proximity_set_speed_setpoint(uint16_t sp)
{
    prox_speed_setpoint = sp;
    nv_config_t *cfg = flash_eeprom_get_mutable();
    cfg->speed_setpoint = prox_speed_setpoint;
    flash_eeprom_mark_dirty();
}

uint16_t proximity_get_speed_setpoint(void)
{
    return prox_speed_setpoint;
}

uint32_t proximity_get_effective_speed_hz(void)
{
    uint32_t s1 = proximity_get_speed(PROX_CHANNEL_1);
    uint32_t s2 = proximity_get_speed(PROX_CHANNEL_2);

    if (prox_speed_src == 0)
    {
        return s1;
    }
    else if (prox_speed_src == 1)
    {
        return s2;
    }
    else
    {
        /* Both: average */
        return (s1 + s2) / 2;
    }
}

uint16_t proximity_get_line_speed_c10(void)
{
    uint32_t hz = proximity_get_effective_speed_hz();
    uint16_t ppm = (prox_pulses_per_meter > 0) ? prox_pulses_per_meter : 100;
    /* Line Speed (0.1 m/min) = ((uint64_t)hz * 600ULL) / (uint64_t)ppm */
    uint64_t val = ((uint64_t)hz * 600ULL) / (uint64_t)ppm;
    if (val > 0xFFFFu)
    {
        return 0xFFFFu;
    }
    return (uint16_t)val;
}
