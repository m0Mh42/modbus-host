/**
 * @file relay.c
 * @brief 16-channel relay bank implementation.
 *
 * Relays are driven through a ULN2803 (active-high MCU pin -> relay ON).
 * If your wiring is inverted, flip RELAY_ACTIVE_LEVEL below.
 */
#include "relay.h"
#include "board_config.h"
#include "hal_gpio.h"

#define RELAY_ACTIVE_LEVEL true /* MCU pin level that energizes the relay */

/* Per-channel pin map confirmed against docs/pins.csv (RELAY_1..16).
 * Order here is logical relay 0..15. */
typedef struct
{
    uint32_t port;
    uint32_t pin;
} relay_pin_t;

static const relay_pin_t relay_pins[RELAY_COUNT] = {
    {RELAY_1_PORT, RELAY_1_PIN},   /* Relay 1  (Pin 54: PD2)  */
    {RELAY_2_PORT, RELAY_2_PIN},   /* Relay 2  (Pin 62: PB9)  */
    {RELAY_3_PORT, RELAY_3_PIN},   /* Relay 3  (Pin 55: PB3)  - JTAG JTDO */
    {RELAY_4_PORT, RELAY_4_PIN},   /* Relay 4  (Pin 61: PB8)  */
    {RELAY_5_PORT, RELAY_5_PIN},   /* Relay 5  (Pin 56: PB4)  - JTAG NJTRST */
    {RELAY_6_PORT, RELAY_6_PIN},   /* Relay 6  (Pin 59: PB7)  */
    {RELAY_7_PORT, RELAY_7_PIN},   /* Relay 7  (Pin 57: PB5)  */
    {RELAY_8_PORT, RELAY_8_PIN},   /* Relay 8  (Pin 58: PB6)  */
    {RELAY_9_PORT, RELAY_9_PIN},   /* Relay 9  (Pin 53: PC12) */
    {RELAY_10_PORT, RELAY_10_PIN}, /* Relay 10 (Pin 45: PA12) */
    {RELAY_11_PORT, RELAY_11_PIN}, /* Relay 11 (Pin 52: PC11) */
    {RELAY_12_PORT, RELAY_12_PIN}, /* Relay 12 (Pin 44: PA11) */
    {RELAY_13_PORT, RELAY_13_PIN}, /* Relay 13 (Pin 51: PC10) */
    {RELAY_14_PORT, RELAY_14_PIN}, /* Relay 14 (Pin 43: PA10) */
    {RELAY_15_PORT, RELAY_15_PIN}, /* Relay 15 (Pin 50: PA15) - JTAG JTDI */
    {RELAY_16_PORT, RELAY_16_PIN}, /* Relay 16 (Pin 42: PA9)  */
};

static uint16_t relay_state = 0;

void relay_early_init(void)
{
#ifndef UNIT_TEST
    /* Release PA15, PB3, PB4 from JTAG as early as possible so they can be
     * safely driven as standard GPIOs for RELAY_15, RELAY_3, and RELAY_5,
     * while retaining SWD (PA13/PA14) for programming and debugging. */
    rcu_periph_clock_enable(RCU_AF);
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
#endif

    for (uint8_t i = 0; i < RELAY_COUNT; i++)
    {
        hal_gpio_init(relay_pins[i].port, relay_pins[i].pin, HAL_GPIO_OUTPUT);
        hal_gpio_write(relay_pins[i].port, relay_pins[i].pin, !RELAY_ACTIVE_LEVEL);
    }
    relay_state = 0;
}

void relay_init(void)
{
    relay_early_init();
}

void relay_set(uint8_t channel, bool on)
{
    if (channel >= RELAY_COUNT)
        return;
    hal_gpio_write(relay_pins[channel].port, relay_pins[channel].pin,
                   on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
    if (on)
        relay_state |= (uint16_t)(1u << channel);
    else
        relay_state &= ~(uint16_t)(1u << channel);
}

bool relay_get(uint8_t channel)
{
    if (channel >= RELAY_COUNT)
        return false;
    return (relay_state >> channel) & 0x1;
}

void relay_set_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++)
    {
        relay_set(i, (mask >> i) & 0x1);
    }
}

uint16_t relay_get_mask(void)
{
    return relay_state;
}

void relay_all_off(void)
{
    relay_set_mask(0);
}
