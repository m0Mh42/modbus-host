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

#define RELAY_ACTIVE_LEVEL  true   /* MCU pin level that energizes the relay */

/* Per-channel pin map. TODO: fill port/pin from MainBoard.SchDoc (RELAY_1..16).
 * Order here is logical relay 0..15. */
typedef struct { uint32_t port; uint32_t pin; } relay_pin_t;

static const relay_pin_t relay_pins[RELAY_COUNT] = {
    /* { GPIOx, GPIO_PIN_y }, ... 16 entries. TODO populate. */
    { GPIOB, GPIO_PIN_0 }, { GPIOB, GPIO_PIN_1 }, { GPIOB, GPIO_PIN_2 }, { GPIOB, GPIO_PIN_3 },
    { GPIOB, GPIO_PIN_4 }, { GPIOB, GPIO_PIN_5 }, { GPIOB, GPIO_PIN_6 }, { GPIOB, GPIO_PIN_7 },
    { GPIOB, GPIO_PIN_8 }, { GPIOB, GPIO_PIN_9 }, { GPIOC, GPIO_PIN_0 }, { GPIOC, GPIO_PIN_1 },
    { GPIOC, GPIO_PIN_2 }, { GPIOC, GPIO_PIN_3 }, { GPIOC, GPIO_PIN_4 }, { GPIOC, GPIO_PIN_5 },
};

static uint16_t relay_state;

void relay_init(void)
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        hal_gpio_init(relay_pins[i].port, relay_pins[i].pin, HAL_GPIO_OUTPUT);
        hal_gpio_write(relay_pins[i].port, relay_pins[i].pin, !RELAY_ACTIVE_LEVEL);
    }
    relay_state = 0;
}

void relay_set(uint8_t channel, bool on)
{
    if (channel >= RELAY_COUNT) return;
    hal_gpio_write(relay_pins[channel].port, relay_pins[channel].pin,
                   on ? RELAY_ACTIVE_LEVEL : !RELAY_ACTIVE_LEVEL);
    if (on) relay_state |=  (uint16_t)(1u << channel);
    else    relay_state &= ~(uint16_t)(1u << channel);
}

bool relay_get(uint8_t channel)
{
    if (channel >= RELAY_COUNT) return false;
    return (relay_state >> channel) & 0x1;
}

void relay_set_mask(uint16_t mask)
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
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
