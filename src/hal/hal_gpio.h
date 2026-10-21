/**
 * @file    hal_gpio.h
 * @brief   Thin GPIO abstraction over the GD32 SPL.
 *          Keeps driver code free of vendor calls.
 */
#ifndef HAL_GPIO_H
#define HAL_GPIO_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    HAL_GPIO_INPUT,
    HAL_GPIO_INPUT_PULLUP,
    HAL_GPIO_OUTPUT,
    HAL_GPIO_OUTPUT_OD,
    HAL_GPIO_ANALOG,
    HAL_GPIO_ALTERNATE,
} hal_gpio_mode_t;

/** Configure a single pin. */
void hal_gpio_init(uint32_t port, uint32_t pin, hal_gpio_mode_t mode);

/** Drive an output pin. */
void hal_gpio_write(uint32_t port, uint32_t pin, bool level);

/** Read an input pin. */
bool hal_gpio_read(uint32_t port, uint32_t pin);

/** Toggle an output pin. */
void hal_gpio_toggle(uint32_t port, uint32_t pin);

#endif /* HAL_GPIO_H */
