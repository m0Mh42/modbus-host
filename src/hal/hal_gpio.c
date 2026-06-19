/**
 * @file hal_gpio.c
 * @brief GPIO abstraction implementation over the GD32F30x SPL.
 *
 * GD32F303 uses the STM32F1-style GPIO model (mode + speed, no separate
 * pull register for most modes). Edit the mode mapping below if your part
 * or routing needs different drive settings.
 */
#include "hal_gpio.h"
#include "gd32f30x.h"   /* SPL umbrella header (provided by framework=spl) */

/* Enable the RCU (peripheral clock) for a GPIO port base address.
 * Add ports here if you route signals to GPIOD/E/etc. */
static void enable_port_clock(uint32_t port)
{
    switch (port) {
        case GPIOA: rcu_periph_clock_enable(RCU_GPIOA); break;
        case GPIOB: rcu_periph_clock_enable(RCU_GPIOB); break;
        case GPIOC: rcu_periph_clock_enable(RCU_GPIOC); break;
        case GPIOD: rcu_periph_clock_enable(RCU_GPIOD); break;
        default: break;
    }
    /* AF clock is needed for alternate-function and EXTI remap. */
    rcu_periph_clock_enable(RCU_AF);
}

void hal_gpio_init(uint32_t port, uint32_t pin, hal_gpio_mode_t mode)
{
    enable_port_clock(port);

    /* Translate the portable mode enum to SPL mode + speed. */
    switch (mode) {
        case HAL_GPIO_INPUT:
            gpio_init(port, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, pin);
            break;
        case HAL_GPIO_INPUT_PULLUP:
            gpio_init(port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pin);
            break;
        case HAL_GPIO_OUTPUT:
            gpio_init(port, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, pin);
            break;
        case HAL_GPIO_OUTPUT_OD:
            gpio_init(port, GPIO_MODE_OUT_OD, GPIO_OSPEED_50MHZ, pin);
            break;
        case HAL_GPIO_ANALOG:
            gpio_init(port, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, pin);
            break;
        case HAL_GPIO_ALTERNATE:
            gpio_init(port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, pin);
            break;
        default:
            break;
    }
}

void hal_gpio_write(uint32_t port, uint32_t pin, bool level)
{
    gpio_bit_write(port, pin, level ? SET : RESET);
}

bool hal_gpio_read(uint32_t port, uint32_t pin)
{
    return gpio_input_bit_get(port, pin) == SET;
}

void hal_gpio_toggle(uint32_t port, uint32_t pin)
{
    /* GD32 SPL has no toggle helper; read the output latch and invert. */
    if (gpio_output_bit_get(port, pin) == SET) {
        gpio_bit_reset(port, pin);
    } else {
        gpio_bit_set(port, pin);
    }
}
