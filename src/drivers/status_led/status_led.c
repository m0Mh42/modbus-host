/**
 * @file    status_led.c
 * @brief   Ready / Fault Dual-LED indicator implementation (PB2).
 */
#include "status_led.h"
#include "board_config.h"
#include "hal_gpio.h"
#include "hal_time.h"

static status_led_mode_t current_mode = STATUS_LED_READY_HEARTBEAT;
static uint32_t last_tick_ms = 0;

static void led_drive_green(void)
{
    hal_gpio_init(STATUS_LED_PORT, STATUS_LED_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_write(STATUS_LED_PORT, STATUS_LED_PIN, true);
}

static void led_drive_red(void)
{
    hal_gpio_init(STATUS_LED_PORT, STATUS_LED_PIN, HAL_GPIO_OUTPUT);
    hal_gpio_write(STATUS_LED_PORT, STATUS_LED_PIN, false);
}

static void led_drive_off(void)
{
    hal_gpio_init(STATUS_LED_PORT, STATUS_LED_PIN, HAL_GPIO_INPUT);
}

void status_led_init(void)
{
    led_drive_green();
    current_mode = STATUS_LED_READY_HEARTBEAT;
    last_tick_ms = hal_millis();
}

void status_led_set_mode(status_led_mode_t mode)
{
    current_mode = mode;
}

void status_led_set_fault_state(bool has_fault, bool is_acknowledged)
{
    if (has_fault)
    {
        current_mode = is_acknowledged ? STATUS_LED_FAULT_SOLID : STATUS_LED_FAULT_FLASH;
    }
    else
    {
        current_mode = STATUS_LED_READY_HEARTBEAT;
    }
}

void status_led_task(void)
{
    uint32_t now = hal_millis();

    switch (current_mode)
    {
    case STATUS_LED_READY_SOLID:
        led_drive_green();
        break;

    case STATUS_LED_FAULT_SOLID:
        led_drive_red();
        break;

    case STATUS_LED_READY_HEARTBEAT:
    {
        /* 150ms Green heartbeat pulse every 1000ms */
        uint32_t phase = now % 1000;
        if (phase < 150)
        {
            led_drive_green();
        }
        else
        {
            led_drive_off();
        }
        break;
    }

    case STATUS_LED_FAULT_FLASH:
    {
        /* 250ms Red flash every 500ms (2 Hz alert) */
        uint32_t phase = now % 500;
        if (phase < 250)
        {
            led_drive_red();
        }
        else
        {
            led_drive_off();
        }
        break;
    }

    case STATUS_LED_OFF:
    default:
        led_drive_off();
        break;
    }
}
