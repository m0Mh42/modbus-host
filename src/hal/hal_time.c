/**
 * @file hal_time.c
 * @brief System tick / timing implementation (SysTick, 1 kHz).
 *
 * SysTick_Handler increments a millisecond counter. If you later move to an
 * RTOS, replace these with the RTOS tick source.
 */
#include "hal_time.h"
#include "gd32f30x.h"

static volatile uint32_t s_millis;

void hal_time_init(void)
{
    /* SystemCoreClock is set by the SPL system init. 1 kHz tick. */
    SysTick_Config(SystemCoreClock / 1000U);
}

/* SysTick interrupt - keep the name matching the vector table. */
void SysTick_Handler(void)
{
    s_millis++;
}

uint32_t hal_millis(void)
{
    return s_millis;
}

void hal_delay_ms(uint32_t ms)
{
    uint32_t start = s_millis;
    while ((s_millis - start) < ms) {
        /* busy-wait; relies on SysTick interrupt running */
    }
}
