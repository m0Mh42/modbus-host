/**
 * @file hal_time.c
 * @brief System tick / timing implementation (SysTick, 1 kHz).
 *
 * SysTick_Handler increments a millisecond counter. If you later move to an
 * RTOS, replace these with the RTOS tick source.
 */
#include "hal_time.h"
#include "hal_clock.h"
#include "hal_uart.h"
#include "gd32f30x.h"

static volatile uint32_t s_millis;

void hal_time_init(void)
{
    /* Initialize clock monitoring and verify external crystal / CKM */
    hal_clock_init();

    /* Update core clock from system registers (120 MHz with PLL) */
    SystemCoreClockUpdate();

    /* Configure NVIC priority group (4 bits preemption, 0 bits subpriority) */
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

    /* 1 kHz SysTick tick */
    SysTick_Config(SystemCoreClock / 1000U);
}

/* SysTick interrupt - keep the name matching the vector table. */
void SysTick_Handler(void)
{
    s_millis++;
    hal_uart_timer_1ms_tick();
}

uint32_t hal_millis(void)
{
    return s_millis;
}

void hal_delay_ms(uint32_t ms)
{
    uint32_t start = s_millis;
    while ((s_millis - start) < ms)
    {
        /* busy-wait; relies on SysTick interrupt running */
    }
}
