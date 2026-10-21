/**
 * @file    hal_clock.c
 * @brief   Clock monitoring and crystal failure recovery for GD32F303.
 */
#include "hal_clock.h"

#ifndef UNIT_TEST
#include "gd32f30x.h"

static volatile bool s_crystal_fault = false;
static volatile hal_clock_source_t s_clock_source = HAL_CLOCK_SOURCE_HXTAL;

void hal_clock_init(void)
{
    /* Check currently selected system clock source and PLL input */
    uint32_t scss = (RCU_CFG0 & RCU_CFG0_SCSS);

    if (scss == RCU_SCSS_PLL)
    {
        if ((RCU_CFG0 & RCU_CFG0_PLLSEL) != 0U)
        {
            /* PLL source is external crystal (HXTAL) */
            s_clock_source = HAL_CLOCK_SOURCE_HXTAL;
            /* Ensure Clock Security System (CKM) is active */
            RCU_CTL |= RCU_CTL_CKMEN;
        }
        else
        {
            /* PLL source is internal RC oscillator (IRC8M) */
            s_clock_source = HAL_CLOCK_SOURCE_IRC8M;
            s_crystal_fault = true;
        }
    }
    else if (scss == RCU_SCSS_HXTAL)
    {
        s_clock_source = HAL_CLOCK_SOURCE_HXTAL;
        RCU_CTL |= RCU_CTL_CKMEN;
    }
    else
    {
        /* Running directly on IRC8M */
        s_clock_source = HAL_CLOCK_SOURCE_IRC8M;
        s_crystal_fault = true;
    }
}

hal_clock_source_t hal_clock_get_source(void)
{
    return s_clock_source;
}

bool hal_clock_is_using_crystal(void)
{
    return (s_clock_source == HAL_CLOCK_SOURCE_HXTAL);
}

bool hal_clock_has_crystal_fault(void)
{
    return s_crystal_fault;
}

void hal_clock_record_crystal_fault(void)
{
    s_crystal_fault = true;
    s_clock_source = HAL_CLOCK_SOURCE_IRC8M;
}

void hal_clock_clear_fault(void)
{
    s_crystal_fault = false;
}

/**
 * @brief Cortex-M Non-Maskable Interrupt (NMI) Handler.
 *
 * Traps runtime clock failures signaled by the GD32 Clock Security System (CKM).
 * When HXTAL oscillator fails or stops during runtime:
 *   1. Hardware automatically switches CK_SYS to internal IRC8M (8 MHz),
 *      disables HXTAL, disables PLL, and asserts CKMIF.
 *   2. This handler clears the CKM interrupt flag.
 *   3. Reconfigures the PLL to multiply IRC8M/2 (4 MHz) * 30 = 120 MHz.
 *   4. Switches CK_SYS back to PLL so the core, SysTick (1 ms), and USART2
 *      (Modbus 115200 baud) continue operating at design frequency without crashing.
 *   5. Flags the crystal error for supervisory monitoring.
 */
void NMI_Handler(void)
{
    if (rcu_interrupt_flag_get(RCU_INT_FLAG_CKM) != RESET)
    {
        /* Clear the CKM interrupt flag */
        rcu_interrupt_flag_clear(RCU_INT_FLAG_CKM_CLR);

        /* Ensure internal 8 MHz RC oscillator is enabled and stable */
        RCU_CTL |= RCU_CTL_IRC8MEN;
        uint32_t irc_timeout = 0U;
        while ((0U == (RCU_CTL & RCU_CTL_IRC8MSTB)) && (++irc_timeout < 0xFFFFU))
        {
            __NOP();
        }

        /* Configure PLL: source = IRC8M / 2 (4 MHz), multiplier = 30 -> 120 MHz */
        RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
        RCU_CFG0 |= RCU_PLLSRC_IRC8M_DIV2;
        RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
        RCU_CFG0 |= RCU_PLL_MUL30;

        /* Enable PLL */
        RCU_CTL |= RCU_CTL_PLLEN;
        uint32_t pll_timeout = 0U;
        while ((0U == (RCU_CTL & RCU_CTL_PLLSTB)) && (++pll_timeout < 0xFFFFU))
        {
            __NOP();
        }

        if (RCU_CTL & RCU_CTL_PLLSTB)
        {
            /* Enable PMU high-drive mode for 120 MHz */
            RCU_APB1EN |= RCU_APB1EN_PMUEN;
            PMU_CTL |= PMU_CTL_HDEN;
            while (0U == (PMU_CS & PMU_CS_HDRF))
            {
                __NOP();
            }
            PMU_CTL |= PMU_CTL_HDS;
            while (0U == (PMU_CS & PMU_CS_HDSRF))
            {
                __NOP();
            }

            /* Select PLL as system clock */
            RCU_CFG0 &= ~RCU_CFG0_SCS;
            RCU_CFG0 |= RCU_CKSYSSRC_PLL;
            uint32_t scs_timeout = 0U;
            while ((0U == (RCU_CFG0 & RCU_SCSS_PLL)) && (++scs_timeout < 0xFFFFU))
            {
                __NOP();
            }
            SystemCoreClock = 120000000U;
        }
        else
        {
            /* Fallback: keep running directly on IRC8M (8 MHz) and adjust SysTick */
            SystemCoreClock = 8000000U;
            SysTick_Config(SystemCoreClock / 1000U);
        }

        /* Record runtime crystal failure */
        hal_clock_record_crystal_fault();
    }
}

#else /* UNIT_TEST */

static bool s_mock_crystal_fault = false;
static hal_clock_source_t s_mock_source = HAL_CLOCK_SOURCE_HXTAL;

void hal_clock_init(void)
{
    s_mock_source = HAL_CLOCK_SOURCE_HXTAL;
    s_mock_crystal_fault = false;
}

hal_clock_source_t hal_clock_get_source(void)
{
    return s_mock_source;
}

bool hal_clock_is_using_crystal(void)
{
    return (s_mock_source == HAL_CLOCK_SOURCE_HXTAL);
}

bool hal_clock_has_crystal_fault(void)
{
    return s_mock_crystal_fault;
}

void hal_clock_record_crystal_fault(void)
{
    s_mock_crystal_fault = true;
    s_mock_source = HAL_CLOCK_SOURCE_IRC8M;
}

void hal_clock_clear_fault(void)
{
    s_mock_crystal_fault = false;
}

#endif /* UNIT_TEST */
