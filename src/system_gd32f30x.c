/*!
    \file    system_gd32f30x.c
    \brief   CMSIS Cortex-M4 Device Peripheral Access Layer Source File for
             GD32F30x Device Series with robust crystal startup & fallback.
*/

#ifndef UNIT_TEST

#include "gd32f30x.h"
#include "hal_clock.h"

/* System frequency define */
#define __IRC8M (IRC8M_VALUE)   /* internal 8 MHz RC oscillator frequency */
#define __HXTAL (HXTAL_VALUE)   /* high speed crystal oscillator frequency */
#define __SYS_OSC_CLK (__IRC8M) /* main oscillator frequency */

#define SEL_IRC8M 0x00U
#define SEL_HXTAL 0x01U
#define SEL_PLL 0x02U
#define RCU_MODIFY                      \
    {                                   \
        volatile uint32_t i;            \
        RCU_CFG0 |= RCU_AHB_CKSYS_DIV2; \
        for (i = 0; i < 20000; i++)     \
            ;                           \
    }

#define HXTAL_TIMEOUT_COUNT 0x3FFFFU /* Generous startup timeout (~100ms) for YJF 8.000MHz */

uint32_t SystemCoreClock = 120000000U;

static void system_clock_120m_irc8m(void);
static void system_clock_120m_hxtal(void);
static void system_clock_config(void);

/*!
    \brief      setup the microcontroller system, initialize the system
    \param[in]  none
    \param[out] none
    \retval     none
*/
void SystemInit(void)
{
    /* FPU settings */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif
    /* Reset the RCU clock configuration to the default reset state */
    /* Set IRC8MEN bit */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    RCU_MODIFY

    /* Reset CFG0 and CFG1 registers */
    RCU_CFG0 = 0x00000000U;
    RCU_CFG1 = 0x00000000U;

    /* Reset HXTALEN, CKMEN and PLLEN bits */
    RCU_CTL &= ~(RCU_CTL_PLLEN | RCU_CTL_CKMEN | RCU_CTL_HXTALEN);
    /* Disable all interrupts */
    RCU_INT = 0x009f0000U;

    /* Reset HXTALBPS bit */
    RCU_CTL &= ~(RCU_CTL_HXTALBPS);

    /* Configure the system clock source, PLL multiplier, AHB/APBx prescalers */
    system_clock_config();
}

/*!
    \brief      configure the system clock
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void system_clock_config(void)
{
    /* Attempt to use the external crystal (8 MHz HXTAL -> 120 MHz PLL).
     * If crystal startup fails, system_clock_120m_hxtal() automatically falls
     * back to system_clock_120m_irc8m(). */
    system_clock_120m_hxtal();
}

/*!
    \brief      configure the system clock to 120M by PLL which selects IRC8M as its clock source
*/
static void system_clock_120m_irc8m(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* Enable IRC8M */
    RCU_CTL |= RCU_CTL_IRC8MEN;

    /* Wait until IRC8M is stable */
    do
    {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_IRC8MSTB);
    } while ((0U == stab_flag) && (IRC8M_STARTUP_TIMEOUT != timeout));

    if (0U == (RCU_CTL & RCU_CTL_IRC8MSTB))
    {
        while (1)
        {
        }
    }

    /* LDO output voltage high mode */
    RCU_APB1EN |= RCU_APB1EN_PMUEN;
    PMU_CTL |= PMU_CTL_LDOVS;

    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* CK_PLL = (CK_IRC8M/2) * 30 = 120 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
    RCU_CFG0 |= RCU_PLLSRC_IRC8M_DIV2;
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
    RCU_CFG0 |= RCU_PLL_MUL30;

    /* Enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* Wait until PLL is stable */
    while (0U == (RCU_CTL & RCU_CTL_PLLSTB))
    {
    }

    /* Enable the high-drive to extend clock frequency to 120 MHz */
    PMU_CTL |= PMU_CTL_HDEN;
    while (0U == (PMU_CS & PMU_CS_HDRF))
    {
    }

    PMU_CTL |= PMU_CTL_HDS;
    while (0U == (PMU_CS & PMU_CS_HDSRF))
    {
    }

    /* Select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* Wait until PLL is selected as system clock */
    while (0U == (RCU_CFG0 & RCU_SCSS_PLL))
    {
    }
}

/*!
    \brief      configure system clock to 120M by PLL with HXTAL(8M), with automatic IRC8M fallback
*/
static void system_clock_120m_hxtal(void)
{
    uint32_t timeout = 0U;
    uint32_t stab_flag = 0U;

    /* Enable HXTAL */
    RCU_CTL |= RCU_CTL_HXTALEN;

    /* Wait until HXTAL is stable or startup timeout expires */
    do
    {
        timeout++;
        stab_flag = (RCU_CTL & RCU_CTL_HXTALSTB);
    } while ((0U == stab_flag) && (timeout < HXTAL_TIMEOUT_COUNT));

    /* If crystal startup fails (absent, slow, or damaged crystal), DO NOT HANG!
     * Fall back immediately to internal 8 MHz RC oscillator. */
    if (0U == (RCU_CTL & RCU_CTL_HXTALSTB))
    {
        RCU_CTL &= ~RCU_CTL_HXTALEN;
        system_clock_120m_irc8m();
        hal_clock_record_crystal_fault();
        return;
    }

    RCU_APB1EN |= RCU_APB1EN_PMUEN;
    PMU_CTL |= PMU_CTL_LDOVS;

    /* HXTAL is stable */
    /* AHB = SYSCLK */
    RCU_CFG0 |= RCU_AHB_CKSYS_DIV1;
    /* APB2 = AHB/1 */
    RCU_CFG0 |= RCU_APB2_CKAHB_DIV1;
    /* APB1 = AHB/2 */
    RCU_CFG0 |= RCU_APB1_CKAHB_DIV2;

    /* Select HXTAL/2 as clock source: 8 MHz / 2 = 4 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLSEL | RCU_CFG0_PREDV0);
    RCU_CFG0 |= (RCU_PLLSRC_HXTAL_IRC48M | RCU_CFG0_PREDV0);

    /* CK_PLL = (CK_HXTAL/2) * 30 = 120 MHz */
    RCU_CFG0 &= ~(RCU_CFG0_PLLMF | RCU_CFG0_PLLMF_4 | RCU_CFG0_PLLMF_5);
    RCU_CFG0 |= RCU_PLL_MUL30;

    /* Enable PLL */
    RCU_CTL |= RCU_CTL_PLLEN;

    /* Wait until PLL is stable */
    while (0U == (RCU_CTL & RCU_CTL_PLLSTB))
    {
    }

    /* Enable high-drive to extend the clock frequency to 120 MHz */
    PMU_CTL |= PMU_CTL_HDEN;
    while (0U == (PMU_CS & PMU_CS_HDRF))
    {
    }

    PMU_CTL |= PMU_CTL_HDS;
    while (0U == (PMU_CS & PMU_CS_HDSRF))
    {
    }

    /* Select PLL as system clock */
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    RCU_CFG0 |= RCU_CKSYSSRC_PLL;

    /* Wait until PLL is selected as system clock */
    while (0U == (RCU_CFG0 & RCU_SCSS_PLL))
    {
    }

    /* Enable Clock Security System (CKM) to detect crystal failure during runtime */
    RCU_CTL |= RCU_CTL_CKMEN;
}

/*!
    \brief      update the SystemCoreClock with current core clock retrieved from cpu registers
*/
void SystemCoreClockUpdate(void)
{
    uint32_t sws;
    uint32_t pllsel, predv0sel, pllmf, ck_src, idx, clk_exp;
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch (sws)
    {
    case SEL_IRC8M:
        SystemCoreClock = IRC8M_VALUE;
        break;
    case SEL_HXTAL:
        SystemCoreClock = HXTAL_VALUE;
        break;
    case SEL_PLL:
        pllsel = (RCU_CFG0 & RCU_CFG0_PLLSEL);
        if (RCU_PLLSRC_HXTAL_IRC48M == pllsel)
        {
            ck_src = HXTAL_VALUE;
            predv0sel = (RCU_CFG0 & RCU_CFG0_PREDV0);
            if (RCU_CFG0_PREDV0 == predv0sel)
            {
                ck_src = HXTAL_VALUE / 2U;
            }
        }
        else
        {
            ck_src = IRC8M_VALUE / 2U;
        }

        pllmf = GET_BITS(RCU_CFG0, 18, 21);
        if ((RCU_CFG0 & RCU_CFG0_PLLMF_4) != 0U)
        {
            pllmf |= 0x10U;
        }
        if ((RCU_CFG0 & RCU_CFG0_PLLMF_5) != 0U)
        {
            pllmf |= 0x20U;
        }

        if (pllmf < 15U)
        {
            SystemCoreClock = ck_src * (pllmf + 2U);
        }
        else if (pllmf < 63U)
        {
            SystemCoreClock = ck_src * (pllmf + 1U);
        }
        else
        {
            SystemCoreClock = ck_src * 64U;
        }
        break;
    default:
        SystemCoreClock = IRC8M_VALUE;
        break;
    }

    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    SystemCoreClock >>= clk_exp;
}

#endif /* !UNIT_TEST */
