/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
typedef enum boot_power
{
    kBootPower_Normal, //!< Boot at 96MHz
    kBootPower_High    //!< Boot at 198MHz, high performance, high power consumption
} boot_power_t;

enum
{
    kFreq_1MHz = 1000000u,
    kFreq_4MHz = 4 * kFreq_1MHz,
    kFreq_24MHz = 24u * kFreq_1MHz,
    kFreq_48MHz = 48u * kFreq_1MHz,
    kFreq_198MHz = 198u * kFreq_1MHz,
    kFreq_264MHz = 264u * kFreq_1MHz,
    kFreq_16MHz = 16u * kFreq_1MHz,
    kFreq_96MHz = 96u * kFreq_1MHz,
    kFreq_240MHz = 240u * kFreq_1MHz,
    kDefaultUsbPllLockTimeUs = 200000u,
};

enum
{
    MAINCLKSELA_12MHz_IRC = 0,
    MAINCLKSELA_OSC_CLK,
    MAINCLKSELA_1M_LPOSC,
    MAINCLKSELA_48M_60M_IRC,
};

enum
{
    PLL528_CLKSRC_16M_IRC = 0,
    PLL528_CLKSRC_OSC,
    PLL528_CLKSRC_48MHz_60MHz_Div2
};

enum
{
    MAINCLKSELB_SYSCLK,
    MAINCLKSELB_16M_IRC,
    MAINCLKSELB_MAIN_PLL_CLK,
    MAINCLKSELB_32K_CLK
};

enum
{
    SYSOSCSEL_XTAL_CLK = 0,
    SYSOSCSEL_CLKIN_CLK,
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
static boot_power_t get_boot_power(void);
static void boot_set_clocks(boot_power_t boot_power);
static void init_syspll(uint32_t clk_src, uint32_t src_clk_freq);
static void delay_us_sw(uint32_t us);
extern void pmc_apply_cfg(void);

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void delay_us_sw(uint32_t us)
{
    uint32_t ticksPerUs = SystemCoreClock / kFreq_1MHz;

    while (us--)
    {
        volatile uint32_t delayTicks = 1 + ticksPerUs / 4;
        while (--delayTicks)
        {
            __NOP();
        }
    }
}

void init_syspll(uint32_t clk_src, uint32_t src_clk_freq)
{
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSPLLLDO_PD_MASK | SYSCTL0_PDRUNCFG0_SYSPLLANA_PD_MASK;

    CLKCTL0->SYSPLL0CLKSEL = clk_src;

    if (kFreq_24MHz == src_clk_freq)
    {
        CLKCTL0->SYSPLL0NUM = 0x0u;
        CLKCTL0->SYSPLL0DENOM = 0x01u;
        // Configure to 528M, assert HOLDRINGOFF in the first half of lock time
        CLKCTL0->SYSPLL0CTL0 = CLKCTL0_SYSPLL0CTL0_MULT(22u) | CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
        // De-assert HOLDRINGOFF in the second half of lock time
        CLKCTL0->SYSPLL0CTL0 &= (uint32_t)~CLKCTL0_SYSPLL0CTL0_HOLDRINGOFF_ENA_MASK;
        delay_us_sw(CLKCTL0->SYSPLL0LOCKTIMEDIV2 / 2);
    }
}

boot_power_t get_boot_power(void)
{
    boot_power_t bootPower = kBootPower_Normal;

    return bootPower;
}

void boot_set_clocks(boot_power_t boot_power)
{
    if (boot_power == kBootPower_High)
    {
        // Configure Clock to certain state before swithching to PLL
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;
        CLKCTL0->MAINCLKSELA = MAINCLKSELA_48M_60M_IRC;

        // Increase the core voltage to 1.138v before setting the PLL
        PMC->RUNCTRL = PMC_RUNCTRL_CORELVL(0x32);
        pmc_apply_cfg();

        // PLL_OUTPUT = 24MHz * (22 + 0 / 1) = 528MHz
        SystemCoreClock = kFreq_48MHz;
        init_syspll(PLL528_CLKSRC_48MHz_60MHz_Div2, kFreq_24MHz);
        // MAIN_PLL = 396MHz
        CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

        // 396MHz / 2 = 198MHz
        CLKCTL0->SYSCPUAHBCLKDIV = (2u - 1u);
        // for glitch free clock swiching. Select the divider clock first then enable divider.
        CLKCTL0->SYSTICKFCLKSEL = 0u;
        CLKCTL0->SYSTICKFCLKDIV = (2u - 1u);

        SystemCoreClock = kFreq_198MHz;

        // Switch to MAIN PLL
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_MAIN_PLL_CLK;
    }
    else
    {
        // Set IRC48M as clock source, core run at 48MHz
        CLKCTL0->SYSCPUAHBCLKDIV = 0u;
        CLKCTL0->SYSTICKFCLKSEL = 0u;
        CLKCTL0->SYSTICKFCLKDIV = 0u;
        CLKCTL0->MAINCLKSELA = MAINCLKSELA_48M_60M_IRC;
        CLKCTL0->MAINCLKSELB = MAINCLKSELB_SYSCLK;

        // Intialize the PLL for uSDHC
        init_syspll(PLL528_CLKSRC_48MHz_60MHz_Div2, kFreq_24MHz);
        // MAIN_PLL = 396MHz
        CLOCK_InitSysPfd(kCLOCK_Pfd0, 24u);

        SystemCoreClock = kFreq_48MHz;
    }

    // Configure uSDHC0 clock = 396/2 = 198 MHz; uSDHC1 clock = 396/4 = 99 MHz
    // Configure uSDHC clock source and divider: main_pll, divider = 1
    CLKCTL0->SDIO0FCLKDIV |= CLKCTL0_SDIO0FCLKDIV_HALT_MASK;
    CLKCTL0->SDIO1FCLKDIV |= CLKCTL0_SDIO1FCLKDIV_HALT_MASK;
    CLKCTL0->SDIO0FCLKSEL = 1u;
    CLKCTL0->SDIO1FCLKSEL = 1u;
    CLKCTL0->SDIO0FCLKDIV = 1u;
    CLKCTL0->SDIO1FCLKDIV = 3u;
}

// See bootloader_common for documentation on this function.
void configure_clocks(bootloader_clock_option_t option)
{
    boot_power_t boot_power = get_boot_power();

    // Configure Clock in ROM code
    if (kClockOption_EnterBootloader == option)
    {
        boot_set_clocks(boot_power);
    }
    else // Restore clock settings before leaving ROM code
    {
        // Restore uSDHC clock settings
        CLKCTL0->SDIO0FCLKDIV = CLKCTL0_SDIO0FCLKDIV_HALT(1);
        CLKCTL0->SDIO0FCLKSEL = 0x07u;
        CLKCTL0->SDIO1FCLKDIV = CLKCTL0_SDIO1FCLKDIV_HALT(1);
        CLKCTL0->SDIO1FCLKSEL = 0x07u;
    }
}
// See bootloader_common.h for documentation on this function.
// Note: this function doesn't apply to FPGA build
uint32_t get_system_core_clock(void)
{
    return CLOCK_GetFreq(kCLOCK_CoreSysClk);
}

// See bootloader_common.h for documentation on this function.
uint32_t get_bus_clock(void)
{
    return get_system_core_clock();
}


////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
