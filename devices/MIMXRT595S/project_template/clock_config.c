/*
 * Copyright 2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/
/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up all selectors to provide selected clocks.
 *
 * 3. Set up all dividers.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v6.0
processor: MIMXRT595S
package_id: MIMXRT595SFFOA
mcu_data: ksdk2_0
processor_version: 0.0.0
board: MIMXRT595-EVK
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "fsl_power.h"
#include "fsl_clock.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_FlexspiClockSafeConfig
 * Description   : FLEXSPI clock source safe configuration weak function.
 *                 Called before clock source(Such as PLL, Main clock) configuration.
 * Note          : Users need override this function to change FLEXSPI clock source to stable source when executing
 *                 code on FLEXSPI memory(XIP). If XIP, the function should runs in RAM and move the FLEXSPI clock
 *source to an stable clock to avoid instruction/data fetch issue during clock updating.
 *END**************************************************************************/
__attribute__((weak)) void BOARD_FlexspiClockSafeConfig(void)
{
}

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_SetFlexspiClock
 * Description   : This function should be overridden if executing code on FLEXSPI memory(XIP).
 *                 To Change FLEXSPI clock, should move to run from RAM and then configure FLEXSPI clock source.
 *                 After the clock is changed and stable,  move back to run on FLEXSPI.
 * Param base    : FLEXSPI peripheral base address.
 * Param src     : FLEXSPI clock source.
 * Param divider : FLEXSPI clock divider.
 *END**************************************************************************/
__attribute__((weak)) void BOARD_SetFlexspiClock(FLEXSPI_Type *base, uint32_t src, uint32_t divider)
{
    if (FLEXSPI0 == base)
    {
        CLKCTL0->FLEXSPI0FCLKSEL = CLKCTL0_FLEXSPI0FCLKSEL_SEL(src);
        CLKCTL0->FLEXSPI0FCLKDIV |= CLKCTL0_FLEXSPI0FCLKDIV_RESET_MASK; /* Reset the divider counter */
        CLKCTL0->FLEXSPI0FCLKDIV = CLKCTL0_FLEXSPI0FCLKDIV_DIV(divider - 1);
        while ((CLKCTL0->FLEXSPI0FCLKDIV) & CLKCTL0_FLEXSPI0FCLKDIV_REQFLAG_MASK)
        {
        }
    }
    else if (FLEXSPI1 == base)
    {
        CLKCTL0->FLEXSPI1FCLKSEL = CLKCTL0_FLEXSPI1FCLKSEL_SEL(src);
        CLKCTL0->FLEXSPI1FCLKDIV |= CLKCTL0_FLEXSPI1FCLKDIV_RESET_MASK; /* Reset the divider counter */
        CLKCTL0->FLEXSPI1FCLKDIV = CLKCTL0_FLEXSPI1FCLKDIV_DIV(divider - 1);
        while ((CLKCTL0->FLEXSPI1FCLKDIV) & CLKCTL0_FLEXSPI1FCLKDIV_REQFLAG_MASK)
        {
        }
    }
    else
    {
        return;
    }
}

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: CLKOUT_clock.outFreq, value: 1.92 MHz}
- {id: FLEXSPI0_clock.outFreq, value: 198 MHz}
- {id: LPOSC1M_clock.outFreq, value: 1 MHz}
- {id: OSTIMER_clock.outFreq, value: 1 MHz}
- {id: SYSTICK_clock.outFreq, value: 198 MHz}
- {id: System_clock.outFreq, value: 198 MHz}
- {id: USBPHY_clock.outFreq, value: 99 MHz}
- {id: WAKE_32K_clock.outFreq, value: 976.5625 Hz}
settings:
- {id: AUDIOPLL0_PFD0_CLK_GATE, value: 'No'}
- {id: CLKCTL.AUDIOPLL0CLKSEL.sel, value: CLKCTL.OSC_CLKSEL}
- {id: CLKCTL.AUDIOPLL0_PFD0_DIV.scale, value: '26', locked: true}
- {id: CLKCTL.AUDIOPLLCLKDIV.scale, value: '15', locked: true}
- {id: CLKCTL.AUDIO_PLL0_PFD0_MUL.scale, value: '18', locked: true}
- {id: CLKCTL.CLKOUTFCLKDIV.scale, value: '100', locked: true}
- {id: CLKCTL.CLKOUTSEL0.sel, value: CLKCTL.FRO_192M_EN}
- {id: CLKCTL.CLKOUTSEL1.sel, value: CLKCTL.CLKOUTSEL0}
- {id: CLKCTL.DMIC0FCLKDIV.scale, value: '1', locked: true}
- {id: CLKCTL.DSPCPUCLKDIV.scale, value: '1', locked: true}
- {id: CLKCTL.FLEXSPI0FCLKDIV.scale, value: '2', locked: true}
- {id: CLKCTL.FLEXSPI0FCLKSEL.sel, value: CLKCTL.MAINCLKSELB}
- {id: CLKCTL.FRGPLLCLKDIV.scale, value: '11', locked: true}
- {id: CLKCTL.I3C01FCLKSDIV.scale, value: '1', locked: true}
- {id: CLKCTL.MAINCLKSELB.sel, value: CLKCTL.PLL0_PFD0_BYPASS}
- {id: CLKCTL.PFCLKDIV1.scale, value: '4', locked: true}
- {id: CLKCTL.PLL0.denom, value: '1'}
- {id: CLKCTL.PLL0.div, value: '22'}
- {id: CLKCTL.PLL0.num, value: '0'}
- {id: CLKCTL.PLL0_PFD0_DIV.scale, value: '24', locked: true}
- {id: CLKCTL.PLL0_PFD0_MUL.scale, value: '18', locked: true}
- {id: CLKCTL.PLL0_PFD2_DIV.scale, value: '24', locked: true}
- {id: CLKCTL.PLL0_PFD2_MUL.scale, value: '18', locked: true}
- {id: CLKCTL.PLL1.denom, value: '75'}
- {id: CLKCTL.PLL1.div, value: '22'}
- {id: CLKCTL.PLL1.num, value: '14'}
- {id: CLKCTL.SYSCPUAHBCLKDIV.scale, value: '2', locked: true}
- {id: CLKCTL.SYSPLL0CLKSEL.sel, value: CLKCTL.OSC_CLKSEL}
- {id: CLKCTL.SYSTICKFCLKDIV.scale, value: '2', locked: true}
- {id: CLKCTL.SYSTICKFCLKSEL.sel, value: CLKCTL.SYSTICKFCLKDIV}
- {id: FRO_12M_EN_CFG, value: Enabled}
- {id: FRO_192M_EN_CFG, value: Enabled}
- {id: FRO_24M_EN_CFG, value: Enabled}
- {id: FRO_48M_EN_CFG, value: Enabled}
- {id: FRO_96M_EN_CFG, value: Enabled}
- {id: PLL0_PFD0_CLK_GATE, value: 'No'}
- {id: PLL0_PFD2_CLK_GATE, value: 'No'}
- {id: SYSCTL_PDRUNCFG_AUDIOPLL_CFG, value: 'No'}
- {id: SYSCTL_PDRUNCFG_FFRO_CFG, value: Power_up}
- {id: SYSCTL_PDRUNCFG_SYSPLL_CFG, value: 'No'}
- {id: SYSCTL_PDRUNCFG_SYSXTAL_CFG, value: Power_up}
- {id: XTAL_LP_Enable, value: LowPowerMode}
sources:
- {id: CLKCTL.XTAL.outFreq, value: 24 MHz, enabled: true}
- {id: CLKCTL.fro_192m.outFreq, value: 192 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
const clock_sys_pll_config_t g_sysPllConfig_BOARD_BootClockRUN = {
    .sys_pll_src  = kCLOCK_SysPllXtalIn, /* OSC clock */
    .numerator    = 0,                   /* Numerator of the SYSPLL0 fractional loop divider is 0 */
    .denominator  = 1,                   /* Denominator of the SYSPLL0 fractional loop divider is 1 */
    .sys_pll_mult = kCLOCK_SysPllMult22  /* Divide by 22 */
};
const clock_audio_pll_config_t g_audioPllConfig_BOARD_BootClockRUN = {
    .audio_pll_src  = kCLOCK_AudioPllXtalIn, /* OSC clock */
    .numerator      = 14,                    /* Numerator of the Audio PLL fractional loop divider is 0 */
    .denominator    = 75,                    /* Denominator of the Audio PLL fractional loop divider is 1 */
    .audio_pll_mult = kCLOCK_AudioPllMult22  /* Divide by 22 */
};
/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Configure LPOSC 1M */
    POWER_DisablePD(kPDRUNCFG_PD_LPOSC); /* Power on LPOSC (1MHz) */
    CLOCK_EnableLpOscClk();              /* Wait until LPOSC stable */

    /* Configure FRO clock source */
    POWER_DisablePD(kPDRUNCFG_PD_FFRO); /* Power on FFRO (192MHz) */
    /* FFRO DIV1(192 MHz) is always enabled and used as Main clock during PLL update */
    CLOCK_EnableFroClk(kCLOCK_FroAllOutEn); /* Enable all FRO outputs. */

    /* Call function BOARD_FlexspiClockSafeConfig() to move FlexSPI clock to a stable clock source to avoid
       instruction/data fetch issue when updating PLL and Main clock if XIP(execute code on FLEXSPI memory). */
    BOARD_FlexspiClockSafeConfig();

    /* Let CPU run on FRO with divider 2 (96Mhz) for safe switching */
    CLOCK_SetClkDiv(kCLOCK_DivSysCpuAhbClk, 2);
    CLOCK_AttachClk(kFRO192M_to_MAIN_CLK);

    /* Configure SYSOSC clock source */
    POWER_DisablePD(kPDRUNCFG_PD_SYSXTAL);                       /* Power on SYSXTAL */
    POWER_UpdateOscSettlingTime(BOARD_SYSOSC_SETTLING_US);       /* Updated XTAL oscillator settling time */
    CLOCK_EnableSysOscClk(true, true, BOARD_SYSOSC_SETTLING_US); /* Enable system OSC */
    CLOCK_SetXtalFreq(BOARD_XTAL_SYS_CLK_HZ);                    /* Sets external XTAL OSC freq */

    /* FIXME: Set the PLL lock time to 150us. Should be removed after getting trimmed sample. */
    CLKCTL0->SYSPLL0LOCKTIMEDIV2   = 150U;
    CLKCTL1->AUDIOPLL0LOCKTIMEDIV2 = 150U;

    /* Configure SysPLL0 clock source */
    CLOCK_InitSysPll(&g_sysPllConfig_BOARD_BootClockRUN);
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 24); /* Enable MAIN PLL clock */
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24); /* Enable AUX0 PLL clock */

    /* Configure Audio PLL clock source */
    CLOCK_InitAudioPll(&g_audioPllConfig_BOARD_BootClockRUN);
    CLOCK_InitAudioPfd(kCLOCK_Pfd0, 26); /* Enable Audio PLL clock */

    CLOCK_SetClkDiv(kCLOCK_DivSysCpuAhbClk, 2U); /* Set SYSCPUAHBCLKDIV divider to value 2 */

    /* Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(kMAIN_PLL_to_MAIN_CLK);        /* Switch MAIN_CLK to MAIN_PLL */
    CLOCK_AttachClk(kMAIN_CLK_DIV_to_SYSTICK_CLK); /* Switch SYSTICK_CLK to MAIN_CLK_DIV */
    CLOCK_AttachClk(kFRO192M_to_CLKOUT);           /* Switch CLKOUT to FRO192M */

    /* Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivAudioPllClk, 15U); /* Set AUDIOPLLCLKDIV divider to value 15 */
    CLOCK_SetClkDiv(kCLOCK_DivPLLFRGClk, 11U);   /* Set FRGPLLCLKDIV divider to value 11 */
    CLOCK_SetClkDiv(kCLOCK_DivSystickClk, 2U);   /* Set SYSTICKFCLKDIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivPfc1Clk, 4U);      /* Set PFCLKDIV1 divider to value 4 */
    CLOCK_SetClkDiv(kCLOCK_DivClockOut, 100U);   /* Set CLKOUTFCLKDIV divider to value 100 */

    /* Call function BOARD_SetFlexspiClock() to set user configured clock source/divider for FlexSPI. */
    BOARD_SetFlexspiClock(FLEXSPI0, 0U, 2U);

    /*< Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}
