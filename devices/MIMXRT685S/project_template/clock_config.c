/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
const clock_sys_pll_config_t g_configSysPll = {
    .sys_pll_src = kCLOCK_SysPllXtalIn, .numerator = 0, .denominator = 1, .sys_pll_mult = kCLOCK_SysPllMult22};
const clock_audio_pll_config_t g_configAudioPll = {
    .audio_pll_src = kCLOCK_AudioPllXtalIn, .numerator = 5040, .denominator = 27000, .mult = 22U};

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}
 
/* BOARD_SetQspiClock should run in RAM */
AT_QUICKACCESS_SECTION_CODE(void BOARD_SetQspiClock(QuadSPI_Type *qspi, uint32_t qspiClockSrc, uint32_t divider))
{
    int32_t i;

    if (qspi)
    {
        /* Make sure QSPI clock is enabled */
        CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_OSPI_OTFAD_CLK_MASK;
        /* Wait until QSPI is not busy */
        while (qspi->SR & QuadSPI_SR_BUSY_MASK)
        {
        }
        /* Make sure module is enabled when reset */
        qspi->MCR &= ~QuadSPI_MCR_MDIS_MASK;
        qspi->MCR |= QuadSPI_MCR_SWRSTHD_MASK | QuadSPI_MCR_SWRSTSD_MASK;
        /* Wait enough cycles until serial flash are reset */
        for (i = 0; i < 200; i++)
        {
            qspi->SR;
        }
        /* Disable module during the reset procedure */
        qspi->MCR |= QuadSPI_MCR_MDIS_MASK;
        /* Clear the reset bits. */
        qspi->MCR &= ~(QuadSPI_MCR_SWRSTHD_MASK | QuadSPI_MCR_SWRSTSD_MASK);
        /* Disable clock before changing clock source */
        CLKCTL0->PSCCTL0_CLR = CLKCTL0_PSCCTL0_SET_OSPI_OTFAD_CLK_MASK;
        /* Update qspi clock. */
        CLKCTL0->OSPIFCLKSEL = qspiClockSrc;
        CLKCTL0->OSPIFCLKDIV |= CLKCTL0_OSPIFCLKDIV_RESET_MASK; /* Reset the divider counter */
        CLKCTL0->OSPIFCLKDIV = CLKCTL0_OSPIFCLKDIV_DIV(divider - 1);
        while ((CLKCTL0->OSPIFCLKDIV) & CLKCTL0_OSPIFCLKDIV_REQFLAG_MASK)
        {
        }
        /* Enable QSPI clock again */
        CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_OSPI_OTFAD_CLK_MASK;
        /* Enable QSPI cache */
        qspi->SOCCR &= ~(QuadSPI_SOCCR_DIS_LPCAC_MASK | QuadSPI_SOCCR_DIS_LPCAC_WTBF_MASK |
                         QuadSPI_SOCCR_FRC_NO_ALLOC_MASK | QuadSPI_SOCCR_CLR_LPCAC_MASK);
        /* Re-enable QSPI module */
        qspi->MCR &= ~QuadSPI_MCR_MDIS_MASK;
    }
}

void BOARD_BootClockVLPR(void)
{
}

void BOARD_BootClockRUN(void)
{
    QuadSPI_Type *qspi = NULL;

    CLOCK_SetXtalFreq(BOARD_XTAL_SYS_CLK_HZ); /* sets external XTAL OSC freq */

    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_LPOSC_PD_MASK;   /* Enable LPOSC (1MHz). */
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SFRO_PD_MASK;    /* Enable SFRO (16MHz). */
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_FFRO_PD_MASK;    /* Enable FFRO (48MHz). */
    SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_SYSXTAL_PD_MASK; /* Enable SYSXTAL. */
    CLOCK_EnableSysOscClk(true);                                /* Enable system OSC */
    CLOCK_EnableOsc32K(true);                                   /* Enable 32K OSC */

    /* Set qspi base address if XIP (execute code on QSPI memory secure or non-secure address) */
    if ((((uint32_t)BOARD_BootClockRUN >= FSL_FEATURE_QSPI_AMBA_BASE) &&
         ((uint32_t)BOARD_BootClockRUN < (FSL_FEATURE_QSPI_AMBA_BASE + 0x8000000))) ||
        (((uint32_t)BOARD_BootClockRUN >= (FSL_FEATURE_QSPI_AMBA_BASE + 0x10000000)) &&
         ((uint32_t)BOARD_BootClockRUN < (FSL_FEATURE_QSPI_AMBA_BASE + 0x18000000))))
    {
        qspi = QUADSPI;
    }

    /* Move QSPI clock source from main clock to FFRO to avoid instruction/data fetch issue in XIP when updating
     * PLL and main clock.
     */
    BOARD_SetQspiClock(qspi, CLKCTL0_OSPIFCLKSEL_SEL(3), 1);

    CLOCK_AttachClk(kFFRO_to_MAIN_CLK); /* Let CPU run on ffro before configure SYS PLL. */

    /* for loop of 12500 is about 1ms (@48 MHz CPU) */
    for (uint32_t i = 500000U; i > 0; i--)
    {
        __asm("NOP");
    }

    CLOCK_InitSysPll(&g_configSysPll); /* Configure system PLL to 528Mhz. */
    /* Valid PFD values are decimal 12-35. */
    CLOCK_InitSysPfd(kCLOCK_Pfd0, 19); /* Enable main PLL clock 500MHz. */
    CLOCK_InitSysPfd(kCLOCK_Pfd1, 17); /* Enable dsp PLL clock 559MHz. */
    CLOCK_InitSysPfd(kCLOCK_Pfd2, 24); /* Enable aux0 PLL clock 396MHz for SDIO */

    CLOCK_InitAudioPll(&g_configAudioPll); /* Configure audio PLL to 532.48Mhz. */

    CLOCK_InitAudioPfd(kCLOCK_Pfd0, 26);        /* Enable audio PLL PFD0 368.64MHz */
    CLOCK_SetClkDiv(kCLOCK_DivAudioPllClk, 15); /* Configure audio_pll_clk to 24.576Mhz */

    /* Let CPU run on SYS PLL PFD0 with divider 2 (250Mhz). */
    CLOCK_SetClkDiv(kCLOCK_DivSysCpuAhbClk, 2);
    CLOCK_AttachClk(kMAIN_PLL_to_MAIN_CLK);

    /* for loop of 65000 is about 1ms (@250 MHz CPU) */
    for (uint32_t i = 2600000U; i > 0; i--)
    {
        __asm("NOP");
    }

    /* Let ARM Systick run same frequency with CPU (250Mhz). */
    CLOCK_AttachClk(kMAIN_CLK_DIV_to_SYSTICK_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivSystickClk, 2);

    /* Let QSPI run on main clock (500/2 = 250MHz) for MX25UM51345G flash. This
     * results in octal DDR read at 62.5MHz I/O speed.
     * NOTE: The frequency limit is up to the flash memory.
     *       Need to adjust QSPI clock frequency if you want to use other flash memory.
     */
    BOARD_SetQspiClock(qspi, CLKCTL0_OSPIFCLKSEL_SEL(0), 2);

    /*Let DSP run on SYS PLL PFD1 with divider 2 (594Mhz). */
    CLOCK_AttachClk(kDSP_PLL_to_DSP_MAIN_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivDspCpuClk, 1);
    CLOCK_SetClkDiv(kCLOCK_DivDspRamClk, 2);

    /* Configure frg_pll clock to 62.5Mhz. */
    CLOCK_SetClkDiv(kCLOCK_DivPllFrgClk, 8);

    /* Select CLKOUT mux as SYSPLL PFD0 with divider 100. */
    CLOCK_AttachClk(kMAIN_PLL_to_CLKOUT);
    CLOCK_SetClkDiv(kCLOCK_DivClockOut, 100);

    SystemCoreClockUpdate();
}
