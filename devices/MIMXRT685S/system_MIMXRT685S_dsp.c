/*
** ###################################################################
**     Processor:           MIMXRT685SEVKA
**     Compiler:            XCC Compiler
**     Reference manual:    MIMXRT685 User manual Rev. 0.1 18 July 2017
**     Version:             rev. 1.0, 2018-06-19
**     Build:               b190722
**
**     Abstract:
**         Provides a system configuration function and a global variable that
**         contains the system frequency. It configures the device and initializes
**         the oscillator (PLL) that is part of the microcontroller device.
**
**     Copyright 2016 Freescale Semiconductor, Inc.
**     Copyright 2016-2019 NXP
**     All rights reserved.
**
**     SPDX-License-Identifier: BSD-3-Clause
**
**     http:                 www.nxp.com
**     mail:                 support@nxp.com
**
**     Revisions:
**     - rev. 1.0 (2018-06-19)
**         Initial version.
**
** ###################################################################
*/

/*!
 * @file MIMXRT685S
 * @version 1.0
 * @date 220719
 * @brief Device specific configuration file for MIMXRT685S (implementation file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"

static uint32_t getSpllFreq(void)
{
  uint32_t freq = 0U;
  uint64_t freqTmp = 0U;

  switch ((CLKCTL0->SYSPLL0CLKSEL) & CLKCTL0_SYSPLL0CLKSEL_SEL_MASK)
  {
    case CLKCTL0_SYSPLL0CLKSEL_SEL(0): /* SFRO clock */
      freq = CLK_FRO_16MHZ;
      break;
    case CLKCTL0_SYSPLL0CLKSEL_SEL(1): /* External clock (clk_in) */
      freq = 24000000U;
      break;
    case CLKCTL0_SYSPLL0CLKSEL_SEL(2): /* FRO clock (48m_irc) divider by 2 */
      freq = 24000000U;
      break;
    default:
      break;
  }

  if (((CLKCTL0->SYSPLL0CTL0) & CLKCTL0_SYSPLL0CTL0_BYPASS_MASK) == 0U)
  {
    /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM). */
    freqTmp = ((uint64_t)freq * ((uint64_t)(CLKCTL0->SYSPLL0NUM))) / ((uint64_t)(CLKCTL0->SYSPLL0DENOM));
    freq *= ((CLKCTL0->SYSPLL0CTL0) & CLKCTL0_SYSPLL0CTL0_MULT_MASK) >> CLKCTL0_SYSPLL0CTL0_MULT_SHIFT;
    freq += (uint32_t)freqTmp;
  }

  return freq;
}

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInit (void) {
  SystemInitHook();
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  /* iMXRT6xx systemCoreClockUpdate */
  uint32_t freq = 0U;

  switch ((CLKCTL1->DSPCPUCLKSELB) & CLKCTL1_DSPCPUCLKSELB_SEL_MASK)
  {
    case CLKCTL1_DSPCPUCLKSELB_SEL(0): /* DSPCPUCLKSELA clock */
      switch ((CLKCTL1->DSPCPUCLKSELA) & CLKCTL1_DSPCPUCLKSELA_SEL_MASK)
      {
        case CLKCTL1_DSPCPUCLKSELA_SEL(0): /* FRO clock (48m_irc) divider by 4 */
          freq = 12000000U;
          break;
        case CLKCTL1_DSPCPUCLKSELA_SEL(1): /* External clock (clk_in) */
          freq = 24000000U;
          break;
        case CLKCTL1_DSPCPUCLKSELA_SEL(2): /* Low Power Oscillator Clock (1m_lposc) */
          freq = CLK_LPOSC_1MHZ;
          break;
        case CLKCTL1_DSPCPUCLKSELA_SEL(3): /* SFRO clock */
          freq = CLK_FRO_16MHZ;
          break;
        default:
            break;
      }
      break;
    case CLKCTL1_DSPCPUCLKSELB_SEL(1): /* Main System PLL clock */
      freq = getSpllFreq();
      if (((CLKCTL0->SYSPLL0CTL0) & CLKCTL0_SYSPLL0CTL0_BYPASS_MASK) == 0U)
      {
        freq = (uint64_t)freq * 18 / ((CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD0_MASK) >>
                                      CLKCTL0_SYSPLL0PFD_PFD0_SHIFT);
      }
      break;
    case CLKCTL1_DSPCPUCLKSELB_SEL(2): /* DSP PLL clock */
      freq = getSpllFreq();
      if (((CLKCTL0->SYSPLL0CTL0) & CLKCTL0_SYSPLL0CTL0_BYPASS_MASK) == 0U)
      {
        freq = (uint64_t)freq * 18 / ((CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD1_MASK) >>
                                      CLKCTL0_SYSPLL0PFD_PFD1_SHIFT);
      }
      break;
    case CLKCTL1_DSPCPUCLKSELB_SEL(3): /* RTC 32KHz clock */
        freq = CLK_RTC_32K_CLK;
        break;
    default:
        break;
  }

  SystemCoreClock = freq / ((CLKCTL1->DSPCPUCLKDIV & 0xffU) + 1U);
}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}
