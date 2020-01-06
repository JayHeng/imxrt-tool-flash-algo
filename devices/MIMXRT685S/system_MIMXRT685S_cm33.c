/*
** ###################################################################
**     Processor:           MIMXRT685SEVKA_cm33
**     Compilers:           GNU C Compiler
**                          IAR ANSI C/C++ Compiler for ARM
**                          Keil ARM C/C++ Compiler
**                          MCUXpresso Compiler
**
**     Reference manual:    MIMXRT685 User manual Rev. 0.1 18 July 2017
**     Version:             rev. 1.0, 2018-06-19
**     Build:               b190702
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
 * @file MIMXRT685S_cm33
 * @version 1.0
 * @date 2018-06-19
 * @brief Device specific configuration file for MIMXRT685S_cm33 (implementation
 *        file)
 *
 * Provides a system configuration function and a global variable that contains
 * the system frequency. It configures the device and initializes the oscillator
 * (PLL) that is part of the microcontroller device.
 */

#include <stdint.h>
#include "fsl_device_registers.h"



/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;

/* ----------------------------------------------------------------------------
   -- SystemInit()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInit (void) {
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

  SCB->CPACR |= ((3UL << 0*2) | (3UL << 1*2));    /* set CP0, CP1 Full Access (enable PowerQuad) */

  SCB->NSACR |= ((3UL << 0) | (3UL << 10));   /* enable CP0, CP1, CP10, CP11 Non-secure Access */
/* iMXRT6xx systemInit */
  uint32_t i;

  /* Disable Systick which might be enabled by bootrom */
  if (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk)
  {
      SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  }
  /* Clear NVIC which might be set by bootrom */
  for (i = 0; i < ((NUMBER_OF_INT_VECTORS - 16) + 31) / 32; i++)
  {
      NVIC->ICER[i] = 0xFFFFFFFFUL;
  }

  SystemInitHook();
}

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {

  /* iMXRT6xx systemCoreClockUpdate */
  uint32_t freq = 0U;
  uint64_t freqTmp = 0U;

  switch ((CLKCTL0->MAINCLKSELB) & CLKCTL0_MAINCLKSELB_SEL_MASK)
  {
    case CLKCTL0_MAINCLKSELB_SEL(0): /* MAINCLKSELA clock */
      switch ((CLKCTL0->MAINCLKSELA) & CLKCTL0_MAINCLKSELA_SEL_MASK)
      {
        case CLKCTL0_MAINCLKSELA_SEL(0): /* FFRO clock (48/60m_irc) divider by 4 */
          freq = 12000000U;
          break;
        case CLKCTL0_MAINCLKSELA_SEL(1): /* External clock (clk_in) */
          freq = 24000000U;
          break;
        case CLKCTL0_MAINCLKSELA_SEL(2): /* Low Power Oscillator Clock (1m_lposc) */
          freq = 1000000U;
          break;
        case CLKCTL0_MAINCLKSELA_SEL(3): /* FFRO clock */
          freq = 48000000U;
          break;
        default:
            break;
      }
      break;
    case CLKCTL0_MAINCLKSELB_SEL(1): /* SFRO clock */
      freq = 16000000U;
      break;
    case CLKCTL0_MAINCLKSELB_SEL(2): /* Main System PLL clock */
      switch ((CLKCTL0->SYSPLL0CLKSEL) & CLKCTL0_SYSPLL0CLKSEL_SEL_MASK)
      {
        case CLKCTL0_SYSPLL0CLKSEL_SEL(0): /* SFRO clock */
          freq = 16000000U;
          break;
        case CLKCTL0_SYSPLL0CLKSEL_SEL(1): /* External clock (clk_in) */
          freq = 24000000U;
          break;
        case CLKCTL0_SYSPLL0CLKSEL_SEL(2): /* FFRO clock (48/60m_irc) divider by 2 */
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
        freq = (uint64_t)freq * 18 /
               ((CLKCTL0->SYSPLL0PFD & CLKCTL0_SYSPLL0PFD_PFD0_MASK) >> CLKCTL0_SYSPLL0PFD_PFD0_SHIFT);
      }
      break;

    case CLKCTL0_MAINCLKSELB_SEL(3): /* RTC 32KHz clock */
        freq = 32768U;
        break;

    default:
        break;
  }

  SystemCoreClock = freq / ((CLKCTL0->SYSCPUAHBCLKDIV & 0xffU) + 1U);

}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}
