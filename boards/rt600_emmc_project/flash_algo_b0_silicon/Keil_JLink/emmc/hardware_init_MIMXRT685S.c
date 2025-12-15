/*
 * Copyright 2017-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "fsl_reset.h"
#include "fsl_clock.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
enum
{
    kSize_32KB = 32u * 1024u,
    kSize_64KB = 64u * 1024u,
    kSize_128KB = 128u * 1024u,
    kSize_256KB = 256u * 1024u
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

void pmc_apply_cfg(void);

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

void pmc_apply_cfg(void)
{
    // Apply PMC change and while until the FSM is idle
    PMC->CTRL |= PMC_CTRL_APPLYCFG_MASK;            // Apply updated PMC PDRUNCFGbits (RAM power gates).
    while (PMC->STATUS & PMC_STATUS_ACTIVEFSM_MASK) // Wait until all the PMC finite state machines are idle
    {
    }
}

#define SYSCTL0_PERICFGENABLE1_SDIO0_EN_MASK     (0x4U)
#define SYSCTL0_PERICFGENABLE1_SDIO1_EN_MASK     (0x8U)

void init_hardware(void)
{
    // Power on eMMC RAM as needed
    if (SYSCTL0->PERICFGENABLE1 & SYSCTL0_PERICFGENABLE1_SDIO0_EN_MASK)
    {
        SYSCTL0->PDRUNCFG1_CLR = (SYSCTL0_PDRUNCFG1_USDHC0_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USDHC0_SRAM_PPD_MASK);
    }

    if (SYSCTL0->PERICFGENABLE1 & SYSCTL0_PERICFGENABLE1_SDIO1_EN_MASK)
    {
        SYSCTL0->PDRUNCFG1_CLR = (SYSCTL0_PDRUNCFG1_USDHC1_SRAM_APD_MASK | SYSCTL0_PDRUNCFG1_USDHC1_SRAM_PPD_MASK);
    }

    // Apply PMC change
    pmc_apply_cfg();

    // Configure clock here because the configure_clocks has been removed from bl_main.c
    configure_clocks(kClockOption_EnterBootloader);
}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
