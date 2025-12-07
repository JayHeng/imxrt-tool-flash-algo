/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include "bootloader_common.h"
#include "property.h"
#include "mmc_memory.h"

//! @addtogroup bl_core
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#define EMMC_RW_SELFTEST (1)

#define BOARD_RT1170_CUSTOMER_2nd_USDHC_FEMDME008G    (0)
#define BOARD_RT600_NXPVAL_1st_USDHC_THGBMNG5D1LBAIT  (0)
#define BOARD_RT600_CUSTOMER_1st_USDHC_MX52LM04A11    (1)

#if BOARD_RT1170_CUSTOMER_2nd_USDHC_FEMDME008G
#define MMC_CFG_OPTION0  (0xc0001200)
#define MMC_CFG_OPTION1  (0x00040002)
#define APP_EXEC_START   (0x2000)
#define APP_LENGTH       (0x6000)
#elif BOARD_RT600_NXPVAL_1st_USDHC_THGBMNG5D1LBAIT
#define MMC_CFG_OPTION0  (0xC0010100)
#define MMC_CFG_OPTION1  (0x00000000)
#define APP_EXEC_START   (0x80000)
#define APP_LENGTH       (0x6000)
#elif BOARD_RT600_CUSTOMER_1st_USDHC_MX52LM04A11
#define MMC_CFG_OPTION0  (0xC0010100)
#define MMC_CFG_OPTION1  (0x00000000)
#define APP_EXEC_START   (0x80000)
#define APP_LENGTH       (0x6000)
#endif

#define APP_EMMC_START  (0x80000)

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

static void bootloader_var_init(void)
{
    g_externalMemoryMap[0].memoryId = kMemoryMMCCard;
    g_externalMemoryMap[0].status = kStatus_Success;
    g_externalMemoryMap[0].basicUnitCount = 0;
    g_externalMemoryMap[0].basicUnitSize = 512;
    g_externalMemoryMap[0].memoryInterface = &g_mmcMemoryInterface;
}


//! @brief Initialize the bootloader and peripherals.
//!
//! This function initializes hardware and clocks, loads user configuration data, and initialzes
//! a number of drivers. It then enters the active peripheral detection phase by calling
//! get_active_peripheral(). Once the peripheral is detected, the packet and comand interfaces
//! are initialized.
//!
//! Note that this routine may not return if peripheral detection times out and the bootloader
//! jumps directly to the user application in flash.
void bootloader_init(void)
{
    bootloader_var_init();

    // Init pinmux and other hardware setup.
    init_hardware();

    // Configure clocks.
    configure_clocks(kClockOption_EnterBootloader);

    // Start the lifetime counter
    microseconds_init();
    
    g_mmcMemoryInterface.init();
}

#if EMMC_RW_SELFTEST
uint8_t s_emmcTestBuffer[512];
#endif

void bootloader_run(void)
{
    status_t status = kStatus_InvalidArgument;
    
    mmc_config_t mmcConfig = 
    {
       .word0.U = MMC_CFG_OPTION0,
       .word1.U = MMC_CFG_OPTION1,
    };

    status = g_mmcMemoryInterface.config((uint32_t *)&mmcConfig);
    if (status == kStatus_Success)
    {
#if EMMC_RW_SELFTEST
        for (uint32_t i = 0; i < sizeof(s_emmcTestBuffer); i++)
        {
            s_emmcTestBuffer[i] = i & 0xFF;
        }
        status = g_mmcMemoryInterface.write(APP_EMMC_START, sizeof(s_emmcTestBuffer), s_emmcTestBuffer);
#endif
        status = g_mmcMemoryInterface.read(APP_EMMC_START, APP_LENGTH, (uint8_t *)APP_EXEC_START);
        if (status == kStatus_Success)
        {
            __NOP();
        }
    }

    while (1)
    {

    }
}

//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
