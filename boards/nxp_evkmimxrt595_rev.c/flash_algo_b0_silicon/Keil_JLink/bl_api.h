/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_API_H__
#define __BL_API_H__

#include "api_tree_root.h"
#include "flexspi_nor_flash.h"

/*******************************************************************************
 * Definition
 ******************************************************************************/
//!@brief FLEXSPI ROOT CLOCK soruce related definitions
enum
{
    kFlexSpiClockSrc_MainClk = 0,
    kFlexSpiClockSrc_MainPllClk,
    kFlexSpiClockSrc_Aux0PllClk,
    kFlexSpiClockSrc_FRO192M_Clk,
    kFlexSpiClockSrc_Aux1PllClk,
};


//!@brief FLEXSPI clock configuration - When clock source is PLL
enum
{
    kFlexSpiSerialClk_30MHz = 1,
    kFlexSpiSerialClk_50MHz = 2,
    kFlexSpiSerialClk_60MHz = 3,
    kFlexSpiSerialClk_80MHz = 4,
    kFlexSpiSerialClk_100MHz = 5,
    kFlexSpiSerialClk_120MHz = 6,
    kFlexSpiSerialClk_133MHz = 7,
    kFlexSpiSerialClk_166MHz = 8,
    kFlexSpiSerialClk_200MHz = 9,
};

//!@brief FLEXSPI clock configuration - When clock source is FRO192
enum
{
    kFlexSpiSerialClk_32MHz = 1,
    kFlexSpiSerialClk_48MHz = 2,
    kFlexSpiSerialClk_64MHz = 3,
    kFlexSpiSerialClk_96MHz = 4,
};

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////
//!@brief This API is an RAM function which can be used in the XIP use case
status_t flexspi_nor_auto_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);

#endif // __BL_API_H__