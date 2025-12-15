/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2018 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#if !defined(__SDMMC_INIT_H__)
#define __SDMMC_INIT_H__

#include "bootloader_common.h"
#include "fsl_sdmmc_spec.h"

/******************************************************************************
 * Definitions.
 *****************************************************************************/
enum
{
    kSDMMC_PWR_DOWN_20MS = 0,
    kSDMMC_PWR_DOWN_10MS = 1,
    kSDMMC_PWR_DOWN_5MS = 2,
    kSDMMC_PWR_DOWN_2D5MS = 3,

    kSDMMC_PWR_UP_5MS = 0,
    kSDMMC_PWR_UP_2D5MS = 1,
};

////////////////////////////////////////////////////////////////////////////////
// Basic Definitions
////////////////////////////////////////////////////////////////////////////////
/*! @brief USDHC ADMA table entries */
/* The max image size supported by fast boot is (ENTRIES - 9) * (64K-3) + Initial Load Size(512byte) + MAC Size(32byte)
 * + KeyStore Size(1592byte). for next0, it is 15.43MB */
/*
 * ADMA discriptor table for fast boot:
 *  <kSdmmcMem_ImageStartOffset><4byte><kSdmmcMem_InitialImageSize><4byte>[[<MAC><4byte>][<KeyStore><4byte>]]<The left
 * image><4byte>
 */
/* The max image size supported by normal boot is (ENTRIES - 3) * (64K-3) + Initial Load Size(512byte) + MAC
 * Size(32byte) + KeyStore Size(1592byte). for next0, it is 15.81MB */
/*
 * ADMA discriptor table for normal boot:
 *  <kSdmmcMem_InitialImageSize>[<MAC>][<KeyStore>]<The left image>
 */
#define BOARD_USDHC_ADMA_TABLE_MAX_ENTRIES (256)
/*! @brief USDHC base address. */
#define BOARD_USDHC0_BASEADDR USDHC0
#define BOARD_USDHC1_BASEADDR USDHC1
/*! @brief USDHC frequency. */
#if BL_TARGET_FPGA
#define BOARD_USDHC0_CLK_FREQ SystemCoreClock
#define BOARD_USDHC1_CLK_FREQ SystemCoreClock
#else
#define BOARD_USDHC0_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Sdio0Clk))
#define BOARD_USDHC1_CLK_FREQ (CLOCK_GetFreq(kCLOCK_Sdio1Clk))
#endif
/*! @brief USDHC pin port. */
#define BOARD_USDHC0_CMD_IOPAD IOPCTL, 1, 30, 1
#define BOARD_USDHC0_CLK_IOPAD IOPCTL, 1, 31, 1
#define BOARD_USDHC0_DATA0_IOPAD IOPCTL, 2, 0, 1
#define BOARD_USDHC0_DATA1_IOPAD IOPCTL, 2, 1, 1
#define BOARD_USDHC0_DATA2_IOPAD IOPCTL, 2, 2, 1
#define BOARD_USDHC0_DATA3_IOPAD IOPCTL, 2, 3, 1
#define BOARD_USDHC0_DATA4_IOPAD IOPCTL, 2, 5, 1
#define BOARD_USDHC0_DATA5_IOPAD IOPCTL, 2, 6, 1
#define BOARD_USDHC0_DATA6_IOPAD IOPCTL, 2, 7, 1
#define BOARD_USDHC0_DATA7_IOPAD IOPCTL, 2, 8, 1
#define BOARD_USDHC0_VSELECT_IOPAD IOPCTL, 2, 11, 1
#define BOARD_USDHC0_RESET_B_IOPAD IOPCTL, 2, 10, 0 /*GPIO*/
#define BOARD_USDHC0_RESET_B_PORT GPIO, 2
#define BOARD_USDHC0_RESET_B_GPIO BOARD_USDHC0_RESET_B_PORT, 10

#define BOARD_USDHC1_CMD_IOPAD IOPCTL, 3, 8, 1
#define BOARD_USDHC1_CLK_IOPAD IOPCTL, 3, 9, 1
#define BOARD_USDHC1_DATA0_IOPAD IOPCTL, 3, 10, 1
#define BOARD_USDHC1_DATA1_IOPAD IOPCTL, 3, 11, 1
#define BOARD_USDHC1_DATA2_IOPAD IOPCTL, 3, 12, 1
#define BOARD_USDHC1_DATA3_IOPAD IOPCTL, 3, 13, 1
#define BOARD_USDHC1_DATA4_IOPAD IOPCTL, 3, 15, 1
#define BOARD_USDHC1_DATA5_IOPAD IOPCTL, 3, 16, 1
#define BOARD_USDHC1_DATA6_IOPAD IOPCTL, 3, 17, 1
#define BOARD_USDHC1_DATA7_IOPAD IOPCTL, 3, 18, 1
#define BOARD_USDHC1_VSELECT_IOPAD IOPCTL, 3, 21, 1
#define BOARD_USDHC1_RESET_B_IOPAD IOPCTL, 3, 20, 0 /*GPIO*/
#define BOARD_USDHC1_RESET_B_PORT GPIO, 3
#define BOARD_USDHC1_RESET_B_GPIO BOARD_USDHC1_RESET_B_PORT, 20

#define USDHC0_DriverIRQHandler SDIO0_IRQHandler
#define USDHC1_DriverIRQHandler SDIO1_IRQHandler
////////////////////////////////////////////////////////////////////////////////
// Basic FUNC Definitions
////////////////////////////////////////////////////////////////////////////////
void usdhc_power_control_init(USDHC_Type *base);
void usdhc_power_control(USDHC_Type *base, bool state);
void usdhc_vselect_init(USDHC_Type *base);
void mmc_pinmux_config(USDHC_Type *base, mmc_data_bus_width_t busWidth);
void sd_pinmux_config(USDHC_Type *base, sd_data_bus_width_t busWidth);
////////////////////////////////////////////////////////////////////////////////
// Board FUNC Definitions
////////////////////////////////////////////////////////////////////////////////
/*! @brief Power control init function. */
#define BOARD_USDHC_POWER_CONTROL_INIT(base) usdhc_power_control_init(base)
/*! @brief Power control enable/disable function. */
#define BOARD_USDHC_POWER_CONTROL(base, state) usdhc_power_control(base, state)

/*! @brief vselect function. */
#define BOARD_USDHC_VSELECT_INIT(base) usdhc_vselect_init(base)

#define BOARD_USDHC_SDCARD_POWER_CONTROL_INIT()
#define BOARD_USDHC_SDCARD_POWER_CONTROL(state)
#define BOARD_USDHC_SDCARD_RESET_CONTROL_INIT(base) BOARD_USDHC_POWER_CONTROL_INIT(base)
#define BOARD_USDHC_SDCARD_RESET_CONTROL(base, enable) BOARD_USDHC_POWER_CONTROL(base, enable)
#define BOARD_USDHC_SDCARD_VSELECT_INIT(base) BOARD_USDHC_VSELECT_INIT(base)
#define BOARD_SD_MUX_CONFIG(base, busWidth)  sd_pinmux_config(base, busWidth)
#define BOARD_SD_PIN_CONFIG(speed, strength)
#define BOARD_SD_IO_UPDATE(base, busWidth, speed, strength)
#define BOARD_USDHC_CD_GPIO_INIT()
#define BOARD_USDHC_CD_STATUS() (0)

/*! @brief MMC power control init function. Not used for ROM */
#define BOARD_USDHC_MMCCARD_POWER_CONTROL_INIT()
/*! @brief MMC power enable/disable function. Not used for ROM */
#define BOARD_USDHC_MMCCARD_POWER_CONTROL(state)

/*! @brief MMC power control init function. Used for ROM */
#define BOARD_USDHC_MMCCARD_RESET_CONTROL_INIT(base) BOARD_USDHC_POWER_CONTROL_INIT(base)

/*! @brief MMC power enable/disable function. Used for ROM */
#define BOARD_USDHC_MMCCARD_RESET_CONTROL(base, state) BOARD_USDHC_POWER_CONTROL(base, state)

/*! @brief MMC vselect init function. */
#define BOARD_USDHC_MMCCARD_VSELECT_INIT(base) BOARD_USDHC_VSELECT_INIT(base)

/*! @brief MMC MUX/PAD config function. */
#define BOARD_MMC_MUX_CONFIG(base, busWidth) mmc_pinmux_config(base, busWidth)

/*! @brief MMC config fucntion for KSDK, not used for ROM */
#define BOARD_MMC_PIN_CONFIG(speed, strength)
/*! @brief Define for MMC config IO driver strength dynamic */
#define BOARD_MMC_IO_UPDATE(base, busWidth, speed, strength) /* No need to update for LPC NEXT0 */

#endif // __SDMMC_INIT_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
