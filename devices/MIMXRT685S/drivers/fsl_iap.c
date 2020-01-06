/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "fsl_iap.h"
#include "fsl_device_registers.h"
/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.iap2"
#endif

/*!
 * @addtogroup rom_api
 * @{
*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief OTP driver API Interface */
typedef struct
{
    status_t (*init)(uint32_t src_clk_freq);
    status_t (*deinit)(void);
    status_t (*fuse_read)(uint32_t addr, uint32_t *data);
    status_t (*fuse_program)(uint32_t addr, uint32_t data, bool lock);
    status_t (*crc_calc)(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum);
    status_t (*reload)(void);
    status_t (*crc_check)(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr);
} ocotp_driver_t;

/*!
 * @brief Root of the bootloader API tree.
 *
 * An instance of this struct resides in read-only memory in the bootloader. It
 * provides a user application access to APIs exported by the bootloader.
 *
 * @note The order of existing fields must not be changed.
 */
typedef struct BootloaderTree
{
    void (*runBootloader)(void *arg);                /*!< Function to start the bootloader executing. */
    const uint32_t reserved0;                        /*!< Reserved */
    const char *copyright;                           /*!< Copyright string. */
    const uint32_t *reserved1;                       /*!< Reserved */
    const uint32_t *reserved2;                       /*!< Reserved */
    const uint32_t *reserved3;                       /*!< Reserved */
    const uint32_t *reserved4;                       /*!< Reserved */
    const uint32_t *qspiNorDriver;                   /*!< QuadSPI NOR FLASH Driver API (TBD). */
    const ocotp_driver_t *otpDriver;                 /*!< OTP driver API. */
} bootloader_tree_t;
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ROM_API_TREE ((uint32_t *)0x13001220)
#define BOOTLOADER_API_TREE_POINTER ((bootloader_tree_t *)ROM_API_TREE)
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Global pointer to the otp driver API table in ROM. */
ocotp_driver_t *OTP_API_TREE;
/*! Get pointer to otp driver API table in ROM. */
#define OTP_API_TREE BOOTLOADER_API_TREE_POINTER->otpDriver

/*******************************************************************************
 * OTP driver
 ******************************************************************************/
status_t IAP_OtpInit(uint32_t src_clk_freq)
{
    return OTP_API_TREE->init(src_clk_freq);
}

status_t IAP_OtpDeinit(void)
{
    return OTP_API_TREE->deinit();
}

status_t IAP_OtpFuseRead(uint32_t addr, uint32_t *data)
{
    return OTP_API_TREE->fuse_read(addr, data);
}

status_t IAP_OtpFuseProgram(uint32_t addr, uint32_t data, bool lock)
{
    return OTP_API_TREE->fuse_program(addr, data, lock);
}

status_t IAP_OtpCrcCalc(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum)
{
    return OTP_API_TREE->crc_calc(src, numberOfWords, crcChecksum);
}

status_t IAP_OtpShadowRegisterReload(void)
{
    return OTP_API_TREE->reload();
}

status_t IAP_OtpCrcCheck(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr)
{
    return OTP_API_TREE->crc_check(start_addr, end_addr, crc_addr);
}

