/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _MMC_MEMORY_H_
#define _MMC_MEMORY_H_

#include "memory.h"
#include "fsl_mmc.h"
#include "fsl_sdmmc_host.h"

/*!
 * @addtogroup mmccard
 * @{
 */

/*******************************************************************************
 * Definitons
 ******************************************************************************/

/*! @brief Configuration structure used for MMC memory. */
typedef struct _mmc_config
{
    union
    {
        struct
        {
            uint32_t boot_config_enable : 1;
            uint32_t rsv0 : 1;
            uint32_t boot_ack : 1;
            uint32_t reset_boot_bus_conditions : 1;
            uint32_t boot_mode : 2;
            uint32_t rsv1 : 2;
            uint32_t bus_width : 4;
            uint32_t timing_interface : 4;
            uint32_t boot_bus_width : 2;
            uint32_t rsv2 : 2;
            uint32_t boot_partition_enable : 3;
            uint32_t rsv3 : 1;
            uint32_t partition_access : 3;
            uint32_t rsv4 : 1;
            uint32_t tag : 4;
        } B;
        uint32_t U;
    } word0;

    union
    {
        struct
        {
            uint32_t instance : 4;
            uint32_t rsv_perm_config_enable : 2;
            uint32_t rsv0 : 10;
            uint32_t rsv_perm_boot_config_prot : 2;
            uint32_t enable1V8 : 1;
            uint32_t enablePowerCycle : 1;
            uint32_t powerUpTime : 1;
            uint32_t rsv1 : 2;
            uint32_t powerPolarity : 1;
            uint32_t powerDownTime : 2;
            uint32_t rsv2 : 2;
            uint32_t rsv_driver_strength : 4;
        } B;
        uint32_t U;
    } word1;
} mmc_config_t;

/*! @brief Context structure used for MMC memory. */
typedef struct _mmc_mem_context
{
    mmc_card_t mmc;
    bool isConfigured;
    bool isReadBufferValid;
    uint32_t readBufferBlockAddr;
    bool isWriteBufferValid;
    uint32_t writeBufferOffset;
    uint32_t writeBufferBlockAddr;
} mmc_mem_context_t;

/*******************************************************************************
 * Externs
 ******************************************************************************/
/* @brief Context variable used for MMC memory */
extern mmc_mem_context_t g_mmcContext;

/*************************************************************************************************
 * API
 ************************************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and deinitialization
 * @{
 */

/*!
 * @brief MMC card initialization function.
 *
 * Init the MMC card on a specific host controller.
 *
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_NotSupportYet Card not support.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SendRelativeAddressFailed Send relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendScrFailed Send SCR failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_init(void);

/*!
 * @brief MMC card re-initialization function.
 *
 * Re-init the MMC card on a specific host controller.
 *
 * @retval kStatus_SDMMC_GoIdleFailed Go idle failed.
 * @retval kStatus_SDMMC_NotSupportYet Card not support.
 * @retval kStatus_SDMMC_SendOperationConditionFailed Send operation condition failed.
 * @retval kStatus_SDMMC_AllSendCidFailed Send CID failed.
 * @retval kStatus_SDMMC_SendRelativeAddressFailed Send relative address failed.
 * @retval kStatus_SDMMC_SendCsdFailed Send CSD failed.
 * @retval kStatus_SDMMC_SelectCardFailed Send SELECT_CARD command failed.
 * @retval kStatus_SDMMC_SendScrFailed Send SCR failed.
 * @retval kStatus_SDMMC_SetBusWidthFailed Set bus width failed.
 * @retval kStatus_SDMMC_SwitchHighSpeedFailed Switch high speed failed.
 * @retval kStatus_SDMMC_SetCardBlockSizeFailed Set card block size failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_config(uint32_t *config);

/*!
 * @brief Reads data from the MMC card.
 *
 * This function reads data from the MMC card to specific destination.
 *
 * @param address Start address to read the data.
 * @param length The number of bytes to read.
 * @param buffer The buffer to save the data read from card.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

/*!
 * @brief Writes data to the MMC card.
 *
 * This function writes Data to the MMC card.
 *
 * @param address Start address to write the data to.
 * @param length The number of bytes to write.
 * @param buffer The buffer to save the data to write to the card.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

/*!
 * @brief Flush the cached data to the MMC card.
 *
 * This function writes one block data to the MMC card.
 *
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_flush(void);

/*!
 * @brief Finalize the write or read operation of the MMC card.
 *
 * This function will finalize the write or read progress, reset the state machine.
 *
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_finalize(void);

/*!
 * @brief Erases blocks of the MMC card.
 *
 * This function erases blocks of the specific card.
 *
 * @param address The start address.
 * @param length The number of blocks to erase.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatusMemoryNotConfigured MMC card not initialized.
 * @retval kStatusMemoryVerifyFailed Erase operation is failed at Erase verification.
 * @retval kStatusMemoryWriteProtected MMC card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t mmc_mem_erase(uint32_t address, uint32_t length);

/*!
 * @brief Check if MMC card fast boot is required.
 *
 * This function gets the enablement of MMC fast boot.
 *
 * @retval True fast boot enabled.
 * @retval False fast boot disabled.
 */
bool is_mmc_fast_boot_enabled(void);

/*!
 * @brief Get the configuration of the MMC card.
 *
 * This function gets the configuration.
 *
 * @param card The mmc card structure.
 *
 * @retval kStatus_Fail Operate failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t get_mmc_configuration(mmc_card_t *card);

/*!
 * @brief Get the default configuration of the MMC card.
 *
 * This function gets the default configuration.
 *
 * @param card The mmc card structure.
 *
 * @retval kStatus_Fail Operate failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t get_mmc_default_configuration(mmc_card_t *card);

#if defined(__cplusplus)
}
#endif
/*! @} */

#endif /* _MMC_MEMORY_H_*/
