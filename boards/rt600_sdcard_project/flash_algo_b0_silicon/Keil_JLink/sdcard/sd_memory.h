/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _SD_MEMORY_H_
#define _SD_MEMORY_H_

#include "memory.h"
#include "fsl_sd.h"
#include "fsl_sdmmc_host.h"

/*!
 * @addtogroup sdcard
 * @{
 */

/******************************************************************************
 * Definitions.
 *****************************************************************************/

/*! @brief Configuration structure used for SD memory. */
typedef struct _sd_config
{
    union {
        struct
        {
            uint32_t instance : 4;
            uint32_t rsv0 : 4;
            uint32_t bus_width : 1;
            uint32_t tuningStart : 3;
            uint32_t timing_interface : 3;
            uint32_t rsv1 : 4;
            uint32_t enablePowerCycle : 1;
            uint32_t powerUpTime : 1;
            uint32_t tuningStep : 2;
            uint32_t powerPolarity : 1;
            uint32_t powerDownTime : 2;
            uint32_t rsv2 : 2;
            uint32_t tag : 4;
        } B;
        uint32_t U;
    } word0;
} sd_config_t;

/*! @brief Context structure used for SD memory. */
typedef struct _sd_mem_context
{
    sd_card_t sd;
    bool isConfigured;
    bool isReadBufferValid;
    uint32_t readBufferBlockAddr;
    bool isWriteBufferValid;
    uint32_t writeBufferOffset;
    uint32_t writeBufferBlockAddr;
} sd_mem_context_t;

/*******************************************************************************
 * Externs
 ******************************************************************************/
/* @brief Context variable used for SD memory */
extern sd_mem_context_t g_sdContext;

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
 * @brief SD card initialization function.
 *
 * Init the SD card on a specific host controller.
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
status_t sd_mem_init(void);

/*!
 * @brief SD card re-initialization function.
 *
 * Re-init the SD card on a specific host controller.
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
status_t sd_mem_config(uint32_t *config);

/*!
 * @brief Reads data from the SD card.
 *
 * This function reads data from the SD card to specific destination.
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
 * @retval kStatusMemoryNotConfigured SD card not initialized.
 * @retval kStatus_Success Operate successfully.
 */
status_t sd_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer);

/*!
 * @brief Writes data to the SD card.
 *
 * This function writes Data to the SD card.
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
 * @retval kStatusMemoryNotConfigured SD card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected SD card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t sd_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer);

/*!
 * @brief Flush the cached data to the SD card.
 *
 * This function writes one block data to the SD card.
 *
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatusMemoryNotConfigured SD card not initialized.
 * @retval kStatusMemoryAlignmentError address is not block boundary aligned.
 * @retval kStatusMemoryVerifyFailed Write operation is failed at write verification.
 * @retval kStatusMemoryWriteProtected SD card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t sd_mem_flush(void);

/*!
 * @brief Finalize the write or read operation of the SD card.
 *
 * This function will finalize the write or read progress, reset the state machine.
 *
 * @retval kStatus_Success Operate successfully.
 */
status_t sd_mem_finalize(void);

/*!
 * @brief Erases blocks of the SD card.
 *
 * This function erases blocks of the specific card.
 *
 * @param address The start address.
 * @param length The number of blocks to erase.
 * @retval kStatus_InvalidArgument Invalid argument.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatusMemoryNotConfigured SD card not initialized.
 * @retval kStatusMemoryVerifyFailed Erase operation is failed at Erase verification.
 * @retval kStatusMemoryWriteProtected SD card is read-only.
 * @retval kStatus_Success Operate successfully.
 */
status_t sd_mem_erase(uint32_t address, uint32_t length);

/*!
 * @brief Get the configuration of the SD card.
 *
 * This function gets the configuration of the SD card.
 *
 * @param card The sd card structure.

 * @retval kStatus_Fail Operate failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t get_sd_configuration(sd_card_t *card);

/*!
 * @brief Get the default configuration of the SD card.
 *
 * This function gets the default configuration of the SD card.
 *
 * @param card The sd card structure.

 * @retval kStatus_Fail Operate failed.
 * @retval kStatus_Success Operate successfully.
 */
status_t get_sd_default_configuration(sd_card_t *card);

#if defined(__cplusplus)
}
#endif
/*! @} */

#endif /* _SD_MEMORY_H_*/
