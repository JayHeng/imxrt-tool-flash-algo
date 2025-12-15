/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "bootloader_common.h"
#include "fsl_clock.h"
#include "fsl_device_registers.h"
#include "memory.h"
#include "property.h"
#include "sd_memory.h"

/*******************************************************************************
 * Definitons
 ******************************************************************************/
/*! @brief The common value */
enum
{
    kSDCardErasedPattern0 = 0x00000000, /*!< The default value of SD memory bits. */
    kSDCardErasedPattern1 = 0xFFFFFFFF, /*!< The default value of SD memory bits when DATA_STAT_AFTER_ERASE is set */
    kSDStartAddress = 0x0,              /*!< The default start address of SD Card. */
#if SDMMCHOST_DMA_BUFFER_ADDR_ALIGN == USDHC_ADMA1_ADDRESS_ALIGN /*!< ADMA1 is not really supported yet */
    kSDBufferBlockCount = 8,                                     /*!< The SD memory buffer size in block count. */
#elif SDMMCHOST_DMA_BUFFER_ADDR_ALIGN == USDHC_ADMA2_ADDRESS_ALIGN
    kSDBufferBlockCount = 1, /*!< The SD memory buffer size in block count. */
#else
#error Invalid USDHC DMA selection.
#endif
    kSDBufferSizeInBytes = FSL_SDMMC_DEFAULT_BLOCK_SIZE * kSDBufferBlockCount, /*!< The SD memory buffer size. */

    kSDConfigTag = 0xD,                          /*!< The SD config block tag. */
    kSDSize_Max4GB_InKbytes = (4 * 1024 * 1024), /* 4GB space in KBytes */
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Load one block data to the read buffer.
 *
 * @param blockAddr The block address to read.
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t sd_mem_load_buffer(uint32_t blockAddr);
/*!
 * @brief Flush one block data to SD
 * @retval kStatus_SDMMC_CardNotSupport Card not support.
 * @retval kStatus_SDMMC_NotSupportYet Not support now.
 * @retval kStatus_SDMMC_WaitWriteCompleteFailed Send status failed.
 * @retval kStatus_SDMMC_TransferFailed Transfer failed.
 * @retval kStatus_SDMMC_StopTransmissionFailed Stop transmission failed.
 * @retval kStatus_Success Operate successfully.
 */
static status_t sd_mem_flush_buffer(void);
/*!
 * @brief Check if the specific memory is erased or not.
 *
 * @retval true Erased.
 * @retval false Not erased.
 */
static bool is_erased_memory(uint32_t blockAddr, uint32_t blockCount, uint32_t erasedPattern);
/*!
 * @brief Check if SD card is initialized successfully or not.
 *
 * @retval true Initialized.
 * @retval false Un-initialized.
 */
static bool is_sd_configured(void);
/*!
 * @brief Check if the block to read is already cached in the buffer.
 *
 * @retval true Cached.
 * @retval false Not cached.
 */
static bool is_read_block_cached(uint32_t blockAddr);
/*!
 * @brief Check if the block to write is already cached in the buffer.
 *
 * @retval true Cached.
 * @retval false Not cached.
 */
static bool is_write_block_cached(uint32_t blockAddr);

#if BL_FEATURE_GEN_KEYBLOB
/*!
 * @brief Check and update keyblob info as needed.
 */
static status_t check_update_keyblob_info(void *config);
#endif // BL_FEATURE_GEN_KEYBLOB

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* @brief Context variable used for SD memory */
sd_mem_context_t g_sdContext = { 0 };

/* @brief Buffer used for SD memory */
BL_ALIGN(SDMMCHOST_DMA_BUFFER_ADDR_ALIGN) static uint8_t s_sd_mem_readBuffer[kSDBufferSizeInBytes] = { 0 };
BL_ALIGN(SDMMCHOST_DMA_BUFFER_ADDR_ALIGN) static uint8_t s_sd_mem_writeBuffer[kSDBufferSizeInBytes] = { 0 };

/* @brief Interface to spi nand memory operations */
const external_memory_region_interface_t g_sdMemoryInterface = {
    .init = sd_mem_init,
    .read = sd_mem_read,
    .write = sd_mem_write,
    .erase = sd_mem_erase,
    .config = sd_mem_config,
    .flush = sd_mem_flush,
    .finalize = sd_mem_finalize,
    .erase_all = sd_mem_erase_all,
};

/*******************************************************************************
 * Code
 ******************************************************************************/

status_t sd_mem_init(void)
{
    status_t status = kStatus_Success;
    sd_card_t *card = &g_sdContext.sd;

    status = get_sd_default_configuration(card);
    if (status != kStatus_Success)
    {
        // get the default configuration failed. Must init the mmc by configure memory command.
        return status;
    }

    status = SD_BL_Init(card);
    if (status == kStatus_Success)
    {
        // Note: blockSize doesn't need to be checked here.
        // Driver uses multi-block read/write.
        // Then the block size will always be 512 bytes during read/write operation.
        // if ((card->blockSize > kSDBufferSizeInBytes) || ((kSDBufferSizeInBytes % card->blockSize) != 0))
        //{
        //    return kStatusMemoryNotSupported;
        //}

        // Update external map entry info.
        uint32_t index;
        status = find_external_map_index(kMemorySDCard, &index);
        if (status != kStatus_Success)
        {
            return status;
        }
        g_externalMemoryMap[index].basicUnitSize = card->blockSize;
        g_externalMemoryMap[index].basicUnitCount = card->blockCount;

        // Once initialization is succeed, SD card is accessable.
        g_sdContext.isConfigured = true;
    }
    else
    {
        g_sdContext.isConfigured = false;
    }

    return status;
}

status_t sd_mem_config(uint32_t *config)
{
    status_t status = kStatus_Fail;

    const sd_config_t *sdConfig = (const sd_config_t *)config;

    // Check the tag.
    if (sdConfig->word0.B.tag != kSDConfigTag)
    {
        return kStatus_InvalidArgument;
    }

    // Clear the Context.
    memset(&g_sdContext, 0, sizeof(sd_mem_context_t));

    sd_card_t *card = &g_sdContext.sd;

    switch (sdConfig->word0.B.instance)
    {
#if defined(BOARD_USDHC0_BASEADDR)
        case 0:
            card->host.base = BOARD_USDHC0_BASEADDR;
            card->host.sourceClock_Hz = BOARD_USDHC0_CLK_FREQ;
            break;
#else
        // it means the instance starts from index 1, if instance 0 is not defined,
        case 0:
#endif // #if defined(BOARD_USDHC0_BASEADDR)
#if defined(BOARD_USDHC1_BASEADDR)
        case 1:
            card->host.base = BOARD_USDHC1_BASEADDR;
            card->host.sourceClock_Hz = BOARD_USDHC1_CLK_FREQ;
            break;
#endif // #if defined(BOARD_USDHC1_BASEADDR)
#if defined(BOARD_USDHC2_BASEADDR)
        case 2:
            card->host.base = BOARD_USDHC2_BASEADDR;
            card->host.sourceClock_Hz = BOARD_USDHC2_CLK_FREQ;
            break;
#endif // #if defined(BOARD_USDHC2_BASEADDR)
        default:
            return kStatus_InvalidArgument;
    }

    card->userConfig.timing = (sd_timing_mode_t)sdConfig->word0.B.timing_interface;
    card->userConfig.busWidth = (sd_data_bus_width_t)sdConfig->word0.B.bus_width;
    card->userConfig.enablePowerCycle = sdConfig->word0.B.enablePowerCycle;
    card->userConfig.powerPolarity = sdConfig->word0.B.powerPolarity;
    switch (sdConfig->word0.B.powerDownTime)
    {
        default:
        case kSDMMC_PWR_DOWN_20MS:
            card->userConfig.powerDownDelay_US = 20 * 1000;
            break;
        case kSDMMC_PWR_DOWN_10MS:
            card->userConfig.powerDownDelay_US = 10 * 1000;
            break;
        case kSDMMC_PWR_DOWN_5MS:
            card->userConfig.powerDownDelay_US = 5 * 1000;
            break;
        case kSDMMC_PWR_DOWN_2D5MS:
            card->userConfig.powerDownDelay_US = 25 * 100;
            break;
    }
    switch (sdConfig->word0.B.powerUpTime)
    {
        default:
        case kSDMMC_PWR_UP_5MS:
            card->userConfig.powerUpDelay_US = 5 * 1000;
            break;
        case kSDMMC_PWR_UP_2D5MS:
            card->userConfig.powerUpDelay_US = 25 * 100;
            break;
    }

    card->userConfig.tuningStart = sdConfig->word0.B.tuningStart * 32;
    card->userConfig.tuningStep = sdConfig->word0.B.tuningStep ? (sdConfig->word0.B.tuningStep * 2) : 1;

    status = SD_BL_Init(card);
    if (status == kStatus_Success)
    {
        // Note: blockSize doesn't need to be checked here.
        // Driver uses multi-block read/write.
        // Then the block size will always be 512 bytes during read/write operation.
        // if ((card->blockSize > kSDBufferSizeInBytes) || ((kSDBufferSizeInBytes % card->blockSize) != 0))
        //{
        //    return kStatusMemoryNotSupported;
        //}

        // Update external map entry info.
        uint32_t index;
        status = find_external_map_index(kMemorySDCard, &index);
        if (status != kStatus_Success)
        {
            return status;
        }
        g_externalMemoryMap[index].basicUnitSize = card->blockSize;
        g_externalMemoryMap[index].basicUnitCount = card->blockCount;

        // Once initialization is succeed, SD card is accessable.
        g_sdContext.isConfigured = true;
    }
    else
    {
        g_sdContext.isConfigured = false;
    }

    return status;
}

status_t sd_mem_read(uint32_t address, uint32_t length, uint8_t *restrict buffer)
{
    assert(buffer);

    status_t status = kStatus_Success;

    // SD should be initialized before access.
    if (!is_sd_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    uint32_t bufferSize = kSDBufferSizeInBytes; // Use CPU reg.
    uint32_t bufferOffset = address % bufferSize;
    uint32_t blockAddr = address / FSL_SDMMC_DEFAULT_BLOCK_SIZE; // Not use "card->blockSize" here.

    uint32_t readLength;

    while (length)
    {
        // Check if current block to read is already read to readbuffer.
        // If no, need to read the whole block to buffer.
        if (!is_read_block_cached(blockAddr))
        {
            // Read the page to buffer.
            status = sd_mem_load_buffer(blockAddr);
            if (status != kStatus_Success)
            {
                return status;
            }
        }
        else
        {
            // If the block is cached in the buffer, then need to change the buffer offset.
            // Buffer might has multi blocks. And the dest might not locate at the first block.
            bufferOffset = address - g_sdContext.readBufferBlockAddr * FSL_SDMMC_DEFAULT_BLOCK_SIZE;
        }

        // If it is a read accoss the block, divide it into two steps.
        if ((bufferOffset + length) <= bufferSize)
        {
            readLength = length;
        }
        else
        {
            readLength = bufferSize - bufferOffset;
        }
        memcpy(buffer, &s_sd_mem_readBuffer[bufferOffset], readLength);
        length -= readLength;
        buffer += readLength;
        bufferOffset += readLength;
        if (bufferOffset >= bufferSize)
        {
            bufferOffset -= bufferSize;
            blockAddr += kSDBufferBlockCount;
        }
    }

    return kStatus_Success;
}

status_t sd_mem_write(uint32_t address, uint32_t length, const uint8_t *buffer)
{
    assert(buffer);

    status_t status = kStatus_Success;

    // SD should be initialized before access.
    if (!is_sd_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    uint32_t bufferSize = kSDBufferSizeInBytes; // Use CPU reg.
    uint32_t bufferOffset = address % bufferSize;
    uint32_t blockAddr = address / FSL_SDMMC_DEFAULT_BLOCK_SIZE; // Not use "card->blockSize" here.

    uint32_t writeLength;

    while (length)
    {
        // Check if current block to write is already cached to writebuffer.
        // If no, need to init the writebuffer.
        if (!is_write_block_cached(blockAddr))
        {
            // There is data already cached in the buffer, flush it to SD.
            if (g_sdContext.isWriteBufferValid)
            {
                status = sd_mem_flush_buffer();
                if (status != kStatus_Success)
                {
                    return status;
                }
            }

            // Start a new block write. The address must block size aligned.
            if (bufferOffset != 0)
            {
                return kStatusMemoryAlignmentError;
            }
            g_sdContext.writeBufferOffset = bufferOffset;
            g_sdContext.writeBufferBlockAddr = blockAddr;
            g_sdContext.isWriteBufferValid = true;
        }
        else
        {
            // If the block is cached in the buffer, then need to change the buffer offset.
            // Buffer might has multi blocks. And the dest might not locate at the first block.
            bufferOffset = address - g_sdContext.writeBufferBlockAddr * FSL_SDMMC_DEFAULT_BLOCK_SIZE;
        }

        // If the address is not continuous, start a new block write.
        if (g_sdContext.writeBufferOffset != bufferOffset)
        {
            status = sd_mem_flush_buffer();
            if (status != kStatus_Success)
            {
                return status;
            }
            continue;
        }

        if (bufferOffset + length <= bufferSize)
        {
            writeLength = length;
        }
        else
        {
            writeLength = bufferSize - bufferOffset;
        }
        memcpy(&s_sd_mem_writeBuffer[bufferOffset], buffer, writeLength);
        g_sdContext.writeBufferOffset += writeLength;
        length -= writeLength;
        buffer += writeLength;
        bufferOffset += writeLength;
        if (bufferOffset >= bufferSize)
        {
            status = sd_mem_flush_buffer();
            if (status != kStatus_Success)
            {
                return status;
            }

            bufferOffset -= bufferSize;
            blockAddr += kSDBufferBlockCount;
        }
    }

    return kStatus_Success;
}
status_t sd_mem_flush(void)
{
    status_t status = kStatus_Success;
    // If there still is data in the buffer, then flush them to SD.
    if (g_sdContext.isWriteBufferValid)
    {
        status = sd_mem_flush_buffer();
    }
    return status;
}
status_t sd_mem_finalize(void)
{
    // Mark buffer to invalid.
    g_sdContext.isWriteBufferValid = false;
    g_sdContext.isReadBufferValid = false;

    return kStatus_Success;
}
status_t sd_mem_erase(uint32_t address, uint32_t length)
{
    status_t status;
    sd_card_t *card = &g_sdContext.sd;

    // SD should be initialized before access.
    if (!is_sd_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    if (SD_CheckReadOnly(card))
    {
        return kStatusMemoryWriteProtected;
    }

    // length = 0 means no erase operation will be executed. Just return success.
    if (length == 0)
    {
        return kStatus_Success;
    }
    // else means 1 block at lest to be erased.

    uint32_t startBlockAddr = address / FSL_SDMMC_DEFAULT_BLOCK_SIZE; // Not use "card->blockSize" here.
    // Don't get block count from length. Address to address + length might across block boundary.
    uint32_t blockCount = (address + length - 1) / FSL_SDMMC_DEFAULT_BLOCK_SIZE - startBlockAddr + 1;

    status = SD_EraseBlocks(card, startBlockAddr, blockCount);
    if (status != kStatus_Success)
    {
        return status;
    }
#if 0
    // Indeed, no need to check if the memory is erased. Card will guarantee it.
    if (!is_erased_memory(startBlockAddr, blockCount, (card->scr.flags & kSD_ScrDataStatusAfterErase) ?
                                                          kSDCardErasedPattern1 :
                                                          kSDCardErasedPattern0))
    {
        return kStatusMemoryVerifyFailed;
    }
#endif
    return status;
}
status_t sd_mem_erase_all(void)
{
    status_t status;
    sd_card_t *card = &g_sdContext.sd;
    // SD should be configured before access.
    if (!is_sd_configured())
    {
        return kStatusMemoryNotConfigured;
    }

    if (SD_CheckReadOnly(card))
    {
        return kStatusMemoryWriteProtected;
    }
    status =
        SD_EraseBlocks(card, 0, (uint64_t)card->blockCount * (uint64_t)card->blockSize / FSL_SDMMC_DEFAULT_BLOCK_SIZE);
    if (status != kStatus_Success)
    {
        return status;
    }
#if 0
    // Indeed, no need to check if the memory is erased. Card will guarantee it.
    if (!is_erased_memory(
            0, (uint64_t)card->blockCount * (uint64_t)card->blockSize / FSL_SDMMC_DEFAULT_BLOCK_SIZE,
            (card->scr.flags & kSD_ScrDataStatusAfterErase) ? kSDCardErasedPattern1 : kSDCardErasedPattern0))
    {
        return kStatusMemoryVerifyFailed;
    }
#endif
    return status;
}
status_t sd_get_property(uint32_t whichProperty, uint32_t *value)
{
    sd_card_t *card = &g_sdContext.sd;

    if (value == NULL)
    {
        return kStatus_InvalidArgument;
    }

    switch (whichProperty)
    {
        case kExternalMemoryPropertyTag_InitStatus:
            *value = is_sd_configured() ? kStatus_Success : kStatusMemoryNotConfigured;
            break;

        case kExternalMemoryPropertyTag_StartAddress:
            *value = kSDStartAddress;
            break;

        case kExternalMemoryPropertyTag_MemorySizeInKbytes:
            *value = (uint32_t)((uint64_t)card->blockCount * card->blockSize / 1024);
            break;

        case kExternalMemoryPropertyTag_BlockSize:
            *value = card->blockSize;
            break;

        default: // catch inputs that are not recognized
            return kStatus_InvalidArgument;
    }

    return kStatus_Success;
}

static status_t sd_mem_load_buffer(uint32_t blockAddr)
{
    status_t status;

    g_sdContext.isReadBufferValid = false; // Mark read buffer invalid.

    // Read the page to read buffer.
    status = SD_ReadBlocks(&g_sdContext.sd, s_sd_mem_readBuffer, blockAddr, kSDBufferBlockCount);
    if (status == kStatus_Success)
    {
        g_sdContext.isReadBufferValid = true;
        g_sdContext.readBufferBlockAddr = blockAddr;
    }
    return status;
}

static status_t sd_mem_flush_buffer(void)
{
    status_t status = kStatus_Success;
    sd_card_t *card = &g_sdContext.sd;

    if (SD_CheckReadOnly(card))
    {
        return kStatusMemoryWriteProtected;
    }

    // Fill up the left bytes.
    if (g_sdContext.writeBufferOffset != kSDBufferSizeInBytes)
    {
        memset(&s_sd_mem_writeBuffer[g_sdContext.writeBufferOffset],
               (card->scr.flags & kSD_ScrDataStatusAfterErase) ? kSDCardErasedPattern1 : kSDCardErasedPattern0,
               kSDBufferSizeInBytes - g_sdContext.writeBufferOffset);
    }

    g_sdContext.isWriteBufferValid = false;

#if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE
    if (!is_erased_memory(
            g_sdContext.writeBufferBlockAddr, kSDBufferBlockCount,
            (card->scr.flags & kSD_ScrDataStatusAfterErase) ? kSDCardErasedPattern1 : kSDCardErasedPattern0))
    {
        return kStatusMemoryCumulativeWrite;
    }
#endif // #if BL_FEATURE_FLASH_CHECK_CUMULATIVE_WRITE

    // Flush the data in the write buffer to SD
    status = SD_WriteBlocks(card, s_sd_mem_writeBuffer, g_sdContext.writeBufferBlockAddr, kSDBufferBlockCount);
    if (status != kStatus_Success)
    {
        return status;
    }

    // Indeed, no need to check if the memory is written correct or not. Card will guarantee it.
    // But still do it to make sure user's data written without any incorrect.
    status = SD_ReadBlocks(card, s_sd_mem_readBuffer, g_sdContext.writeBufferBlockAddr, kSDBufferBlockCount);
    if (status != kStatus_Success)
    {
        return status;
    }

    if (memcmp(s_sd_mem_writeBuffer, s_sd_mem_readBuffer, kSDBufferSizeInBytes))
    {
        return kStatusMemoryVerifyFailed;
    }

    return status;
}

static bool is_erased_memory(uint32_t blockAddr, uint32_t blockCount, uint32_t erasedPattern)
{
    status_t status = kStatus_Success;
    sd_card_t *card = &g_sdContext.sd;
    uint32_t offset, *buffer, readblock;
    while (blockCount)
    {
        if (blockCount > kSDBufferBlockCount)
        {
            readblock = kSDBufferBlockCount;
        }
        else
        {
            readblock = blockCount;
        }

        // Read the page firstly.
        status = SD_ReadBlocks(card, s_sd_mem_readBuffer, blockAddr, readblock);
        if (status != kStatus_Success)
        {
            // If read failed, return false.
            return false;
        }

        offset = 0;
        buffer = (uint32_t *)s_sd_mem_readBuffer;
        while (offset < (readblock * FSL_SDMMC_DEFAULT_BLOCK_SIZE))
        {
            // Check if all 0xFFs or all 0x00s
            if (*buffer != erasedPattern)
            {
                return false;
            }
            buffer++;
            offset += 4;
        }
        blockCount -= readblock;
    }
    return true;
}

static bool is_sd_configured(void)
{
    return g_sdContext.isConfigured;
}

static bool is_read_block_cached(uint32_t blockAddr)
{
    return (g_sdContext.isReadBufferValid) && (g_sdContext.readBufferBlockAddr <= blockAddr) &&
           ((g_sdContext.readBufferBlockAddr + kSDBufferBlockCount) > blockAddr);
}

static bool is_write_block_cached(uint32_t blockAddr)
{
    return (g_sdContext.isWriteBufferValid) && (g_sdContext.writeBufferBlockAddr == blockAddr) &&
           ((g_sdContext.writeBufferBlockAddr + kSDBufferBlockCount) > blockAddr);
}

