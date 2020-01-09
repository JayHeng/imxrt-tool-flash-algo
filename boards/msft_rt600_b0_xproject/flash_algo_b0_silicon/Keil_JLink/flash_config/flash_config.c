/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "flash_config.h"
#include "platform.h"
#include "FlashW25Q16FW.h"
#include "fsl_flexspi.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
#if defined(BOOT_HEADER_ENABLE) && (BOOT_HEADER_ENABLE == 1)
#if defined(__ARMCC_VERSION) || defined(__GNUC__)
__attribute__((section(".flash_conf")))
#elif defined(__ICCARM__)
#pragma location = ".flash_conf"
#endif

const flexspi_boot_config_t g_flexSpiConfig = {
    .tag                  = FLASH_CONFIG_BLOCK_TAG,
    .version              = FLASH_CONFIG_BLOCK_VERSION,
    .readSampleClkSrc     = 2,
    .csHoldTime           = 3,
    .csSetupTime          = 3,
    .columnAddressWidth   = 0,
    .deviceModeCfgEnable  = 1, // Enable quad mode by sending an initial command to flash
    .deviceModeType       = 0,
    .waitTimeCfgCommands  = 0,
    .deviceModeSeq        = {.seqNum = 0x01,
                             .seqId  = 0x02,
                            },  // Run single command, sequence #2
    .deviceModeArg        = 0x00000002, //Set bit 2 with that command (Write Status Register 2)
    .configCmdEnable      = 0,
    .configModeType       = {0},
    //.configCmdSeqs        = {0},
    .configCmdArgs        = {0},
    .controllerMiscOption = (0),
    .deviceType           = 0x0,
    .sflashPadType        = kSerialFlash_4Pads,
    .serialClkFreq        = kFlexSpiSerialClk_48MHz,
    .lutCustomSeqEnable   = 0,
    .sflashA1Size         = BOARD_FLASH_SIZE,
    .sflashA2Size         = 0,
    .sflashB1Size         = 0,
    .sflashB2Size         = 0,
    .csPadSettingOverride = 0,
    .sclkPadSettingOverride = 0,
    .dataPadSettingOverride = 0,
    .dqsPadSettingOverride  = 0,
    .timeoutInMs            = 0,
    .commandInterval        = 0,
    .busyOffset             = 0,
    .busyBitPolarity        = 0,
    .lookupTable            = {

/*
        // Sequence 0 - Read Data
        // 0x03 - Read Data command, 0x18 - 24 bit address
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS]     =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_READ_DATA,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // 0x80 - read 128 bytes, stop
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS + 1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR,  kFLEXSPI_1PAD, 0x80,
                            kFLEXSPI_Command_STOP,      0x00,          0x00),
*/

        // Sequence 0 - Quad Read
        // 0x6B - Fast read Quad Output command, 0x18 - 24 bit address
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS]     =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_QUAD_READ,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // 0x8 - 8 dummy clocks, 0x80 - read 128 bytes
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS + 1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08,
                            kFLEXSPI_Command_READ_SDR,  kFLEXSPI_4PAD, 0x80),
        // Sequence 1 - Read Status Register
        // 0x05 - Read status register command, 0x4 - read 4 bytes
        [FLEXSPI_READ_STATUS_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_READ_STATUS_REGISTER,
                            kFLEXSPI_Command_READ_SDR,  kFLEXSPI_1PAD, 0x04),
        // Sequence 2 - Write Status Register 2
        // 0x31 - Write status register 2 command, 0x1 - write 1 byte
        [FLEXSPI_WRITE_STATUS_REGISTER_2_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS]  =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_WRITE_STATUS_REGISTER_2,
                            kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x01),
        // Sequence 3 - Write enable
        // 0x06 - Write enable command
        [FLEXSPI_WRITE_ENABLE_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_WRITE_ENABLE,
                            kFLEXSPI_Command_STOP,      0x00,          0x00),
        //[16] - Seq 4 empty
        // Sequence 5 - 4K Sector erase
        // 0x20 - Sector erase command, 0x18 - 24 bit address
        [FLEXSPI_ERASE_SECTOR_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_SECTOR_ERASE,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        //[24] - Seq 6 empty
        //[28] - Seq 7 empty
        // Sequence 8 - 64K Block erase
        // 0xD8 - Block erase command, 0x18 - 24 bit address
        [FLEXSPI_ERASE_BLOCK_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_BLOCK_ERASE,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // Sequence 9 - Page Program, 256 bytes
        // 0x02 - page program command, 0x18 - 24 bit address
        [FLEXSPI_PROGRAM_PAGE_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_PAGE_PROGRAM,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // 0x04 - write 4 bytes
        [FLEXSPI_PROGRAM_PAGE_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS + 1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04,
                            kFLEXSPI_Command_STOP,      0x00,          0x00),
        //[40] - Seq 10 empty
        // Sequence 11 - Chip erase
        // 0x60 - chip erase command
        [44] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_CHIP_ERASE,
                               kFLEXSPI_Command_STOP,      0x00,          0x00),
        // Sequence 12 - Read JEDEC
        // 0x9F - read JEDEC command, 0x04 - read 4 bytes
        [48] = FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_READ_JEDEC,
                               kFLEXSPI_Command_READ_SDR,  kFLEXSPI_1PAD, 0x04),
        //[52-60] - Seqs 13 - 15 empty
    },
    .pageSize           = 0x100,
    .sectorSize         = 0x1000,
    .ipcmdSerialClkFreq = 1,
    .isUniformBlockSize = 0,
    .blockSize          = 0x10000,
};
#endif /* BOOT_HEADER_ENABLE */