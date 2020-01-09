/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "flash_config.h"
#include "FlashW25Q16FW.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*! @name LUT - LUT 0..LUT 127 */
/*! @{ */
#define FLEXSPI_LUT_OPERAND0_MASK                (0xFFU)
#define FLEXSPI_LUT_OPERAND0_SHIFT               (0U)
/*! OPERAND0 - OPERAND0
 */
#define FLEXSPI_LUT_OPERAND0(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND0_SHIFT)) & FLEXSPI_LUT_OPERAND0_MASK)
#define FLEXSPI_LUT_NUM_PADS0_MASK               (0x300U)
#define FLEXSPI_LUT_NUM_PADS0_SHIFT              (8U)
/*! NUM_PADS0 - NUM_PADS0
 */
#define FLEXSPI_LUT_NUM_PADS0(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS0_SHIFT)) & FLEXSPI_LUT_NUM_PADS0_MASK)
#define FLEXSPI_LUT_OPCODE0_MASK                 (0xFC00U)
#define FLEXSPI_LUT_OPCODE0_SHIFT                (10U)
/*! OPCODE0 - OPCODE
 */
#define FLEXSPI_LUT_OPCODE0(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE0_SHIFT)) & FLEXSPI_LUT_OPCODE0_MASK)
#define FLEXSPI_LUT_OPERAND1_MASK                (0xFF0000U)
#define FLEXSPI_LUT_OPERAND1_SHIFT               (16U)
/*! OPERAND1 - OPERAND1
 */
#define FLEXSPI_LUT_OPERAND1(x)                  (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPERAND1_SHIFT)) & FLEXSPI_LUT_OPERAND1_MASK)
#define FLEXSPI_LUT_NUM_PADS1_MASK               (0x3000000U)
#define FLEXSPI_LUT_NUM_PADS1_SHIFT              (24U)
/*! NUM_PADS1 - NUM_PADS1
 */
#define FLEXSPI_LUT_NUM_PADS1(x)                 (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_NUM_PADS1_SHIFT)) & FLEXSPI_LUT_NUM_PADS1_MASK)
#define FLEXSPI_LUT_OPCODE1_MASK                 (0xFC000000U)
#define FLEXSPI_LUT_OPCODE1_SHIFT                (26U)
/*! OPCODE1 - OPCODE1
 */
#define FLEXSPI_LUT_OPCODE1(x)                   (((uint32_t)(((uint32_t)(x)) << FLEXSPI_LUT_OPCODE1_SHIFT)) & FLEXSPI_LUT_OPCODE1_MASK)
/*! @} */

/*! @brief pad definition of FLEXSPI, use to form LUT instruction. */
enum _flexspi_pad
{
    kFLEXSPI_1PAD = 0x00U, /*!< Transmit command/address and transmit/receive data only through DATA0/DATA1. */
    kFLEXSPI_2PAD = 0x01U, /*!< Transmit command/address and transmit/receive data only through DATA[1:0]. */
    kFLEXSPI_4PAD = 0x02U, /*!< Transmit command/address and transmit/receive data only through DATA[3:0]. */
    kFLEXSPI_8PAD = 0x03U, /*!< Transmit command/address and transmit/receive data only through DATA[7:0]. */
};

enum _flexspi_command
{
    kFLEXSPI_Command_STOP           = 0x00U, /*!< Stop execution, deassert CS. */
    kFLEXSPI_Command_SDR            = 0x01U, /*!< Transmit Command code to Flash, using SDR mode. */
    kFLEXSPI_Command_RADDR_SDR      = 0x02U, /*!< Transmit Row Address to Flash, using SDR mode. */
    kFLEXSPI_Command_CADDR_SDR      = 0x03U, /*!< Transmit Column Address to Flash, using SDR mode. */
    kFLEXSPI_Command_MODE1_SDR      = 0x04U, /*!< Transmit 1-bit Mode bits to Flash, using SDR mode. */
    kFLEXSPI_Command_MODE2_SDR      = 0x05U, /*!< Transmit 2-bit Mode bits to Flash, using SDR mode. */
    kFLEXSPI_Command_MODE4_SDR      = 0x06U, /*!< Transmit 4-bit Mode bits to Flash, using SDR mode. */
    kFLEXSPI_Command_MODE8_SDR      = 0x07U, /*!< Transmit 8-bit Mode bits to Flash, using SDR mode. */
    kFLEXSPI_Command_WRITE_SDR      = 0x08U, /*!< Transmit Programming Data to Flash, using SDR mode. */
    kFLEXSPI_Command_READ_SDR       = 0x09U, /*!< Receive Read Data from Flash, using SDR mode. */
    kFLEXSPI_Command_LEARN_SDR      = 0x0AU, /*!< Receive Read Data or Preamble bit from Flash, SDR mode. */
    kFLEXSPI_Command_DATSZ_SDR      = 0x0BU, /*!< Transmit Read/Program Data size (byte) to Flash, SDR mode. */
    kFLEXSPI_Command_DUMMY_SDR      = 0x0CU, /*!< Leave data lines undriven by FlexSPI controller.*/
    kFLEXSPI_Command_DUMMY_RWDS_SDR = 0x0DU, /*!< Leave data lines undriven by FlexSPI controller,
                                                  dummy cycles decided by RWDS. */
    kFLEXSPI_Command_DDR            = 0x21U, /*!< Transmit Command code to Flash, using DDR mode. */
    kFLEXSPI_Command_RADDR_DDR      = 0x22U, /*!< Transmit Row Address to Flash, using DDR mode. */
    kFLEXSPI_Command_CADDR_DDR      = 0x23U, /*!< Transmit Column Address to Flash, using DDR mode. */
    kFLEXSPI_Command_MODE1_DDR      = 0x24U, /*!< Transmit 1-bit Mode bits to Flash, using DDR mode. */
    kFLEXSPI_Command_MODE2_DDR      = 0x25U, /*!< Transmit 2-bit Mode bits to Flash, using DDR mode. */
    kFLEXSPI_Command_MODE4_DDR      = 0x26U, /*!< Transmit 4-bit Mode bits to Flash, using DDR mode. */
    kFLEXSPI_Command_MODE8_DDR      = 0x27U, /*!< Transmit 8-bit Mode bits to Flash, using DDR mode. */
    kFLEXSPI_Command_WRITE_DDR      = 0x28U, /*!< Transmit Programming Data to Flash, using DDR mode. */
    kFLEXSPI_Command_READ_DDR       = 0x29U, /*!< Receive Read Data from Flash, using DDR mode. */
    kFLEXSPI_Command_LEARN_DDR      = 0x2AU, /*!< Receive Read Data or Preamble bit from Flash, DDR mode. */
    kFLEXSPI_Command_DATSZ_DDR      = 0x2BU, /*!< Transmit Read/Program Data size (byte) to Flash, DDR mode. */
    kFLEXSPI_Command_DUMMY_DDR      = 0x2CU, /*!< Leave data lines undriven by FlexSPI controller.*/
    kFLEXSPI_Command_DUMMY_RWDS_DDR = 0x2DU, /*!< Leave data lines undriven by FlexSPI controller,
                                               dummy cycles decided by RWDS. */
    kFLEXSPI_Command_JUMP_ON_CS = 0x1FU,     /*!< Stop execution, deassert CS and save operand[7:0] as the
                                               instruction start pointer for next sequence */
};

#define FLEXSPI_LUT_SEQUENCE_SIZE_WORDS 4
#define FLEXSPI_READ_DATA_LUT_SEQ_INDEX                 0
#define FLEXSPI_READ_STATUS_LUT_SEQ_INDEX               1
#define FLEXSPI_WRITE_STATUS_REGISTER_2_LUT_SEQ_INDEX   2
#define FLEXSPI_WRITE_ENABLE_LUT_SEQ_INDEX              3
#define FLEXSPI_ERASE_SECTOR_LUT_SEQ_INDEX              5
#define FLEXSPI_ERASE_BLOCK_LUT_SEQ_INDEX               8
#define FLEXSPI_PROGRAM_PAGE_LUT_SEQ_INDEX              9


const flexspi_boot_config_t g_flexSpiConfig = {
    .tag                  = FLASH_CONFIG_BLOCK_TAG,
    .version              = FLASH_CONFIG_BLOCK_VERSION,
    .readSampleClkSrc     = kFlexSPIReadSampleClk_LoopbackInternally,
    .csHoldTime           = 3,
    .csSetupTime          = 3,
    .columnAddressWidth   = 0,
    .deviceModeCfgEnable  = 0, // TODO enable quad mode by sending an initial command to flash
    .deviceModeType       = 0,
    .waitTimeCfgCommands  = 0,
    .deviceModeSeq        = {.seqNum = 0,
                             .seqId  = 0,
                            },  // Run single command, sequence #2
    .deviceModeArg        = 0, //Set bit 2 with that command (Write Status Register 2)
    .configCmdEnable      = 0,
    .configModeType       = {0},
    //.configCmdSeqs        = {0},
    .configCmdArgs        = {0},
    .controllerMiscOption = (0),
    .deviceType           = 0x0,
    .sflashPadType        = kSerialFlash_1Pads,
    .serialClkFreq        = kFlexSpiSerialClk_48MHz,
    .lutCustomSeqEnable   = 0,
    .sflashA1Size         = 0x00200000,
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


        // Sequence 0 - Read Data
        // 0x03 - Read Data command, 0x18 - 24 bit address
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS]     =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_READ_DATA,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // 0x80 - read 128 bytes, stop
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS + 1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR,  kFLEXSPI_1PAD, 0x80,
                            kFLEXSPI_Command_STOP,      0x00,          0x00),

/*
        // Sequence 0 - Quad Read
        // 0x6B - Fast read Quad Output command, 0x18 - 24 bit address
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS]     =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR,       kFLEXSPI_1PAD, W25Q16FW_FLASH_COMMAND_QUAD_READ,
                            kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
        // 0x8 - 8 dummy clocks, 0x80 - read 128 bytes
        [FLEXSPI_READ_DATA_LUT_SEQ_INDEX * FLEXSPI_LUT_SEQUENCE_SIZE_WORDS + 1] =
            FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08,
                            kFLEXSPI_Command_READ_SDR,  kFLEXSPI_4PAD, 0x80),
*/

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
