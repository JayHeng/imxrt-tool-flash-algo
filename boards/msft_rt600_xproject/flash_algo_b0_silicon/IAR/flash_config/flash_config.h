/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FLASH_CONFIG__
#define __FLASH_CONFIG__

#include <stdint.h>
#include <stdbool.h>
//#include "fsl_common.h"
/*******************************************************************************
 * Definition
 ******************************************************************************/

/* FLEXSPI memory config block related defintions */
#define FLASH_CONFIG_BLOCK_TAG (0x42464346)
#define FLASH_CONFIG_BLOCK_VERSION (0x56010400)

#define CMD_SDR 0x01
#define CMD_DDR 0x21
#define RADDR_SDR 0x02
#define RADDR_DDR 0x22
#define CADDR_SDR 0x03
#define CADDR_DDR 0x23
#define MODE1_SDR 0x04
#define MODE1_DDR 0x24
#define MODE2_SDR 0x05
#define MODE2_DDR 0x25
#define MODE4_SDR 0x06
#define MODE4_DDR 0x26
#define MODE8_SDR 0x07
#define MODE8_DDR 0x27
#define WRITE_SDR 0x08
#define WRITE_DDR 0x28
#define READ_SDR 0x09
#define READ_DDR 0x29
#define LEARN_SDR 0x0A
#define LEARN_DDR 0x2A
#define DATSZ_SDR 0x0B
#define DATSZ_DDR 0x2B
#define DUMMY_SDR 0x0C
#define DUMMY_DDR 0x2C
#define DUMMY_RWDS_SDR 0x0D
#define DUMMY_RWDS_DDR 0x2D
#define JMP_ON_CS 0x1F
#define STOP 0

#define FLEXSPI_1PAD 0
#define FLEXSPI_2PAD 1
#define FLEXSPI_4PAD 2
#define FLEXSPI_8PAD 3

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1)                                                              \
    (FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
     FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

//!@brief FlexSPI Read Sample Clock Source definition
enum
{
    kFlexSPIReadSampleClk_LoopbackInternally      = 0,
    kFlexSPIReadSampleClk_LoopbackFromDqsPad      = 1,
    kFlexSPIReadSampleClk_ExternalInputFromDqsPad = 3,
};

/* !@brief Data pad used in Read command */
enum
{
    kSerialFlash_1Pads = 1,
    kSerialFlash_2Pads = 2,
    kSerialFlash_4Pads = 4,
    kSerialFlash_8Pads = 8,
};

/* !@brief FLEXSPI clock configuration - In High speed boot mode mode */
/*
enum
{
    kFlexSpiSerialClk_30MHz  = 1,
    kFlexSpiSerialClk_50MHz  = 2,
    kFlexSpiSerialClk_60MHz  = 3,
    kFlexSpiSerialClk_80MHz  = 4,
    kFlexSpiSerialClk_100MHz = 5,
    kFlexSpiSerialClk_120MHz = 6,
    kFlexSpiSerialClk_133MHz = 7,
    kFlexSpiSerialClk_166MHz = 8,
    kFlexSpiSerialClk_200MHz = 9,
};
*/

/* !@brief FLEXSPI clock configuration - In Normal boot mode */
enum
{
    kFlexSpiSerialClk_32MHz  = 1,
    kFlexSpiSerialClk_48MHz  = 2,
    kFlexSpiSerialClk_64MHz  = 3,
    kFlexSpiSerialClk_96MHz  = 4,
    kFlexSpiSerialClk_192MHz = 5,
};

/* !@brief Misc feature bit definitions */
enum
{
    kFlexSpiMiscOffset_DiffClkEnable            = 0, //!< Bit for Differential clock enable
    kFlexSpiMiscOffset_Ck2Enable                = 1, //!< Bit for CK2 enable
    kFlexSpiMiscOffset_ParallelEnable           = 2, //!< Bit for Parallel mode enable
    kFlexSpiMiscOffset_WordAddressableEnable    = 3, //!< Bit for Word Addressable enable
    kFlexSpiMiscOffset_SafeConfigFreqEnable     = 4, //!< Bit for Safe Configuration Frequency enable
    kFlexSpiMiscOffset_PadSettingOverrideEnable = 5, //!< Bit for Pad setting override enable
    kFlexSpiMiscOffset_DdrModeEnable            = 6, //!< Bit for DDR clock confiuration indication.
};

//!@brief FlexSPI LUT Sequence structure
typedef struct _lut_sequence
{
    uint8_t seqNum; //!< Sequence Number, valid number: 1-16
    uint8_t seqId;  //!< Sequence Index, valid number: 0-15
    uint16_t reserved;
} flexspi_lut_seq_t;

typedef struct
{
    uint8_t time_100ps;  // Data valid time, in terms of 100ps
    uint8_t delay_cells; // Data valid time, in terms of delay cells
} flexspi_dll_time_t;

typedef struct _FlexspiBootConfig
{
    uint32_t tag;
    uint32_t version;
    uint32_t reserved0;
    uint8_t readSampleClkSrc;   //!< [0x00c-0x00c] Read Sample Clock Source, valid value: 0/1/3
    uint8_t csHoldTime;
    uint8_t csSetupTime;
    uint8_t columnAddressWidth;
    uint8_t deviceModeCfgEnable; /* off: 0x10 */
    uint8_t deviceModeType;
    uint16_t waitTimeCfgCommands;
    flexspi_lut_seq_t deviceModeSeq;
    uint32_t deviceModeArg;
    uint8_t configCmdEnable;
    uint8_t configModeType[3];
    flexspi_lut_seq_t configCmdSeqs[3]; /* off: 0x20 */
    uint32_t reserved1;
    uint32_t configCmdArgs[3]; /* off: 0x30 */
    uint32_t reserved2;
    uint32_t controllerMiscOption; /* off: 0x40 */
    uint8_t deviceType;
    uint8_t sflashPadType;
    uint8_t serialClkFreq;
    uint8_t lutCustomSeqEnable;
    uint32_t reserved3[2];
    uint32_t sflashA1Size; /* off: 0x50 */
    uint32_t sflashA2Size;
    uint32_t sflashB1Size;
    uint32_t sflashB2Size;
    uint32_t csPadSettingOverride; /* off: 0x60 */
    uint32_t sclkPadSettingOverride;
    uint32_t dataPadSettingOverride;
    uint32_t dqsPadSettingOverride;
    uint32_t timeoutInMs; /* off: 0x70 */
    uint32_t commandInterval;
    flexspi_dll_time_t dataValidTime[2];
    uint16_t busyOffset;
    uint16_t busyBitPolarity;
    uint32_t lookupTable[64];       /* off: 0x80 */
    flexspi_lut_seq_t lutCustomSeq[12]; /* off: 0x180 */
    uint32_t reserved4[4];
    uint32_t pageSize;      /* off: 0x1C0 */
    uint32_t sectorSize;
    uint8_t ipcmdSerialClkFreq;
    uint8_t isUniformBlockSize;
    uint8_t isDataOrderSwapped;
    uint8_t reserved5[1];
    uint8_t serialNorType;
    uint8_t needExitNoCmdMode;
    uint8_t halfClkForNonReadCmd;
    uint8_t needRestoreNoCmdMode;
    uint32_t blockSize; /* off: 0x1D0 */
    uint32_t flashStateCtx;
    uint32_t reserve6[10];
} flexspi_boot_config_t;

extern const flexspi_boot_config_t g_flexSpiConfig;


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif
#endif /* __FLASH_CONFIG__ */
