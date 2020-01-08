/*
 * The Clear BSD License
 * Copyright 2014-2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*********************************************************************************************************************
 * Revision History
 *
 * V1.0.0 First implementation, support all the flash devices in the market
 *         except for the Adesto ATXP series under OPI/QPI mode
 * V1.1.0 Add optional delay as an alternative in case the read status is not applicable for all the commands in config
 *block
 *        Add second write enable command support for the device that need multiple phase to switch to other mode
 * v2.0.0 Full re-architect QuadSPI Configuration Block
 *
 **********************************************************************************************************************/

#ifndef __FSL_QUADSPI_H__
#define __FSL_QUADSPI_H__

#include <stdbool.h>
#include <stdint.h>

#ifndef QuadSPI
#define QuadSPI QUADSPI
#endif

/*! @brief Construct a status code value from a group and code number. */
#define MAKE_STATUS(group, code) ((((group)*100) + (code)))
/*! @brief Type used for all status and error return values. */
typedef int32_t status_t;
/*! @brief Group number for QSPI status codes. */
#define kStatusGroup_QSPI 45

//!@brief LUT Command definitions
#define CMD_SDR 0x01
#define ADDR_SDR 0x02
#define DUMMY 0x03
#define MODE8_SDR 0x04
#define MODE2_SDR 0x05
#define MODE4_SDR 0x06
#define READ_SDR 0x07
#define WRITE_SDR 0x08
#define JMP_ON_CS 0x09
#define ADDR_DDR 0x0a
#define MODE8_DDR 0x0b
#define MODE2_DDR 0x0c
#define MODE4_DDR 0x0d
#define READ_DDR 0x0e
#define WRITE_DDR 0x0f
#define DATA_LEAN 0x10
#define CMD_DDR 0x11
#define CADDR_SDR 0x12
#define CADDR_DDR 0x13
#define STOP 0x00

#define QUADSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1) \
    (((cmd0) << 10) | ((pad0) << 8) | (op0) | ((cmd1) << 26) | ((pad1) << 24) | ((op1) << 16))

//!@brief Flash Pad Definitions
enum
{
    kSerialFlash_1Pad = 1,
    kSerialFlash_2Pads = 2,
    kSerialFlash_4Pads = 4,
    kSerialFlash_8Pads = 8,
};

enum
{
    kQuadSpiSerialClk_30MHz = 1,
    kQuadSpiSerialClk_50MHz = 2,
    kQuadSpiSerialClk_60MHz = 3,
    kQuadSpiSerialClk_72MHz = 4,
    kQuadSpiSerialClk_80MHz = 5,
    kQuadSpiSerialClk_90MHz = 6,
    kQuadSpiSerialClk_100MHz = 7,
};

#define QUADSPI_1PAD 0
#define QUADSPI_2PAD 1
#define QUADSPI_4PAD 2
#define QUADSPI_8PAD 3

//!@brief Flash Configuration Command Type
enum
{
    kDeviceConfigCmdType_Generic,    //!< Generic command, for example: configure dummy cycles, drive strength, etc
    kDeviceConfigCmdType_QuadEnable, //!< Quad Enable command
    kDeviceConfigCmdType_Spi2Xpi,    //!< Switch from SPI to DPI/QPI/OPI mode
    kDeviceConfigCmdType_Xpi2Spi,    //!< Switch from DPI/QPI/OPI to SPI mode
    kDeviceConfigCmdType_Spi2NoCmd,  //!< Switch to 0-4-4/0-8-8 mode
    kDeviceConfigCmdType_Reset,      //!< Reset device command
};

//!@brief QuadSPI Read Sampling option
enum
{
    kQuadSpiReadSamplingOption_InternalSampling = 0,
    kQuadSpiReadSamplingOption_InternalLoopback = 1,
    kQuadSpiReadSamplingOption_InternalDqsLoopback = 2,
    kQuadSpiReadSamplingOption_ExternalDqs = 3
};

//!@brief Flash Type Definition
enum
{
    kQuadSpiDeviceType_SerialNOR = 1,       //!< Flash devices are Serial NOR
    kQuadSpiDeviceType_SerialNAND = 2,      //!< Flash devices are Serial NAND
    kQuadSpiDeviceType_MCP_NOR_NAND = 0x12, //!< Flash device is MCP device, A1 is Serial NOR, A2 is Serial NAND
};

//!@brief Misc feature bit definitions
enum
{
    kQuadSpiMiscOffset_DiffClkEnable = 0,            //!< Bit for Differential clock enable
    kQuadSpiMiscOffset_Ck2Enable = 1,                //!< Bit for CK2 enable
    kQuadSpiMiscOffset_ParallelEnable = 2,           //!< Bit for Parallel mode enable
    kQuadSpiMiscOffset_WordAddressableEnable = 3,    //!< Bit for Word Addressable enable
    kQuadSpiMiscOffset_SafeConfigFreqEnable = 4,     //!< Bit for Safe Configuration Frequency enable
    kQuadSpiMiscOffset_PadSettingOverrideEnable = 5, //!< Bit for Pad setting override enable
    kQuadSpiMiscOffset_DdrModeEnable = 6,            //!< Bit for DDR clock confiuration indication.
    kQuadSpiMiscOffset_DqsInvSel = 7,                //!< Bit for 1X Inverted Clk as DQS clock
    kQuadSpiMiscOffset_SkipAhbBufConfig = 0x10,      //!< Bit for Skip AHB buffer configuration
};

//!@brief QuadSPI LUT Sequence structure
typedef struct __lut_sequence
{
    uint8_t seqNum; //!< Sequence number, valid number: 1-16
    uint8_t seqId;  //!< Sequence index, valid value: 0-15
    uint16_t reserved;
} quadspi_lut_seq_t;

typedef enum _quadspi_xfer_mode {
    kQuadSpiMode_SDR,
    kQuadSpiMode_DDR,
} quadspi_xfer_mode_t;

/* QuadSPI memory config block related defintions */
#define QUADSPI_CFG_BLK_TAG (0x42464346UL)     // ascii "FCFB" Big Endian
#define QUADSPI_CFG_BLK_VERSION (0x56020000UL) // V2.0.0
#define QUADSPI_CFG_BLK_SIZE (512)

/* Lookup table related defintions */
#define CMD_INDEX_READ 0
#define CMD_INDEX_READSTATUS 1
#define CMD_INDEX_WRITEENABLE 2
#define CMD_INDEX_WRITE 4

#define CMD_LUT_SEQ_IDX_READ 0
#define CMD_LUT_SEQ_IDX_READSTATUS 1
#define CMD_LUT_SEQ_IDX_WRITEENABLE 3
#define CMD_LUT_SEQ_IDX_WRITE 9

//! @brief Error codes of QuadSPI driver
typedef enum _qspi_status {
    //! @brief Error code which represents that a command is not supported under
    //!        certain mode.
    kStatus_QspiCommandNotSupported = MAKE_STATUS(kStatusGroup_QSPI, 0),
    //! @brief Error code which represents that operation is timeout
    kStatus_QspiCommandTimeout = MAKE_STATUS(kStatusGroup_QSPI, 1),

    //! @brief Error code which represents that QSPI cannot perform write operation at expected frequency
    kStatus_QspiTxUnderflow = MAKE_STATUS(kStatusGroup_QSPI, 2),

    //! @brief Error code which represents that QSPI cannot perform write operation at expected frequency
    kStatus_QspiRxOverflow = MAKE_STATUS(kStatusGroup_QSPI, 3),

    //! @brief Error code which represents that the QSPI module is busy, which may be caused by incorrect
    //        commands in LUT
    kStatusQspiModuleBusy = MAKE_STATUS(kStatusGroup_QSPI, 4),
} qspi_status_t;

//!@brief QuadSPI Memory Configuration Block 2.0
typedef struct __QuadSpiConfig
{
    uint32_t tag;                       //!< [0x000 - 0x003] Tag, fixed value
    uint32_t version;                   //!< [0x004 - 0x007] version
    uint32_t reserved0;                 //!< [0x008 - 0x00b] Reserved
    uint8_t readSamplingOption;         //!< [0x00c - 0x00c] Read sampling option
    uint8_t csHoldTime;                 //!< [0x00d - 0x00d] CS hold time
    uint8_t csSetupTime;                //!< [0x00e - 0x00e] CS setup time
    uint8_t columnAddressWidth;         //!< [0x00f - 0x00f] Column address width
    uint8_t deviceModeCfgEnable;        //!< [0x010 - 0x010] Device mode Configuration Enable
    uint8_t deviceModeType;             //!< [0x011 - 0x011] Device mode type
    uint16_t waitTimeCfgCommands;       //!< [0x012 - 0x013] Wait time for all configuration commands, unit: 100us
    quadspi_lut_seq_t deviceModeSeq;    //!< [0x014 - 0x017] Device Mode Configuration LUT sequence
    uint32_t deviceModeArg;             //!< [0x018 - 0x01b] Device Mode argument
    uint8_t configCmdEnable;            //!< [0x01c - 0x01c] Config Command Enable flag
    uint8_t configModeType[3];          //!< [0x01d - 0x01f] Config command types, support up to 3 types
    quadspi_lut_seq_t configCmdSeqs[3]; //!< [0x020 - 0x02b] Config Command sequences, support up to 3 sequences
    uint32_t reserved1;                 //!< [0x02c - 0x02f] Reserved
    uint32_t configCmdArgs[3];          //!< [0x030 - 0x03b] Configure comamnd arugments
    uint32_t reserved2;                 //!< [0x03c - 0x03f] Reserved
    uint32_t controllerMiscOption;      //!< [0x040 - 0x043] Misc. Control options
    uint8_t deviceType;                 //!< [0x044 - 0x044] Device type: 1 - Serial NOR
    uint8_t sflashPadType;              //!< [0x045 - 0x045] Flash Pad type
    uint8_t serialClkFreq;              //!< [0x046 - 0x046] Flash Frequencye option
    uint8_t lutCustomSeqEnable;         //!< [0x047 - 0x047] LUT customizition Enable flag
    uint32_t reserved3[2];              //!< [0x048 - 0x04f] Reserved
    uint32_t sflashSize[4];             //!< [0x050 - 0x05f] Flash Size
    uint32_t csPadSettingOverride;      //!< [0x060 - 0x063] CS pad override value
    uint32_t sclkPadSettingOverride;    //!< [0x064 - 0x067] SCKL pad override value
    uint32_t dataPadSettingOverride;    //!< [0x068 - 0x06b] Data pad override value
    uint32_t dqsPadSettingOverride;     //!< [0x06c - 0x06f] DQS pad override value
    uint32_t timeoutInMs;               //!< [0x070 - 0x073] Timeout value to terminate busy check
    uint8_t coarseTuning;               //!< [0x074 - 0x074] Coarse tuning (sample phase)
    uint8_t reserved4;                  //!< [0x075 - 0x075] Reserved
    uint8_t fineTuning[2];              //!< [0x076 - 0x077] Fine tuning (delay cells)
    uint8_t samplePoint;                //!< [0x078 - 0x078] Sampling point
    uint8_t dataHoldTime;               //!< [0x079 - 0x079] Data Hold time
    uint8_t reserved5[2];               //!< [0x07a - 0x07b] Reserved
    uint16_t busyOffset;                //!< [0x07c - 0x07d] Busy offset
    uint16_t busyBitPolarity; //!< [0x07e - 0x07f] Bussy offset polarity: 0 - busy bit is 1 when device is busy
    uint32_t lookupTable[64]; //!< [0x080 - 0x17f] Lockup table
    quadspi_lut_seq_t lutCustomSeq[12]; //!< [0x180 - 0x1af] Customized Sequences for pre-defined commands
    uint32_t reserved6[4];              //!< [0x1b0 - 0x1bf] // Reserved
} quadspi_mem_config_t;

typedef enum _QuadSpiOperationType {
    kQuadSpiOperation_Command, //!< QuadSPI operation: Only command, both TX and
    //! RX buffer are ignored.
    kQuadSpiOperation_Config, //!< QuadSPI operation: Configure device mode, the
    //! TX FIFO size is fixed in LUT.
    kQuadSpiOperation_Write, //!< QuadSPI operation: Write,  only TX buffer is
    //! effective
    kQuadSpiOperation_Read, //!< QuadSPI operation: Read, only Rx Buffer is
    //! effective.
    kQuadSpiOperation_End = kQuadSpiOperation_Read,
} quadspi_operation_t;

//!@brief QuadSPI Transfer Context
typedef struct _QuadSpiXfer
{
    quadspi_operation_t operation; //!< QuadSPI operation
    uint32_t baseAddress;          //!< QuadSPI operation base address
    int32_t seqId;                 //!< Sequence Id
    int32_t seqNum;                //!< Sequence Number
    bool isParallelModeEnable;     //!< Is a parallel transfer
    uint32_t *txBuffer;            //!< Tx buffer
    uint32_t txSize;               //!< Tx size in bytes
    uint32_t *rxBuffer;            //!< Rx buffer
    uint32_t rxSize;               //!< Rx size in bytes
} quadspi_xfer_t;

#define QUADSPI_BITMASK(bit_offset) (1u << (bit_offset))

/**********************************************************************************************************************
 * API
 *********************************************************************************************************************/

#ifdef __cplusplus
extern "C" {
#endif

status_t quadspi_init(quadspi_mem_config_t *config);

status_t quadspi_device_write_enable(quadspi_mem_config_t *config, bool isParallelMode, uint32_t baseAddr);

status_t quadspi_device_wait_busy(quadspi_mem_config_t *config, bool isParallelMode, uint32_t baseAddr);

status_t quadspi_update_lut(uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);

status_t quadspi_command_xfer(quadspi_xfer_t *xfer);

void quadspi_wait_idle(void);

void quadspi_clear_cache(void);

void quadspi_clear_sequence_pointer(void);

extern void quadspi_clock_enable(void);

extern void quadspi_clock_disable(void);

//!@brief Check whether Differential clock feature is enabled.
bool quadspi_is_differential_clock_enable(quadspi_mem_config_t *config);

//!@brief Check whether DDR mode feature is enabled.
bool quadspi_is_ddr_mode_enable(quadspi_mem_config_t *config);

bool quadspi_is_parallel_mode_enable(quadspi_mem_config_t *config);

bool quadspi_is_busy(void);


#ifdef __cplusplus
}
#endif

#endif // __FSL_QUADSPI_H__
