/*
 * The Clear BSD License
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
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

#ifndef __FLEXSPI_NOR_FLASH_H__
#define __FLEXSPI_NOR_FLASH_H__

#include "fsl_common.h"

/**********************************************************************************************************************
 * Definitions
 *********************************************************************************************************************/

/* FLEXSPI memory config block related defintions */
#define FLEXSPI_CFG_BLK_TAG     (0x42464346UL) // ascii "FCFB" Big Endian
#define FLEXSPI_CFG_BLK_VERSION (0x56010400UL) // V1.4.0
#define FLEXSPI_CFG_BLK_SIZE    (512)

#define CMD_SDR        0x01
#define CMD_DDR        0x21
#define RADDR_SDR      0x02
#define RADDR_DDR      0x22
#define CADDR_SDR      0x03
#define CADDR_DDR      0x23
#define MODE1_SDR      0x04
#define MODE1_DDR      0x24
#define MODE2_SDR      0x05
#define MODE2_DDR      0x25
#define MODE4_SDR      0x06
#define MODE4_DDR      0x26
#define MODE8_SDR      0x07
#define MODE8_DDR      0x27
#define WRITE_SDR      0x08
#define WRITE_DDR      0x28
#define READ_SDR       0x09
#define READ_DDR       0x29
#define LEARN_SDR      0x0A
#define LEARN_DDR      0x2A
#define DATSZ_SDR      0x0B
#define DATSZ_DDR      0x2B
#define DUMMY_SDR      0x0C
#define DUMMY_DDR      0x2C
#define DUMMY_RWDS_SDR 0x0D
#define DUMMY_RWDS_DDR 0x2D
#define JMP_ON_CS      0x1F
#define STOP           0

#define FLEXSPI_1PAD 0
#define FLEXSPI_2PAD 1
#define FLEXSPI_4PAD 2
#define FLEXSPI_8PAD 3

#define FLEXSPI_LUT_SEQ(cmd0, pad0, op0, cmd1, pad1, op1)                                                              \
    (FLEXSPI_LUT_OPERAND0(op0) | FLEXSPI_LUT_NUM_PADS0(pad0) | FLEXSPI_LUT_OPCODE0(cmd0) | FLEXSPI_LUT_OPERAND1(op1) | \
     FLEXSPI_LUT_NUM_PADS1(pad1) | FLEXSPI_LUT_OPCODE1(cmd1))

//!@brief Defintions for FlexSPI Serial Clock Frequency
typedef enum _FlexSpiSerialClockFreq
{
    kFlexSpiSerialClk_NoChange = 0,
    kFlexSpiSerialClk_30MHz    = 1,
    kFlexSpiSerialClk_50MHz    = 2,
    kFlexSpiSerialClk_60MHz    = 3,
    kFlexSpiSerialClk_75MHz    = 4,
    kFlexSpiSerialClk_80MHz    = 5,
    kFlexSpiSerialClk_100MHz   = 6,
    kFlexSpiSerialClk_120MHz   = 7,
    kFlexSpiSerialClk_133MHz   = 8,
    kFlexSpiSerialClk_166MHz   = 9,
} flexspi_serial_clk_freq_t;

//!@brief FlexSPI clock configuration type
enum
{
    kFlexSpiClk_SDR, //!< Clock configure for SDR mode
    kFlexSpiClk_DDR, //!< Clock configurat for DDR mode
};

//!@brief FlexSPI Read Sample Clock Source definition
typedef enum _FlashReadSampleClkSource
{
    kFlexSPIReadSampleClk_LoopbackInternally      = 0,
    kFlexSPIReadSampleClk_LoopbackFromDqsPad      = 1,
    kFlexSPIReadSampleClk_LoopbackFromSckPad      = 2,
    kFlexSPIReadSampleClk_ExternalInputFromDqsPad = 3,
} flexspi_read_sample_clk_t;

/* status code for flexspi */
enum _flexspi_status
{
    kStatus_FLEXSPI_SequenceExecutionTimeout =
        MAKE_STATUS(kStatusGroup_FLEXSPI, 0),                               //!< Status for Sequence Execution timeout
    kStatus_FLEXSPI_InvalidSequence = MAKE_STATUS(kStatusGroup_FLEXSPI, 1), //!< Status for Invalid Sequence
    kStatus_FLEXSPI_DeviceTimeout   = MAKE_STATUS(kStatusGroup_FLEXSPI, 2), //!< Status for Device timeout
};

//!@brief Misc feature bit definitions
enum
{
    kFlexSpiMiscOffset_DiffClkEnable            = 0, //!< Bit for Differential clock enable
    kFlexSpiMiscOffset_Ck2Enable                = 1, //!< Bit for CK2 enable
    kFlexSpiMiscOffset_ParallelEnable           = 2, //!< Bit for Parallel mode enable
    kFlexSpiMiscOffset_WordAddressableEnable    = 3, //!< Bit for Word Addressable enable
    kFlexSpiMiscOffset_SafeConfigFreqEnable     = 4, //!< Bit for Safe Configuration Frequency enable
    kFlexSpiMiscOffset_PadSettingOverrideEnable = 5, //!< Bit for Pad setting override enable
    kFlexSpiMiscOffset_DdrModeEnable            = 6, //!< Bit for DDR clock confiuration indication.
    kFlexSpiMiscOffset_UseValidTimeForAllFreq   = 7, //!< Bit for DLLCR settings under all modes
    kFlexSpiMiscOffset_SecondPinMux             = 8, //!< Bit for Second Pinmux group
};

//!@brief Flash Type Definition
enum
{
    kFlexSpiDeviceType_SerialNOR  = 1, //!< Flash devices are Serial NOR
    kFlexSpiDeviceType_SerialNAND = 2, //!< Flash devices are Serial NAND
};

//!@brief Flash Pad Definitions
enum
{
    kSerialFlash_1Pad  = 1,
    kSerialFlash_2Pads = 2,
    kSerialFlash_4Pads = 4,
    kSerialFlash_8Pads = 8,
};

//!@brief FlexSPI LUT Sequence structure
typedef struct _lut_sequence
{
    uint8_t seqNum; //!< Sequence Number, valid number: 1-16
    uint8_t seqId;  //!< Sequence Index, valid number: 0-15
    uint16_t reserved;
} flexspi_lut_seq_t;

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

typedef struct
{
    uint8_t time_100ps;  // Data valid time, in terms of 100ps
    uint8_t delay_cells; // Data valid time, in terms of delay cells
} flexspi_dll_time_t;

//!@brief FlexSPI Memory Configuration Block
typedef struct _FlexSPIConfig
{
    uint32_t tag;               //!< [0x000-0x003] Tag, fixed value 0x42464346UL
    uint32_t version;           //!< [0x004-0x007] Version,[31:24] -'V', [23:16] - Major, [15:8] - Minor, [7:0] - bugfix
    uint32_t reserved0;         //!< [0x008-0x00b] Reserved for future use
    uint8_t readSampleClkSrc;   //!< [0x00c-0x00c] Read Sample Clock Source, valid value: 0/1/3
    uint8_t dataHoldTime;       //!< [0x00d-0x00d] Data hold time, default value: 3
    uint8_t dataSetupTime;      //!< [0x00e-0x00e] Date setup time, default value: 3
    uint8_t columnAddressWidth; //!< [0x00f-0x00f] Column Address with, for HyperBus protocol, it is fixed to 3, For
    //! Serial NAND, need to refer to datasheet
    uint8_t deviceModeCfgEnable; //!< [0x010-0x010] Device Mode Configure enable flag, 1 - Enable, 0 - Disable
    uint8_t deviceModeType; //!< [0x011-0x011] Specify the configuration command type:Quad Enable, DPI/QPI/OPI switch,
    //! Generic configuration, etc.
    uint16_t waitTimeCfgCommands; //!< [0x012-0x013] Wait time for all configuration commands, unit: 100us, Used for
    //! DPI/QPI/OPI switch or reset command
    flexspi_lut_seq_t deviceModeSeq; //!< [0x014-0x017] Device mode sequence info, [7:0] - LUT sequence id, [15:8] - LUt
    //! sequence number, [31:16] Reserved
    uint32_t deviceModeArg;    //!< [0x018-0x01b] Argument/Parameter for device configuration
    uint8_t configCmdEnable;   //!< [0x01c-0x01c] Configure command Enable Flag, 1 - Enable, 0 - Disable
    uint8_t configModeType[3]; //!< [0x01d-0x01f] Configure Mode Type, similar as deviceModeTpe
    flexspi_lut_seq_t
        configCmdSeqs[3]; //!< [0x020-0x02b] Sequence info for Device Configuration command, similar as deviceModeSeq
    uint32_t reserved1;   //!< [0x02c-0x02f] Reserved for future use
    uint32_t configCmdArgs[3];     //!< [0x030-0x03b] Arguments/Parameters for device Configuration commands
    uint32_t reserved2;            //!< [0x03c-0x03f] Reserved for future use
    uint32_t controllerMiscOption; //!< [0x040-0x043] Controller Misc Options, see Misc feature bit definitions for more
    //! details
    uint8_t deviceType;    //!< [0x044-0x044] Device Type:  See Flash Type Definition for more details
    uint8_t sflashPadType; //!< [0x045-0x045] Serial Flash Pad Type: 1 - Single, 2 - Dual, 4 - Quad, 8 - Octal
    uint8_t serialClkFreq; //!< [0x046-0x046] Serial Flash Frequencey, device specific definitions, See System Boot
    //! Chapter for more details
    uint8_t lutCustomSeqEnable; //!< [0x047-0x047] LUT customization Enable, it is required if the program/erase cannot
    //! be done using 1 LUT sequence, currently, only applicable to HyperFLASH
    uint32_t reserved3[2];               //!< [0x048-0x04f] Reserved for future use
    uint32_t sflashA1Size;               //!< [0x050-0x053] Size of Flash connected to A1
    uint32_t sflashA2Size;               //!< [0x054-0x057] Size of Flash connected to A2
    uint32_t sflashB1Size;               //!< [0x058-0x05b] Size of Flash connected to B1
    uint32_t sflashB2Size;               //!< [0x05c-0x05f] Size of Flash connected to B2
    uint32_t csPadSettingOverride;       //!< [0x060-0x063] CS pad setting override value
    uint32_t sclkPadSettingOverride;     //!< [0x064-0x067] SCK pad setting override value
    uint32_t dataPadSettingOverride;     //!< [0x068-0x06b] data pad setting override value
    uint32_t dqsPadSettingOverride;      //!< [0x06c-0x06f] DQS pad setting override value
    uint32_t timeoutInMs;                //!< [0x070-0x073] Timeout threshold for read status command
    uint32_t commandInterval;            //!< [0x074-0x077] CS deselect interval between two commands
    flexspi_dll_time_t dataValidTime[2]; //!< [0x078-0x07b] CLK edge to data valid time for PORT A and PORT B
    uint16_t busyOffset;                 //!< [0x07c-0x07d] Busy offset, valid value: 0-31
    uint16_t busyBitPolarity; //!< [0x07e-0x07f] Busy flag polarity, 0 - busy flag is 1 when flash device is busy, 1 -
    //! busy flag is 0 when flash device is busy
    uint32_t lookupTable[64];           //!< [0x080-0x17f] Lookup table holds Flash command sequences
    flexspi_lut_seq_t lutCustomSeq[12]; //!< [0x180-0x1af] Customizable LUT Sequences
    uint32_t reserved4[4];              //!< [0x1b0-0x1bf] Reserved for future use
} flexspi_mem_config_t;

typedef enum _FlexSPIOperationType
{
    kFlexSpiOperation_Command, //!< FlexSPI operation: Only command, both TX and
    //! RX buffer are ignored.
    kFlexSpiOperation_Config, //!< FlexSPI operation: Configure device mode, the
    //! TX FIFO size is fixed in LUT.
    kFlexSpiOperation_Write, //!< FlexSPI operation: Write,  only TX buffer is
    //! effective
    kFlexSpiOperation_Read, //!< FlexSPI operation: Read, only Rx Buffer is
    //! effective.
    kFlexSpiOperation_End = kFlexSpiOperation_Read,
} flexspi_operation_t;

//!@brief FlexSPI Transfer Context
typedef struct _FlexSpiXfer
{
    flexspi_operation_t operation; //!< FlexSPI operation
    uint32_t baseAddress;          //!< FlexSPI operation base address
    uint32_t seqId;                //!< Sequence Id
    uint32_t seqNum;               //!< Sequence Number
    bool isParallelModeEnable;     //!< Is a parallel transfer
    uint32_t *txBuffer;            //!< Tx buffer
    uint32_t txSize;               //!< Tx size in bytes
    uint32_t *rxBuffer;            //!< Rx buffer
    uint32_t rxSize;               //!< Rx size in bytes
} flexspi_xfer_t;

//!@brief FlexSPI Clock Type
typedef enum
{
    kFlexSpiClock_CoreClock,       //!< ARM Core Clock
    kFlexSpiClock_AhbClock,        //!< AHB clock
    kFlexSpiClock_SerialRootClock, //!< Serial Root Clock
    kFlexSpiClock_IpgClock,        //!< IPG clock
} flexspi_clock_type_t;

//!@brief Generate bit mask
#define FLEXSPI_BITMASK(bit_offset) (1u << (bit_offset))

enum
{
    kStatusGroup_FLEXSPINOR = 201,
};

/* FlexSPI NOR status */
enum _flexspi_nor_status
{
    kStatus_FLEXSPINOR_ProgramFail = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 0), //!< Status for Page programming failure
    kStatus_FLEXSPINOR_EraseSectorFail = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 1), //!< Status for Sector Erase failure
    kStatus_FLEXSPINOR_EraseAllFail    = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 2), //!< Status for Chip Erase failure
    kStatus_FLEXSPINOR_WaitTimeout     = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 3), //!< Status for timeout
    kStatus_FlexSPINOR_NotSupported    = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 4), // Status for PageSize overflow
    kStatus_FlexSPINOR_WriteAlignmentError = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 5), //!< Status for Alignement error
    kStatus_FlexSPINOR_CommandFailure =
        MAKE_STATUS(kStatusGroup_FLEXSPINOR, 6), //!< Status for Erase/Program Verify Error
    kStatus_FlexSPINOR_SFDP_NotFound = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 7), //!< Status for SFDP read failure
    kStatus_FLEXSPINOR_Unsupported_SFDP_Version =
        MAKE_STATUS(kStatusGroup_FLEXSPINOR, 8), //!< Status for Unrecognized SFDP version
    kStatus_FLEXSPINOR_Flash_NotFound = MAKE_STATUS(kStatusGroup_FLEXSPINOR, 9), //!< Status for Flash detection failure
    kStatus_FLEXSPINOR_DTRRead_DummyProbeFailed =
        MAKE_STATUS(kStatusGroup_FLEXSPINOR, 10), //!< Status for DDR Read dummy probe failure
};

enum
{
    kSerialNorCfgOption_Tag                         = 0x0c,
    kSerialNorCfgOption_DeviceType_ReadSFDP_SDR     = 0,
    kSerialNorCfgOption_DeviceType_ReadSFDP_DDR     = 1,
    kSerialNorCfgOption_DeviceType_HyperFLASH1V8    = 2,
    kSerialNorCfgOption_DeviceType_HyperFLASH3V0    = 3,
    kSerialNorCfgOption_DeviceType_MacronixOctalDDR = 4,
    kSerialNorCfgOption_DeviceType_MacronixOctalSDR = 5,
    kSerialNorCfgOption_DeviceType_MicronOctalDDR   = 6,
    kSerialNorCfgOption_DeviceType_MicronOctalSDR   = 7,
    kSerialNorCfgOption_DeviceType_AdestoOctalDDR   = 8,
    kSerialNorCfgOption_DeviceType_AdestoOctalSDR   = 9,
};

enum
{
    kSerialNorQuadMode_NotConfig            = 0,
    kSerialNorQuadMode_StatusReg1_Bit6      = 1,
    kSerialNorQuadMode_StatusReg2_Bit1      = 2,
    kSerialNorQuadMode_StatusReg2_Bit7      = 3,
    kSerialNorQuadMode_StatusReg2_Bit1_0x31 = 4,
};

enum
{
    kSerialNorEnhanceMode_Disabled         = 0,
    kSerialNorEnhanceMode_0_4_4_Mode       = 1,
    kSerialNorEnhanceMode_0_8_8_Mode       = 2,
    kSerialNorEnhanceMode_DataOrderSwapped = 3,
    kSerialNorEnhanceMode_2ndPinMux        = 4,
};

/*
 * Serial NOR Configuration Option
 */
typedef struct _serial_nor_config_option
{
    union
    {
        struct
        {
            uint32_t max_freq : 4;          //!< Maximum supported Frequency
            uint32_t misc_mode : 4;         //!< miscellaneous mode
            uint32_t quad_mode_setting : 4; //!< Quad mode setting
            uint32_t cmd_pads : 4;          //!< Command pads
            uint32_t query_pads : 4;        //!< SFDP read pads
            uint32_t device_type : 4;       //!< Device type
            uint32_t option_size : 4;       //!< Option size, in terms of uint32_t, size = (option_size + 1) * 4
            uint32_t tag : 4;               //!< Tag, must be 0x0E
        } B;
        uint32_t U;
    } option0;

    union
    {
        struct
        {
            uint32_t dummy_cycles : 8;     //!< Dummy cycles before read
            uint32_t status_override : 8;  //!< Override status register value during device mode configuration
            uint32_t is_pinmux_group2 : 4; //!< The second group of pinmux
            uint32_t reserved : 12;        //!< Reserved for future use
        } B;
        uint32_t U;
    } option1;

} serial_nor_config_option_t;

/*
 *  Serial NOR configuration block
 */
typedef struct _flexspi_nor_config
{
    flexspi_mem_config_t memConfig; //!< Common memory configuration info via FlexSPI
    uint32_t pageSize;              //!< Page size of Serial NOR
    uint32_t sectorSize;            //!< Sector size of Serial NOR
    uint8_t ipcmdSerialClkFreq;     //!< Clock frequency for IP command
    uint8_t isUniformBlockSize;     //!< Sector/Block size is the same
    uint8_t isDataOrderSwapped;     //!< Data order (D0, D1, D2, D3) is swapped (D1,D0, D3, D2)
    uint8_t reserved0[1];           //!< Reserved for future use
    uint8_t serialNorType;          //!< Serial NOR Flash type: 0/1/2/3
    uint8_t needExitNoCmdMode;      //!< Need to exit NoCmd mode before other IP command
    uint8_t halfClkForNonReadCmd;   //!< Half the Serial Clock for non-read command: true/false
    uint8_t needRestoreNoCmdMode;   //!< Need to Restore NoCmd mode after IP commmand execution
    uint32_t blockSize;             //!< Block size
    uint32_t reserve2[11];          //!< Reserved for future use
} flexspi_nor_config_t;

#ifdef __cplusplus
extern "C" {
#endif

//!@brief Configure FlexSPI Lookup table
status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);

//!@brief Perform FlexSPI command
status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer);

//!@brief Clear FlexSPI cache
void flexspi_clear_cache(uint32_t instance);

//!@brief Initialize Serial NOR devices via FlexSPI
status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config);

//!@brief Program data to Serial NOR via FlexSPI
status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src);

//!@brief Erase all the Serial NOR devices connected on FlexSPI
status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config);

//!@brief Get FlexSPI NOR Configuration Block based on specified option
status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);

//!@brief Erase Flash Region specified by address and length
status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length);

//!@brief Read data from Serial NOR
status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);

#ifdef __cplusplus
}
#endif

#endif // __FLEXSPI_NOR_FLASH_H__
