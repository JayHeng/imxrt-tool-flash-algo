/*
 * The Clear BSD License
 * Copyright 2017-2018 NXP
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

#ifndef __QUADSPI_NOR_FLASH_H__
#define __QUADSPI_NOR_FLASH_H__

#include "fsl_quadspi.h"

/*  */
#define NOR_CMD_INDEX_READ CMD_INDEX_READ               //!< 0
#define NOR_CMD_INDEX_READSTATUS CMD_INDEX_READSTATUS   //!< 1
#define NOR_CMD_INDEX_WRITEENABLE CMD_INDEX_WRITEENABLE //!< 2
#define NOR_CMD_INDEX_ERASESECTOR 3                     //!< 3
#define NOR_CMD_INDEX_PAGEPROGRAM CMD_INDEX_WRITE       //!< 4
#define NOR_CMD_INDEX_CHIPERASE 5                       //!< 5
#define NOR_CMD_INDEX_DUMMY 6                           //!< 6
#define NOR_CMD_INDEX_ERASEBLOCK 7                      //!< 7

#define NOR_CMD_LUT_SEQ_IDX_READ CMD_LUT_SEQ_IDX_READ //!< 0  READ LUT sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS \
    CMD_LUT_SEQ_IDX_READSTATUS //!< 1  Read Status LUT sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS_XPI \
    2 //!< 2  Read status DPI/QPI/OPI sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE \
    CMD_LUT_SEQ_IDX_WRITEENABLE //!< 3  Write Enable sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE_XPI \
    4 //!< 4  Write Enable DPI/QPI/OPI sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR 5 //!< 5  Erase Sector sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_ERASEBLOCK 8  //!< 8 Erase Block sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM \
    CMD_LUT_SEQ_IDX_WRITE                //!< 9  Program sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_CHIPERASE 11 //!< 11 Chip Erase sequence in lookupTable id stored in config block
#define NOR_CMD_LUT_SEQ_IDX_READ_SFDP 13 //!< 13 Read SFDP sequence in lookupTable id stored in config block
#define NOR_CMD_LUT_SEQ_IDX_RESTORE_NOCMD \
    14 //!< 14 Restore 0-4-4/0-8-8 mode sequence id in lookupTable stored in config block
#define NOR_CMD_LUT_SEQ_IDX_EXIT_NOCMD \
    15 //!< 15 Exit 0-4-4/0-8-8 mode sequence id in lookupTable stored in config blobk

enum
{
    kSerialNorCfgOption_Tag = 0x0C,
    kSerialNorCfgOption_DeviceType_ReadSFDP_SDR = 0,
    kSerialNorCfgOption_DeviceType_ReadSFDP_DDR = 1,
    kSerialNorCfgOption_DeviceType_HyperFLASH1V8 = 2,
    kSerialNorCfgOption_DeviceType_HyperFLASH3V0 = 3,
    kSerialNorCfgOption_DeviceType_MacronixOctalDDR = 4,
    kSerialNorCfgOption_DeviceType_MacronixOctalSDR = 5,
    kSerialNorCfgOption_DeviceType_MicronOctalDDR = 6,
    kSerialNorCfgOption_DeviceType_MicronOctalSDR = 7,
    kSerialNorCfgOption_DeviceType_AdestoOctalDDR = 8,
    kSerialNorCfgOption_DeviceType_AdestoOctalSDR = 9,
};

enum
{
    kSerialNorQuadMode_NotConfig = 0,
    kSerialNorQuadMode_StatusReg1_Bit6 = 1,
    kSerialNorQuadMode_StatusReg2_Bit1 = 2,
    kSerialNorQuadMode_StatusReg2_Bit7 = 3,
    kSerialNorQuadMode_StatusReg2_Bit1_0x31 = 4,
};

enum
{
    kSerialNorEnhanceMode_Disabled = 0,
    kSerialNorEnhanceMode_0_4_4_Mode = 1,
    kSerialNorEnhanceMode_0_8_8_Mode = 2,
    kSerialNorEnhanceMode_DataOrderSwapped = 3,
    kSerialNorEnhanceMode_2ndPinMux = 4,
};

enum
{
    kSerialNorType_StandardSPI, //!< Device that support Standard SPI and Extended SPI mode
    kSerialNorType_HyperBus,    //!< Device that supports HyperBus only
    kSerialNorType_XPI,         //!< Device that works under DPI, QPI or OPI mode
    kSerialNorType_NoCmd, //!< Device that works under No command mode (XIP mode/Performance Enhance mode/continous read
    //! mode)
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
            uint32_t tag : 4;               //!< Tag, must be 0x0C
        } B;
        uint32_t U;
    } option0;

    union
    {
        struct
        {
            uint32_t dummy_cycles : 8;    //!< Dummy cycles before read
            uint32_t status_override : 8; //!< Override status register value during device mode configuration
            uint32_t reserved : 16;       //!< Reserved for future use
        } B;
        uint32_t U;
    } option1;

} serial_nor_config_option_t;

typedef union
{
    struct
    {
        uint8_t por_mode;
        uint8_t current_mode;
        uint8_t exit_no_cmd_sequence;
        uint8_t restore_sequence;
    } B;
    uint32_t U;
} flash_run_context_t;

enum
{
    kRestoreSequence_None = 0,
    kRestoreSequence_HW_Reset = 1,
    kRestoreSequence_4QPI_FF = 2,
    kRestoreSequence_5QPI_FF = 3,
    kRestoreSequence_8QPI_FF = 4,
    kRestoreSequence_Send_F0 = 5,
    kRestoreSequence_Send_66_99 = 6,
    kRestoreSequence_Send_6699_9966 = 7,
    kRestoreSequence_Send_06_FF, // Adesto EcoXIP
};

enum
{
    kFlashInstMode_ExtendedSpi = 0x00,
    kFlashInstMode_0_4_4_SDR = 0x01,
    kFlashInstMode_0_4_4_DDR = 0x02,
    kFlashInstMode_QPI_SDR = 0x41,
    kFlashInstMode_QPI_DDR = 0x42,
    kFlashInstMode_OPI_SDR = 0x81,
    kFlashInstMode_OPI_DDR = 0x82,
};

enum
{
    kFlashResetLogic_Disabled = 0,
    kFlashResetLogic_ResetPin = 1,
    kFlashResetLogic_JedecHwReset = 2,
};

/* QuadSPI NOR status */
enum _quadspi_nor_status
{
    kStatusGroup_QUADSPINOR = 4,
    kStatus_QuadSpiNOR_FlashSizeError =
        MAKE_STATUS(kStatusGroup_QUADSPINOR, 0), //!< Status for error QuadSPI NOR Flash size
    kStatus_QuadSpiNOR_WriteAlignmentError = MAKE_STATUS(kStatusGroup_QUADSPINOR, 1), //!< Status for Alignement error
    kStatus_QuadSpiNOR_AddressError = MAKE_STATUS(kStatusGroup_QUADSPINOR, 2),
    kStatus_QuadSpiNOR_CommandFailure =
        MAKE_STATUS(kStatusGroup_QUADSPINOR, 3), //!< Status for Erase/Program Verify Error
    kStatus_QuadSpiNOR_UnknownProperty = MAKE_STATUS(kStatusGroup_QUADSPINOR, 4),
    kStatus_QuadSpiNOR_NotConfigured = MAKE_STATUS(kStatusGroup_QUADSPINOR, 5),
    kStatus_QuadSpiNOR_NotSupported = MAKE_STATUS(kStatusGroup_QUADSPINOR, 6), // Status for PageSize overflow
    kStatus_QuadSpiNOR_CommandTimout = MAKE_STATUS(kStatusGroup_QUADSPINOR, 7),
    kStatus_QuadSpiNOR_ProgramFail = MAKE_STATUS(kStatusGroup_QUADSPINOR, 8), //!< Status for Page programming failure
    kStatus_QuadSpiNOR_Busy = MAKE_STATUS(kStatusGroup_QUADSPINOR, 9),
    kStatus_QuadSpiNOR_EraseSectorFail = MAKE_STATUS(kStatusGroup_QUADSPINOR, 10), //!< Status for Sector Erase failure
    kStatus_QuadSpiNOR_EraseAllFail = MAKE_STATUS(kStatusGroup_QUADSPINOR, 11),    //!< Status for Chip Erase failure
    kStatus_QuadSpiNOR_WaitTimeout = MAKE_STATUS(kStatusGroup_QUADSPINOR, 12),     //!< Status for timeout

    kStatus_QuadSpiNOR_SFDP_NotFound = MAKE_STATUS(kStatusGroup_QUADSPINOR, 13), //!< Status for SFDP read failure
    kStatus_QuadSpiNOR_Unsupported_SFDP_Version =
        MAKE_STATUS(kStatusGroup_QUADSPINOR, 14), //!< Status for Unrecognized SFDP version
    kStatus_QuadSpiNOR_Flash_NotFound =
        MAKE_STATUS(kStatusGroup_QUADSPINOR, 15), //!< Status for Flash detection failure
    kStatus_QuadSpiNOR_DTRRead_DummyProbeFailed =
        MAKE_STATUS(kStatusGroup_QUADSPINOR, 16), //!< Status for DDR Read dummy probe failure
    kStatus_QuadSpiNOR_UnsupportedOnNonBlockingMode = MAKE_STATUS(kStatusGroup_QUADSPINOR, 17),
};

/*
 *  Serial NOR configuration block
 */
typedef struct _quadspi_nor_config
{
    quadspi_mem_config_t memConfig; //!< Common memory configuration info via QuadSPI
    uint32_t pageSize;              //!< Page size of Serial NOR
    uint32_t sectorSize;            //!< Sector size of Serial NOR
    uint8_t ipcmdSerialClkFreq;     //!< Clock frequency for IP command
    uint8_t isUniformBlockSize;     //!< Sector/Block size is the same
    uint8_t isDataOrderSwapped;     //!< Data order (D0, D1, D2, D3) is swapped (D1,D0, D3, D2)
    uint8_t resetLogic;             //!< Reset Logic: 0 - No reset, 1 - Reset Pin, 2 - JEDEC HW reset
    uint8_t serialNorType;          //!< Serial NOR Flash type: 0/1/2/3
    uint8_t needExitNoCmdMode;      //!< Need to exit NoCmd mode before other IP command
    uint8_t halfClkForNonReadCmd;   //!< Half the Serial Clock for non-read command: true/false
    uint8_t needRestoreNoCmdMode;   //!< Need to Restore NoCmd mode after IP commmand execution
    uint32_t blockSize;             //!< Block size
    uint8_t isNonBlockingMode;      //!< Non-blocking mode flag
    uint8_t reserved0[3];
    uint32_t reserved1[10]; //!< Reserved for future use
} quadspi_nor_config_t;

enum
{
    kQuadSpiNorApiPropertyTag_Start = 0,
    kQuadSpiNorApiPropertyTag_ReadClock = kQuadSpiNorApiPropertyTag_Start,
    kQuadSpiNorApiPropertyTag_NonReadClock,
    kQuadSpiNorApiPropertyTag_NonBlockAccess,
    kQuadSpiNorApiPropertyTag_End = kQuadSpiNorApiPropertyTag_NonBlockAccess,
};

#ifdef __cplusplus
extern "C" {
#endif

//!@brief Initialize Serial NOR devices via QuadSPI
status_t quadspi_nor_init(quadspi_nor_config_t *config);

//!@brief Program data to Serial NOR via QuadSPI
status_t quadspi_nor_page_program(quadspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src);

//!@brief Erase all the Serial NOR devices connected on QuadSPI
status_t quadspi_nor_erase_all(quadspi_nor_config_t *config);

//!@brief Wait until QuadSPI NOR Flash is not busy
status_t quadspi_nor_wait_busy(quadspi_nor_config_t *config, bool isParallelMode, uint32_t address);

//!@brief Erase one sector specified by address
status_t quadspi_nor_erase_sector(quadspi_nor_config_t *config, uint32_t address);

//!@brief Erase one block specified by address
status_t quadspi_nor_erase_block(quadspi_nor_config_t *config, uint32_t address);

//!@brief Get QuadSPI NOR Configuration Block based on specified option
status_t quadspi_nor_get_config(quadspi_nor_config_t *config, serial_nor_config_option_t *option);

//!@brief Erase Flash Region specified by address and length
status_t quadspi_nor_erase(quadspi_nor_config_t *config, uint32_t start, uint32_t length);

//!@brief Read data from Serial NOR
status_t quadspi_nor_read(quadspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);

//!@brief Reset QuadSPI NOR Flash
extern status_t quadspi_nor_hw_reset(quadspi_nor_config_t *config);

//!@brief Write QuadSPI persistent content
extern status_t quadspi_nor_write_persistent(const uint32_t data);

//!@brief Read QuadSPI persistent content
extern status_t quadspi_nor_read_persistent(uint32_t *data);

//!@brief Wait until QuadSPI NOR powers up
extern void quadspi_nor_wait_powerup(void);

//!@brief Restore Flash to SPI protocol
status_t quadspi_nor_restore_spi_protocol(quadspi_nor_config_t *config, flash_run_context_t *run_ctx);

//!@brief Set QuadSPI API property
status_t quadspi_nor_set_api_property(quadspi_nor_config_t *config, uint32_t property_tag, uint32_t property);

#ifdef __cplusplus
}
#endif

#endif // __QUADSPI_NOR_FLASH_H__
