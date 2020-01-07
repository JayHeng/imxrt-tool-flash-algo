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
#include "flexspi_nor_flash.h"


////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//!@brief FLEXSPI Flash driver API Interface
typedef struct
{
    uint32_t version;
    status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*page_program)(uint32_t instance, flexspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
    //    status_t (*wait_busy)(uint32_t instance, flexspi_nor_config_t *config, bool isParallelMode, uint32_t address);
    status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length);
    status_t (*erase_sector)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    status_t (*erase_block)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    status_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);
    status_t (*read)(uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);
    //    status_t (*hw_reset)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);
    status_t (*set_clock_source)(uint32_t clockSrc);
    void (*config_clock)(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode);
} flexspi_nor_flash_driver_t;

//!@brief OTP driver API Interface
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

//! @brief Root of the bootloader API tree.
//!
//! An instance of this struct resides in read-only memory in the bootloader. It
//! provides a user application access to APIs exported by the bootloader.
//!
//! @note The order of existing fields must not be changed.
//!
//! @ingroup context
typedef struct BootloaderTree
{
    void (*runBootloader)(void *arg); //!< Function to start the bootloader executing.
    uint32_t version;                 //!< Bootloader version number.
    const char *copyright;            //!< Copyright string.
    const uint32_t reserved0;
    const uint32_t reserved1;
    const uint32_t reserved2;
    const uint32_t reserved3;
    const flexspi_nor_flash_driver_t *flexspiNorDriver; //!< FlexSPI NOR FLASH Driver API.
    const ocotp_driver_t *otpDriver;                    //!< OTP driver API.
    const uint32_t reserved4;
} bootloader_tree_t;


/* ROM API Tree address */
#define g_bootloaderTree ((bootloader_tree_t*)(0x1303f000))


#if defined(__cplusplus)
extern "C" {
#endif

#if defined(__cplusplus)
}
#endif

