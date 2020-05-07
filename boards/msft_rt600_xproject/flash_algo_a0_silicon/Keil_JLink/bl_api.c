/*
 * The Clear BSD License
 * Copyright 2018 NXP
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
#include "bl_api.h"



/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Codes
 ******************************************************************************/

void bl_api_init(void)
{
}

/*******************************************************************************
 * OTP driver
 ******************************************************************************/
status_t otp_init(uint32_t src_clk_freq)
{
    return g_bootloaderTree->otpDriver->init(src_clk_freq);
}

status_t otp_deinit(void)
{
    return g_bootloaderTree->otpDriver->deinit();
}

status_t otp_fuse_read(uint32_t addr, uint32_t *data)
{
    return g_bootloaderTree->otpDriver->fuse_read(addr, data);
}

status_t otp_fuse_program(uint32_t addr, uint32_t data, bool lock)
{
    return g_bootloaderTree->otpDriver->fuse_program(addr, data, lock);
}

status_t otp_crc_calc(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum)
{
    return g_bootloaderTree->otpDriver->crc_calc(src, numberOfWords, crcChecksum);
}

status_t otp_shadow_register_reload(void)
{
    return g_bootloaderTree->otpDriver->reload();
}

status_t otp_crc_check(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr)
{
    return g_bootloaderTree->otpDriver->crc_check(start_addr, end_addr, crc_addr);
}


/*******************************************************************************
 * QSPI NOR driver
 ******************************************************************************/
status_t quadspi_nor_init(quadspi_nor_config_t *config)
{
    return g_bootloaderTree->qspiNorDriver->init(config);
}

status_t quadspi_nor_set_api_property(quadspi_nor_config_t *config, uint32_t property_tag, uint32_t property)
{
    return g_bootloaderTree->qspiNorDriver->set_property( config, property_tag, property);
}

status_t quadspi_nor_page_program(quadspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src)
{
    return g_bootloaderTree->qspiNorDriver->page_program(config, dstAddr, src);
}

status_t quadspi_nor_erase_all(quadspi_nor_config_t *config)
{
    return g_bootloaderTree->qspiNorDriver->erase_all(config);
}

status_t quadspi_nor_wait_busy(quadspi_nor_config_t *config, bool isParallelMode, uint32_t baseAddr)
{
   return g_bootloaderTree->qspiNorDriver->wait_busy(config, isParallelMode, baseAddr);
}

status_t quadspi_nor_erase(quadspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return g_bootloaderTree->qspiNorDriver->erase(config, start, length);
}

status_t quadspi_nor_erase_sector(quadspi_nor_config_t *config, uint32_t address)
{
    return g_bootloaderTree->qspiNorDriver->erase_sector(config, address);
}

status_t quadspi_nor_erase_block(quadspi_nor_config_t *config, uint32_t address)
{
     return g_bootloaderTree->qspiNorDriver->erase_block(config, address);
}

status_t quadspi_nor_get_config(quadspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    return g_bootloaderTree->qspiNorDriver->get_config(config, option);
}

status_t quadspi_nor_read(quadspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_bootloaderTree->qspiNorDriver->read(config, dst, start, bytes);
}

status_t quadspi_nor_hw_reset(quadspi_nor_config_t *config)
{
    return g_bootloaderTree->qspiNorDriver->hw_reset(config);
}

status_t quadspi_command_xfer(quadspi_xfer_t *xfer)
{
    return g_bootloaderTree->qspiNorDriver->xfer(xfer);
}

status_t quadspi_update_lut(uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq)
{
    return g_bootloaderTree->qspiNorDriver->update_lut(seqIndex, lutBase, numberOfSeq);
}


