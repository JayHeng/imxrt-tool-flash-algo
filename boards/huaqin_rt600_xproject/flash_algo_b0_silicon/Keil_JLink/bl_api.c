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
#include <string.h>
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

status_t otp_shadow_register_reload(void)
{
    return g_bootloaderTree->otpDriver->reload();
}

status_t otp_crc_check(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr)
{
    return g_bootloaderTree->otpDriver->crc_check(start_addr, end_addr, crc_addr);
}

status_t otp_crc_calc(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum)
{
    return g_bootloaderTree->otpDriver->crc_calc(src, numberOfWords, crcChecksum);
}


/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
uint32_t flexspi_nor_driver_get_version(void)
{
    return g_bootloaderTree->flexspiNorDriver->version;
}

status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexspiNorDriver->init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    return g_bootloaderTree->flexspiNorDriver->page_program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexspiNorDriver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    return g_bootloaderTree->flexspiNorDriver->get_config(instance, config, option);
}

status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return g_bootloaderTree->flexspiNorDriver->erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_bootloaderTree->flexspiNorDriver->read(instance, config, dst, start, bytes);
}

status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq)
{
    return g_bootloaderTree->flexspiNorDriver->update_lut(instance, seqIndex, lutBase, numberOfSeq);
}

status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    return g_bootloaderTree->flexspiNorDriver->xfer(instance, xfer);
}

status_t flexspi_nor_set_clock_source(uint32_t clockSrc)
{
    return g_bootloaderTree->flexspiNorDriver->set_clock_source(clockSrc);
}

void flexspi_nor_flash_config_clock(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode)
{
    g_bootloaderTree->flexspiNorDriver->config_clock(instance, freqOption, sampleClkMode);
}

status_t flexspi_nor_auto_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    // Wait until the FLEXSPI is idle
    register uint32_t delaycnt = 10000u;
    while(delaycnt--)
    {
    }
    status_t status = g_bootloaderTree->flexspiNorDriver->get_config(instance, config, option);
    if (status != 0)
    {
        return status;
    }

    return g_bootloaderTree->flexspiNorDriver->init(instance, config);

    // Note: Below are the Position-independent codes for above the flow.
    // static uint8_t s_ramfuncArray[] = { 0xF8, 0xB5, 0x0D, 0x46, 0x04, 0x46, 0x42, 0xF2, 0x10, 0x71, 0x08,
                                        // 0x46, 0x41, 0x1E, 0x00, 0x28, 0xFB, 0xD1, 0x4F, 0xF6, 0x1C, 0x76,
                                        // 0xC1, 0xF2, 0x01, 0x36, 0x33, 0x68, 0x29, 0x46, 0x20, 0x46, 0xDF,
                                        // 0x69, 0xB8, 0x47, 0x30, 0xB9, 0x32, 0x68, 0x29, 0x46, 0x20, 0x46,
                                        // 0x53, 0x68, 0xBD, 0xE8, 0xF4, 0x40, 0x18, 0x47, 0xF2, 0xBD };

    // uint32_t alignedBuffer[(sizeof(s_ramfuncArray) + 3) / sizeof(uint32_t)];
    // memcpy(alignedBuffer, s_ramfuncArray, sizeof(s_ramfuncArray));
    // static status_t (*autoConfigCallback)(uint32_t, flexspi_nor_config_t *, serial_nor_config_option_t *);
    // autoConfigCallback =
        // (status_t (*)(uint32_t, flexspi_nor_config_t *, serial_nor_config_option_t *))((uint32_t)&alignedBuffer + 1);

    // return autoConfigCallback(instance, config, option);
}
