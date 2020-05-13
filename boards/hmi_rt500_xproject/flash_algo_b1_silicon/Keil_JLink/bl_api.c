/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "bl_api.h"
#include <string.h>

/*******************************************************************************
 * Definition
 ******************************************************************************/
#define BOOTLOADER_TREE_LOCATION    (0x1302f000u)
#define g_bootloaderTree    ((bootloader_tree_t*)BOOTLOADER_TREE_LOCATION)


/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
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
    volatile register uint32_t delaycnt = 10000u;
    while(delaycnt--)
    {
    }
    status_t status = g_bootloaderTree->flexspiNorDriver->get_config(instance, config, option);
    if (status != 0)
    {
        return status;
    }

    return g_bootloaderTree->flexspiNorDriver->init(instance, config);
}

/*******************************************************************************
 * FlexSPI NOR driver
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
 * ReInvoke ROM API
 ******************************************************************************/
void bootloader_user_entry(void *arg)
{
    g_bootloaderTree->runBootloader(arg);
}

