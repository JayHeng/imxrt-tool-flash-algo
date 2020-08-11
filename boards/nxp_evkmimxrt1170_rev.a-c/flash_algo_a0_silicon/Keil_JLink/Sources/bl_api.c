/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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
 * FlexSPI NOR driver
 ******************************************************************************/
status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    return g_bootloaderTree->flexSpiNorDriver->page_program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    return g_bootloaderTree->flexSpiNorDriver->get_config(instance, config, option);
}

status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return g_bootloaderTree->flexSpiNorDriver->erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_bootloaderTree->flexSpiNorDriver->read(instance, config, dst, start, bytes);
}

status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq)
{
    return g_bootloaderTree->flexSpiNorDriver->update_lut(instance, seqIndex, lutBase, numberOfSeq);
}

status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    return g_bootloaderTree->flexSpiNorDriver->xfer(instance, xfer);
}

void flexspi_clear_cache(uint32_t instance)
{
    g_bootloaderTree->flexSpiNorDriver->clear_cache(instance);
}

status_t flexspi_update_clock_source(uint32_t instance, uint32_t source)
{
    return g_bootloaderTree->flexSpiNorDriver->set_clock_source(instance, source);
}
