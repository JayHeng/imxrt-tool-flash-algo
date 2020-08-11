/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef __BL_API_H__
#define __BL_API_H__

#include "fsl_device_registers.h"
#include "flexspi_nor_flash.h"

typedef struct
{
    uint32_t version;
    status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*page_program)(uint32_t instance, flexspi_nor_config_t *config, uint32_t dstAddr, const uint32_t *src);
    status_t (*erase_all)(uint32_t instance, flexspi_nor_config_t *config);
    status_t (*erase)(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length);
    status_t (*read)(uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes);
    void (*clear_cache)(uint32_t instance);
    status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq);
    status_t (*get_config)(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option);
    status_t (*erase_sector)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    status_t (*erase_block)(uint32_t instance, flexspi_nor_config_t *config, uint32_t address);
    void (*hw_reset)(uint32_t instance, uint32_t resetLogic);
    status_t (*wait_busy)(uint32_t instance, flexspi_nor_config_t *config, bool isParallelMode, uint32_t address);
    status_t (*set_clock_source)(uint32_t instance, uint32_t clockSrc);
    void (*config_clock)(uint32_t instance, uint32_t freqOption, uint32_t sampleClkMode);

} flexspi_nor_flash_driver_t;

typedef struct
{
    void (*runBootloader)(void *arg);                   //!< Function to start the bootloader executing
    const uint32_t version;                             //!< Bootloader version number
    const char *copyright;                              //!< Bootloader Copyright
    const flexspi_nor_flash_driver_t *flexSpiNorDriver; //!< FlexSPI NOR Flash API
} bootloader_api_entry_t;

enum
{
    kEnterBootloader_Tag                   = 0xEB,
    kEnterBootloader_Mode_Default          = 0,
    kEnterBootloader_Mode_SerialDownloader = 1,

    kEnterBootloader_SerialInterface_Auto = 0,
    kEnterBootloader_SerialInterface_USB  = 1,
    kEnterBootloader_SerialInterface_UART = 2,

    kEnterBootloader_ImageIndex_Max = 3,
};

typedef union
{
    struct
    {
        uint32_t imageIndex : 4;
        uint32_t reserved : 12;
        uint32_t serialBootInterface : 4;
        uint32_t bootMode : 4;
        uint32_t tag : 8;
    } B;
    uint32_t U;
} run_bootloader_ctx_t;

/* ROM API Tree address */
#define g_bootloaderTree (*(bootloader_api_entry_t **)0x0020001c)

static inline void runBootloader(run_bootloader_ctx_t *ctx)
{
    g_bootloaderTree->runBootloader(ctx);
}

#endif //__BL_API_H__
