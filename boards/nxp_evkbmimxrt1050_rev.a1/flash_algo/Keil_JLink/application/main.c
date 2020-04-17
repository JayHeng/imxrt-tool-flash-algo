/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "flexspi_nor/flexspi_nor_flash.h"

#define FLEXSPI_INSTANCE 0

#define FLEXSPI_AMBA_BASE 0x60000000u

extern void clock_init();
void error_handler(status_t status);

void main(void)
{
    SCB_DisableDCache();

    clock_init();

    serial_nor_config_option_t nor_config_option;
    flexspi_nor_config_t flexspi_nor_config;

    /*
     * Typical Options:
     * QuadSPI NOR - Quad SDR Read: option0 = 0xc0000006 (100MHz)
     * QuadSPI NOR - Quad DDR Read: option0 = 0xc0100003 (60MHz)
     * HyperFLASH 1V8: option0 = 0xc0233007 (133MHz)
     * MXIC OPI DDR: option0=0xc0403006 (100MHz), SPI->OPI DDR
     * MXIC OPI DDR (OPI DDR enabled by default): option=0xc0433006(100MHz)
     * Micron Octal DDR: option0=0xc0600006 (100MHz)
     * Micron OPI DDR: option0=0xc0603006 (100MHz), SPI->OPI DDR
     * Micron OPI DDR (DDR read enabled by default): option0=0xc0633006(100MHz)
     * Adesto OPI DDR: option0=0xc0803007(133MHz), SPI->OPI DDR
     *
     */
    nor_config_option.option0.U = 0xc0233007;

    status_t status = flexspi_nor_get_config(FLEXSPI_INSTANCE, &flexspi_nor_config, &nor_config_option);
    if (status != kStatus_Success)
    {
        error_handler(status);
    }

    status = flexspi_nor_flash_init(FLEXSPI_INSTANCE, &flexspi_nor_config);
    if (status != kStatus_Success)
    {
        error_handler(status);
    }

    // Erase: Assume erase the first 16KB, start from 0x6000_0000u
    uint32_t actualAddress = 0x60000000u - FLEXSPI_AMBA_BASE;
    status                 = flexspi_nor_flash_erase(FLEXSPI_INSTANCE, &flexspi_nor_config, actualAddress, 16u * 1024u);
    if (status != kStatus_Success)
    {
        error_handler(status);
    }

    // Program
    uint32_t buf[256 / sizeof(uint32_t)];
    uint8_t *byte_stream = (uint8_t *)buf;
    for (uint32_t i = 0; i < sizeof(buf); i++)
    {
        *byte_stream++ = (uint8_t)(i & 0xFF);
    }
    status = flexspi_nor_flash_page_program(FLEXSPI_INSTANCE, &flexspi_nor_config, actualAddress, buf);
    if (status != kStatus_Success)
    {
        error_handler(status);
    }

    // Read: There are two kinds of Read, AHB read or IPS read
    // 1. IP read
    uint32_t read_buf[256 / sizeof(uint32_t)];
    status = flexspi_nor_flash_read(FLEXSPI_INSTANCE, &flexspi_nor_config, read_buf, actualAddress, sizeof(read_buf));
    if (status != kStatus_Success)
    {
        error_handler(status);
    }

    // 2. AHB read
    actualAddress += FLEXSPI_AMBA_BASE;
    memcpy(read_buf, (uint32_t *)actualAddress, sizeof(read_buf));

    while (1)
    {
    }
}

void error_handler(status_t status)
{
    while (1)
        ;
}
