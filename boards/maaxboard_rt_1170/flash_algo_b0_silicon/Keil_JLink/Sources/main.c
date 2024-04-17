/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include "bl_api.h"
#include <stdio.h>

/***********************************************************************************************************************
 *
 *  Definitions
 *
 **********************************************************************************************************************/
#define FLEXSPI_INSTANCE (1)
#define FLEXSPI2_INSTANCE (2)

#define FLASH_BASE_ADDR 0x30000000
#define FLASH_INSTANCE FLEXSPI_INSTANCE

#define TEST_OFFSET 0x1000
#define TEST_LENGTH 0x100
#define CONFIG_OPTION 0xc0000005

/***********************************************************************************************************************
 *
 *  Variables
 *
 **********************************************************************************************************************/
flexspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;
uint32_t programBuffer[64];
uint32_t verifyBuffer[64];

/***********************************************************************************************************************
 *
 *  Prototypes
 *
 **********************************************************************************************************************/

/***********************************************************************************************************************
 *
 *  Codes
 *
 **********************************************************************************************************************/

void main(void)
{
    uint32_t i = 0;

    uint8_t *pBufStart = (uint8_t *)programBuffer;
    for (uint32_t i = 0; i < sizeof(programBuffer); i++)
    {
        *pBufStart++ = (uint8_t)(i & 0xFF);
    }

    configOption.option0.U = CONFIG_OPTION;

    status_t status = flexspi_nor_get_config(FLASH_INSTANCE, &flashConfig, &configOption);
    if (status != kStatus_Success)
    {
        // PRINTF("Flash get configuration failed\r\n");
        return;
    }
    if (status == kStatus_Success)
    {
        // PRINTF("Flash get configuration passed\r\n");
    }

    status = flexspi_nor_flash_init(FLASH_INSTANCE, &flashConfig);
    if (status != kStatus_Success)
    {
        // PRINTF("Flash initialization failed\r\n");
        return;
    }
    else
    {
        // PRINTF("Flash initialization passed\r\n");
    }

    status = flexspi_nor_flash_erase(FLASH_INSTANCE, &flashConfig, TEST_OFFSET, TEST_LENGTH);
    if (status != kStatus_Success)
    {
        // PRINTF("Flash erase failed\r\n");
        return;
    }
    else
    {
        // PRINTF("Flash erase passed\r\n");
    }

    flexspi_nor_flash_read(FLASH_INSTANCE, &flashConfig, verifyBuffer, TEST_OFFSET, sizeof(programBuffer));
    bool hasErased = true;
    for (uint32_t i = 0; i < ARRAY_SIZE(verifyBuffer); i++)
    {
        if (verifyBuffer[i] != 0xffffffffu)
        {
            hasErased = false;
            break;
        }
    }
    if (hasErased)
    {
        // PRINTF("Flash erase verify passed\r\n");
    }
    else
    {
        // PRINTF("Flash erase verify failed\r\n");
        return;
    }

    for (i = 0; i < 0x40U; i++)
    {
        programBuffer[i] = i;
    }

    status = flexspi_nor_flash_page_program(FLASH_INSTANCE, &flashConfig, TEST_OFFSET, programBuffer);
    if (status != kStatus_Success)
    {
        // PRINTF("Flash program failed\r\n");
        return;
    }
    else
    {
        // PRINTF("Flash program passed\r\n");
    }
    // Verify
    flexspi_nor_flash_read(FLASH_INSTANCE, &flashConfig, verifyBuffer, TEST_OFFSET, sizeof(programBuffer));
    if (memcmp(verifyBuffer, programBuffer, sizeof(programBuffer)) == 0)
    {
        // PRINTF("Flash program verify passed\r\n");
    }
    else
    {
        // PRINTF("Flash program verify failed\r\n");
    }

    //    flexspi_nor_flash_erase_all(FLASH_INSTANCE, &flashConfig);

    while (1)
    {
    }
}
