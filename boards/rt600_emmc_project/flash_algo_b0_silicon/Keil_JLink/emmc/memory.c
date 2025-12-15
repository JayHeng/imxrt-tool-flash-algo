/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "memory.h"

//! @addtogroup memif
//! @{

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// Variables
////////////////////////////////////////////////////////////////////////////////

external_memory_map_entry_t g_externalMemoryMap[] = {
    // MMC card memory
    { 0 },
    // Terminator
    { 0 } 
};

////////////////////////////////////////////////////////////////////////////////
// Code
////////////////////////////////////////////////////////////////////////////////

status_t find_external_map_index(uint32_t memoryId, uint32_t *index)
{
    status_t status = kStatus_InvalidArgument;

    const external_memory_map_entry_t *map;
    uint32_t searchingIndex = 0;

    if (index == NULL)
    {
        return status;
    }

    map = &g_externalMemoryMap[0];
    // Scan memory map array looking for a match.
    while(map && (map->memoryId != 0) && (map->memoryInterface != NULL))
    {
        if (memoryId == map->memoryId)
        {
            *index = searchingIndex;
            // Find the correct index.
            status = kStatus_Success;
            break;
        }
        searchingIndex++;
        map++;
    };

    return status;
}


//! @}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
