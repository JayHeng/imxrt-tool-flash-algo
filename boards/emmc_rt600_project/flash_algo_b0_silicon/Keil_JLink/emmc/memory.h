/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _memory_h
#define _memory_h

#include <stdint.h>
#include "bootloader_common.h"

//! @addtogroup memif
//! @{

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

/*! @brief Memory device ID definition. */
enum _bl_memory_id
{
    kMemoryMMCCard = 1,      // MMC, eMMC memory Card
};

//! @brief Memory interface status codes.
enum _memory_interface_status
{
    kStatusMemoryRangeInvalid = MAKE_STATUS(kStatusGroup_MemoryInterface, 0),
    kStatusMemoryReadFailed = MAKE_STATUS(kStatusGroup_MemoryInterface, 1),
    kStatusMemoryWriteFailed = MAKE_STATUS(kStatusGroup_MemoryInterface, 2),
    kStatusMemoryCumulativeWrite = MAKE_STATUS(kStatusGroup_MemoryInterface, 3),
    kStatusMemoryAppOverlapWithExecuteOnlyRegion = MAKE_STATUS(kStatusGroup_MemoryInterface, 4),
    kStatusMemoryNotConfigured = MAKE_STATUS(kStatusGroup_MemoryInterface, 5),
    kStatusMemoryAlignmentError = MAKE_STATUS(kStatusGroup_MemoryInterface, 6),
    kStatusMemoryVerifyFailed = MAKE_STATUS(kStatusGroup_MemoryInterface, 7),
    kStatusMemoryWriteProtected = MAKE_STATUS(kStatusGroup_MemoryInterface, 8),
    kStatusMemoryAddressError = MAKE_STATUS(kStatusGroup_MemoryInterface, 9),
    kStatusMemoryBlankCheckFailed = MAKE_STATUS(kStatusGroup_MemoryInterface, 10),
    kStatusMemoryBlankPageReadDisallowed = MAKE_STATUS(kStatusGroup_MemoryInterface, 11),
    kStatusMemoryProtectedPageReadDisallowed = MAKE_STATUS(kStatusGroup_MemoryInterface, 12),
    kStatusMemoryUnsupportedCommand = MAKE_STATUS(kStatusGroup_MemoryInterface, 13),
};

typedef struct _external_memory_region_interface
{
    status_t (*init)(void);
    status_t (*read)(uint32_t address, uint32_t length, uint8_t *buffer);
    status_t (*write)(uint32_t address, uint32_t length, const uint8_t *buffer);
    status_t (*erase)(uint32_t address, uint32_t length);
    status_t (*config)(uint32_t *buffer);
    status_t (*flush)(void);
    status_t (*finalize)(void);
    status_t (*erase_all)(void);
} external_memory_region_interface_t;

typedef struct _external_memory_map_entry
{
    uint32_t memoryId;
    uint32_t status;
    uint32_t basicUnitCount;
    uint32_t basicUnitSize;
    const external_memory_region_interface_t *memoryInterface;
} external_memory_map_entry_t;

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////

extern external_memory_map_entry_t g_externalMemoryMap[];

extern const external_memory_region_interface_t g_mmcMemoryInterface;

//@}

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if defined(__cplusplus)
extern "C"
{
#endif // __cplusplus

    //! @brief Find an external map index that matches the given memory id.
    status_t find_external_map_index(uint32_t memoryId, uint32_t *index);

    //@}

    //! @name MMC CARD
    //@{
    //! @brief Erase all MMC memory
    status_t mmc_mem_erase_all(void);
    //! @brief Get Property from MMC card driver.
    status_t mmc_get_property(uint32_t whichProperty, uint32_t *value);


#if defined(__cplusplus)
}
#endif // __cplusplus

//! @}

#endif // _memory_h
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
