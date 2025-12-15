/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _property_h
#define _property_h

#include <stdint.h>

#include "bootloader_common.h"
#include "fsl_device_registers.h"

//! @addtogroup property
//! @{

////////////////////////////////////////////////////////////////////////////////
// Declarations
////////////////////////////////////////////////////////////////////////////////

//! @brief External Memory Properties tag
enum _external_memory_property_tags
{
    kExternalMemoryPropertyTag_InitStatus = 0,         //!< Init status tag
    kExternalMemoryPropertyTag_StartAddress = 1,       //!< Start address tag
    kExternalMemoryPropertyTag_MemorySizeInKbytes = 2, //!< Memory size tag
    kExternalMemoryPropertyTag_PageSize = 3,           //!< Pag size tag
    kExternalMemoryPropertyTag_SectorSize = 4,         //!< Sector size tag
    kExternalMemoryPropertyTag_BlockSize = 5,          //!< Block size tag

    kExternalMemoryPropertyTag_Start = kExternalMemoryPropertyTag_StartAddress,
    kExternalMemoryPropertyTag_End = kExternalMemoryPropertyTag_BlockSize,
};

////////////////////////////////////////////////////////////////////////////////
// Externs
////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

#if __cplusplus
extern "C"
{
#endif

#if __cplusplus
}
#endif

//! @}

#endif // _property_h
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
