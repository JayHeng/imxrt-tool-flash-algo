/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description for New Device Flash               */
/*                                                                     */
/***********************************************************************/

#include "FlashOS.H" // FlashOS Structures

struct FlashDevice const FlashDevice = {FLASH_DRV_VERS,                     // Driver Version, do not modify!
                                        "MIMXRT106x IS25LP256 FLEXSPI",     // Device Name
                                        EXTSPI,                             // Device Type
                                        0x60000000,                         // Device Start Address
                                        0x02000000,                         // Device Size in Bytes (32mB)
                                        256,                                // Programming Page Size
                                        0,                                  // Reserved, must be 0
                                        0xFF,                               // Initial Content of Erased Memory
                                        100,                                // Program Page Timeout 100 mSec
                                        5000,                               // Erase Sector Timeout 5000 mSec

                                        // Specify Size and Address of Sectors
                                        0x8000, 0x00000000, // Sector Size  32kB (256 Sectors)
                                        SECTOR_END};
