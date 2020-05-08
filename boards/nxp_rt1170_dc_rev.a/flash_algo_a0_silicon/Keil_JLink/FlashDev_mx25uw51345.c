/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Device Description for New Device Flash               */
/*                                                                     */
/***********************************************************************/

#include "FlashOS.H" // FlashOS Structures

struct FlashDevice const FlashDevice = {FLASH_DRV_VERS,                      // Driver Version, do not modify!
                                        "MIMXRT117x 16mB QuadSPI NOR Flash", // Device Name
                                        EXTSPI,                              // Device Type
                                        0x30000000,                          // Device Start Address
                                        0x01000000,                          // Device Size in Bytes (16mB)
                                        256,                                 // Programming Page Size
                                        0,                                   // Reserved, must be 0
                                        0xFF,                                // Initial Content of Erased Memory
                                        100,                                 // Program Page Timeout 100 mSec
                                        5000,                                // Erase Sector Timeout 5000 mSec

                                        // Specify Size and Address of Sectors
                                        0x10000, 0x00000000, // Sector Size  64kB (256 Sectors)
                                        SECTOR_END};
