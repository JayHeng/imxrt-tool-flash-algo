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
                                        "MIMXRT117x_QuadSPI_4KB_SEC",        // Device Name
                                        EXTSPI,                              // Device Type
                                        FLASH_BASE_ADDRESS,                          // Device Start Address of Alias
                                        FLASH_BASE_SIZE,                           // Device Size in Bytes (16mB)
                                        FLASH_PAGE_SIZE,                                 // Programming Page Size
                                        0,                                   // Reserved, must be 0
                                        0xFF,                                // Initial Content of Erased Memory
                                        100,                                 // Program Page Timeout 100 mSec
                                        15000,                                // Erase Sector Timeout 5000 mSec

                                        // Specify Size and Address of Sectors
                                        FLASH_SECTOR_SIZE, 0x00000000, // Sector Size  64kB (256 Sectors)
                                        SECTOR_END};
