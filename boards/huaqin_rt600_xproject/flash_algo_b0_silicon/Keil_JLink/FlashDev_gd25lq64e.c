/* -----------------------------------------------------------------------------
 * Copyright (c) 2014 ARM Ltd.
 *
 * This software is provided 'as-is', without any express or implied warranty. 
 * In no event will the authors be held liable for any damages arising from 
 * the use of this software. Permission is granted to anyone to use this 
 * software for any purpose, including commercial applications, and to alter 
 * it and redistribute it freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not 
 *    claim that you wrote the original software. If you use this software in
 *    a product, an acknowledgment in the product documentation would be 
 *    appreciated but is not required. 
 * 
 * 2. Altered source versions must be plainly marked as such, and must not be 
 *    misrepresented as being the original software. 
 * 
 * 3. This notice may not be removed or altered from any source distribution.
 *   
 *
 * $Date:        04. September 2014
 * $Revision:    V1.00
 *  
 * Project:      Flash Device Description for
 *               NXP MIMXRT6XX W25Q16FW QSPI Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "FlashOS.H"        // FlashOS Structures

struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT6XX GD25LQ64E FLEXSPI",  // Device Name 
   EXTSPI,                     // Device Type
   0x08000000,                 // Device Start Address
   0x00200000,                 // Device Size is 2 MB (16 Mb)
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   300,                        // Program Page Timeout 300 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x01000, 0x0,               // sectors are 4 KB
   SECTOR_END
};

