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
 *               NXP MIMXRT6XX MX25 QSPI Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include "../FlashOS.H"        // FlashOS Structures

#if defined(RT1180_EVB_A)
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT1180 FLEXSPI",       // Device Name 
   EXTSPI,                     // Device Type
   0x04000000,                 // Device Start Address
   0x01000000,                 // Device Size is 16MB
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   300,                        // Program Page Timeout 300 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x1000, 0x0,                // sectors are 4 KB
   SECTOR_END
};
#elif defined(RT1180_EVK_FSPI1_QSPI)
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
#if defined(CPU_MIMXRT1189CVM8B_cm33)
   "MIMXRT1180 CM33 FLEXSPI",  // CM33 Device Name 
#elif defined(CPU_MIMXRT1189CVM8B_cm7)
	 "MIMXRT1180 CM7 FLEXSPI",  // CM7 Device Name
#else
	 "MIMXRT1180 FLEXSPI",  // CM7 Device Name
#endif
   EXTSPI,                     // Device Type
   0x28000000,                 // Device Start Address
   0x01000000,                 // Device Size is 16MB
   256,                        // Programming Page Size
   0,                          // Reserved, must be 0
   0xFF,                       // Initial Content of Erased Memory
   300,                        // Program Page Timeout 300 mSec
   3000,                       // Erase Sector Timeout 3000 mSec

// Specify Size and Address of Sectors
   0x1000, 0x0,                // sectors are 4 KB
   SECTOR_END
};
#endif

