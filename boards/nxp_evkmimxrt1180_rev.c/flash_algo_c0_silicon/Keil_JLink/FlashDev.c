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

#ifdef SECURE_ADDR
#define SEC_ADDR_MASK (0x10000000U)
#define SECURE_NAME   " SEC"
#else
#define SEC_ADDR_MASK (0x0U)
#define SECURE_NAME   ""
#endif

#ifdef ALIAS_ADDR
#define ALIAS_NAME    " ALIAS"
#define	FSPI1_ADDR    (0x02000000 | SEC_ADDR_MASK)
#define FSPI2_ADDR    (0x22000000 | SEC_ADDR_MASK)
#else
#define ALIAS_NAME    ""
#define	FSPI1_ADDR    (0x28000000 | SEC_ADDR_MASK)
#define FSPI2_ADDR    (0x04000000 | SEC_ADDR_MASK)
#endif

#ifdef CPU_MIMXRT1189CVM8B_cm33
#define CORE_NAME     "CM33"
#else
#define CORE_NAME     "CM7"
#endif

#if defined(RT1180_EVB_A) || defined(RT1180_FRDM)
struct FlashDevice const FlashDevice = {
   FLASH_DRV_VERS,             // Driver Version, do not modify!
   "MIMXRT1180 " CORE_NAME " FLEXSPI2" ALIAS_NAME SECURE_NAME,  // CM33 Device Name
   EXTSPI,                     // Device Type
   FSPI2_ADDR,                 // Device Start Address
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
   "MIMXRT1180 " CORE_NAME " FLEXSPI1" ALIAS_NAME SECURE_NAME,  // Device Name
   EXTSPI,                     // Device Type
   FSPI1_ADDR,                 // Device Start Address
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

