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
 * Project:      Flash Programming Functions for
 *               NXP LPC18xx/LPC43xx S25FL032 SPIFI Flash
 * --------------------------------------------------------------------------- */

/* History:
 *  Version 1.00
 *    Initial release
 */ 

#include <stdbool.h>
#include "FlashOS.H"        // FlashOS Structures
#include <string.h>
#include "cmsis_compiler.h"
#include "bootloader_common.h"
#include "property.h"
#include "sd_memory.h"


#define NATIVE_READ (false)
#if NATIVE_READ
  #define OPEN_READ_FUNC  NULL
#else
  #define OPEN_READ_FUNC  SEGGER_OPEN_Read
#endif


/** local definitions **/

#define BOARD_RT600_EVK_USDHC_CARD    (1)

#if BOARD_RT600_EVK_USDHC_CARD
#define SD_CFG_OPTION0   (0xD0080100)
#endif

uint32_t SystemCoreClock;

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
    SystemCoreClock = 12000000;
    g_externalMemoryMap[0].memoryId = kMemorySDCard;
    g_externalMemoryMap[0].status = kStatus_Success;
    g_externalMemoryMap[0].basicUnitCount = 0;
    g_externalMemoryMap[0].basicUnitSize = 512;
    g_externalMemoryMap[0].memoryInterface = &g_sdMemoryInterface;

    // Init pinmux and other hardware setup.
    init_hardware();

    // Configure clocks.
    configure_clocks(kClockOption_EnterBootloader);
    
    g_sdMemoryInterface.init();

    status_t status = kStatus_InvalidArgument;
    
    sd_config_t sdConfig = 
    {
       .word0.U = SD_CFG_OPTION0,
    };

    status = g_sdMemoryInterface.config((uint32_t *)&sdConfig);

    return (int)status;
}


/*  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit (unsigned long fnc) {


  
  return 0;                                  // Finished without Errors
}


/*  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed */

int EraseChip (void) {

    /*Erase all*/
    status_t status = g_sdMemoryInterface.erase_all();

    return (int)status;
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

    /*Erase Sector*/
    status_t status = g_sdMemoryInterface.erase(adr, FSL_SDMMC_DEFAULT_BLOCK_SIZE);

    return (int)status;
}


/*  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {
    status_t status = g_sdMemoryInterface.write(adr, sz, (const uint8_t *)buf);

    return (int)status;
}

int SEGGER_OPEN_Read (unsigned long Addr, unsigned long NumBytes, unsigned char *pDestBuf) {
    int ReadBytes = -1;
    status_t status = g_sdMemoryInterface.read(Addr, NumBytes, pDestBuf);
    if (status == kStatus_Success)
    {
        ReadBytes = NumBytes;
    }
    return ReadBytes;
}

