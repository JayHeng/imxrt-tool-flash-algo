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
#include "bl_api.h"

/** local definitions **/

#define FLASH_BASE_ADDR 0x08000000

quadspi_nor_config_t flashConfig = {.pageSize = 0x400};

quadspi_mem_config_t w25q_config = {
  .tag = QUADSPI_CFG_BLK_TAG,
  .version = QUADSPI_CFG_BLK_VERSION,
  .readSamplingOption = 0x3,
  .csHoldTime = 0x3,
  .csSetupTime = 0x3,
  .columnAddressWidth = 0x0,
  .deviceModeCfgEnable = 0x0,
  .deviceModeType = 0x0,
  .waitTimeCfgCommands = 0x0,
  .deviceModeSeq = 0x0,
  .deviceModeArg = 0x0,
  .configCmdEnable = 0x0,
  .configModeType = 0x0,
  .configCmdSeqs = 0x0,
  .configCmdArgs = 0x0,
  .controllerMiscOption = 0x0,
  .deviceType = 0,
  .sflashPadType = kSerialFlash_1Pad,
  .serialClkFreq = kQuadSpiSerialClk_50MHz,
  .sflashSize[0] = 0x200000,
  .sflashSize[1] = 0,
  .sflashSize[2] = 0,
  .sflashSize[3] = 0,	
  .csPadSettingOverride = 0,
  .sclkPadSettingOverride = 0,
  .dataPadSettingOverride = 0,
  .dqsPadSettingOverride = 0,
  .timeoutInMs = 0,
  .coarseTuning = 0,
  .fineTuning = 0x0,
  .samplePoint = 0x0,
  .dataHoldTime = 0x0,
  .busyOffset = 0,
  .busyBitPolarity = 0,
  .lookupTable = {
  /* Seq 0 :Single Read */
  /* CMD:        0x03 - Single Read, Single pad */
  /* ADDR:       0x18 - 24bit address, Single pads */
  /* READ:       0x80 - Read 128 bytes, Single pads */		
  [0] = 0x08180403, [1] = 0x24001C80, 

  /* Seq 1: Read Status Register 1*/
  /* CMD:    0x05 - Read Status, single pad */
  /* READ:   0x01 - Read 1 byte */
  [4] = 0x1C040405, 
    
  /* Seq 3: Write Enable */
  /* CMD:      0x06 - Write Enable, Single pad */		
  [12] = 0x0406, 
     
  /* Seq 5: Erase Sector */
  /* CMD:  0x20 - Sector Erase, single pad */
  /* ADDR: 0x18 - 24 bit address, single pad */		
  [20] = 0x08180420,
     
  /* Seq 8: Erase Block */
  /* CMD:  0x52 - Block Erase, single pad */
  /* ADDR: 0x18 - 24 bit address, single pad */			
  [32] = 0x08180452,

  /* Seq 8: Page Program */
  /* CMD:    0x02 - Page Program, Single pad */
  /* ADDR:   0x18 - 24bit address, Single pad */
  /* WRITE:  0x80 - Write 128 bytes at one pass, Single pad */
  [36] = 0x08180402, [37] = 0x2080,

  /* Seq 11: Chip Erase*/
  /* CMD:    0x60 - Erase All chip, Single pad */		
  [44] = 0x460,      
  },

};

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  memset((void *)&flashConfig, 0U, sizeof(quadspi_nor_config_t));

  flashConfig.memConfig = w25q_config; 
  flashConfig.pageSize = 0x100;              //!< Page size of Serial NOR
  flashConfig.sectorSize = 0x1000;            //!< Sector size of Serial NOR
  flashConfig.ipcmdSerialClkFreq = 0;     //!< Clock frequency for IP command
  flashConfig.isUniformBlockSize = 0;     //!< Sector/Block size is the same
  flashConfig.isDataOrderSwapped = 0;     //!< Data order (D0, D1, D2, D3) is swapped (D1,D0, D3, D2)
  flashConfig.resetLogic = 0;             //!< Reset Logic: 0 - No reset, 1 - Reset Pin, 2 - JEDEC HW reset
  flashConfig.serialNorType = 0;          //!< Serial NOR Flash type: 0/1/2/3
  flashConfig.needExitNoCmdMode = 0;      //!< Need to exit NoCmd mode before other IP command
  flashConfig.halfClkForNonReadCmd = 0;   //!< Half the Serial Clock for non-read command: true/false
  flashConfig.needRestoreNoCmdMode = 0;   //!< Need to Restore NoCmd mode after IP commmand execution
  flashConfig.blockSize = 0x8000;             //!< Block size
  flashConfig.isNonBlockingMode = 0;      //!< Non-blocking mode flag

  status_t status = quadspi_nor_init(&flashConfig);

  return status;                                  /* Finished without Errors */
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
  status_t status =  quadspi_nor_erase_all(&flashConfig);
  
  return status;
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

void delay(void)
{
    volatile uint32_t i = 0;
    for (i = 0; i < 200000000; ++i)
    {
        __asm("NOP"); /* delay */
    }
}

int EraseSector (unsigned long adr) {

  /*Erase Sector*/
  status_t status =  quadspi_nor_erase(&flashConfig, adr, flashConfig.sectorSize);
  //delay();
  
  return status;
}


/*  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf) {

  status_t status = 0;

  for(uint32_t size = 0; size < sz; size+=flashConfig.pageSize,
                                       buf+=flashConfig.pageSize,
                                       adr+=flashConfig.pageSize)
  {
    status =  quadspi_nor_page_program(&flashConfig, adr, (uint32_t *)buf);
  }

  return status;
}
