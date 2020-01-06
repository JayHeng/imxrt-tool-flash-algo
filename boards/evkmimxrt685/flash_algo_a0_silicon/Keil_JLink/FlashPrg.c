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
#define CONFIG_OPTION 0xc0000000

quadspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {

  memset((void *)&flashConfig, 0U, sizeof(quadspi_nor_config_t));
  configOption.option0.U = CONFIG_OPTION;
  status_t status = quadspi_nor_get_config(&flashConfig, &configOption);

  if(!status)
  {
    status = quadspi_nor_init(&flashConfig);
  }

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

int EraseSector (unsigned long adr) {

  /*Erase Sector*/
  status_t status =  quadspi_nor_erase(&flashConfig, adr, flashConfig.sectorSize);
  
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
