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
#define CONFIG_OPTION 0xc0403004

#define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)

flexspi_nor_config_t flashConfig = {.pageSize = 256};

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
  uint32_t v;

  memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
  serial_nor_config_option_t configOption;
  configOption.option0.U = CONFIG_OPTION;
  configOption.option1.U = 0;

  //CACHE64_CTRL0->CCR = 0;
  MEM_WriteU32(0x40033800, 0);
  //CACHE64_POLSEL0->POLSEL = 0;
  MEM_WriteU32(0x4003301C, 0);
  // PMC->MEMSEQCTRL = 0x101U;
  MEM_WriteU32(0x40135030, 0x101U);
  //SYSCTL0->PDRUNCFG0_CLR = SYSCTL0_PDRUNCFG0_FFRO_PD_MASK;
  *((volatile uint32_t *)0x40002630u) = 0x10000U;
  /* Flexspi SRAM APD/PPD */
  *((volatile uint32_t *)0x40002634u) = 0xCU;
  // PDRUNCFG2, 3 CLR
  MEM_WriteU32(0x40002638, 0xFFFFFFFF);
  MEM_WriteU32(0x4000263C, 0xFFFFFFFF);

  // PMC CTRL APPLYCFG
  *((volatile uint32_t *)0x4013500C) = *((volatile uint32_t *)0x4013500C) | 1;
  // WAIT PMC update done
  do {
      v = *((volatile uint32_t *)0x40135004) & 1;
  } while (v);
  
//  CLKCTL0->FFRODIVOEN = CLKCTL0_FFRODIVOEN_FFRO_DIV1_O_EN_MASK | CLKCTL0_FFRODIVOEN_FFRO_DIV2_O_EN_MASK |
//                        CLKCTL0_FFRODIVOEN_FFRO_DIV4_O_EN_MASK | CLKCTL0_FFRODIVOEN_FFRO_DIV8_O_EN_MASK |
//                        CLKCTL0_FFRODIVOEN_FFRO_DIV16_O_EN_MASK;
  *((volatile uint32_t *)0x40001110u) = 0x1FU;
  /* MAINCLKSELA */
  *((volatile uint32_t *)0x40001430u) = 0x3U;
  /* MAINCLKSELB */
  *((volatile uint32_t *)0x40001434u) = 0x0U;

  flexspi_nor_set_clock_source(kFlexSpiClockSrc_FRO192M_Clk);
  //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_MASK;
  *((volatile uint32_t *)0x40001040) = 0x10000U;
  //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI0_OTFAD_MASK;
  *((volatile uint32_t *)0x40000070) = 0x10000U;

  return (int)flexspi_nor_auto_config(1, &flashConfig, &configOption);
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
  status_t status =  flexspi_nor_flash_erase_all(1, &flashConfig);
  
  return (int)status;
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

  /*Erase Sector*/
  status_t status =  flexspi_nor_flash_erase(1, &flashConfig, adr - FLASH_BASE_ADDR, flashConfig.sectorSize);
  
  return (int)status;
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
    status =  flexspi_nor_flash_page_program(1, &flashConfig, adr - FLASH_BASE_ADDR, (uint32_t *)buf);
  }

  return (int)status;
}
