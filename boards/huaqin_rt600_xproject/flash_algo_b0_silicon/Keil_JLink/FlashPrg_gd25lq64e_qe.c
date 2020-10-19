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
#include "flash_config.h"
#include "cmsis_compiler.h"

/** local definitions **/

#define FLASH_BASE_ADDR 0x08000000
#define CONFIG_OPTION 0xc0000208

#define FLEXSPI_INSTANCE_0 (0)
#define FLEXSPI_INSTANCE_SEL FLEXSPI_INSTANCE_0

#define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)

flexspi_nor_config_t flashConfig = {.pageSize = 0x400};

void configBootClk(void)
{
  uint32_t v;
  __disable_irq();
  // Disable cache
  MEM_WriteU32(0x40033800, 0);
  MEM_WriteU32(0x4003301C, 0);
  // PMC->MEMSEQCTRL = 0x1U;
  MEM_WriteU32(0x40135030, 0x1U);
  // Power FFRO
  MEM_WriteU32(0x40002630, 0x10000U);
  /* Flexspi SRAM APD/PPD */
  MEM_WriteU32(0x40002634, 0xCU);
  // Power SRAM
  MEM_WriteU32(0x40002638, 0xFFFFFFFF);
  MEM_WriteU32(0x4000263C, 0xFFFFFFFF);

  // PMC CTRL APPLYCFG
  *((volatile uint32_t *)0x4013500C) = *((volatile uint32_t *)0x4013500C) | 1;
  // WAIT PMC update done
  do {
      v = *((volatile uint32_t *)0x40135004) & 1;
  } while (v);

  /* MAINCLKSELA */
  MEM_WriteU32(0x40001430, 0x3U);
  /* MAINCLKSELB */
  MEM_WriteU32(0x40001434, 0x0U);

  flexspi_nor_set_clock_source(kFlexSpiClockSrc_FFRO_Clk);
  //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI_OTFAD_CLK_MASK;
  MEM_WriteU32(0x40001040, 0x10000U);
  //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI_OTFAD_MASK;
  MEM_WriteU32(0x40000070, 0x10000U);

  //IOPCTL->PIO[2][12] = 0x130;
  MEM_WriteU32(0x40004130, 0x130U);
  //CLKCTL1->PSCCTL1_SET = CLKCTL1_PSCCTL1_SET_HSGPIO2_CLK_SET_MASK;
  MEM_WriteU32(0x40021044, 0x4U);
  //RSTCTL1->PRSTCTL1_CLR = RSTCTL1_PRSTCTL1_CLR_HSGPIO2_RST_CLR_MASK;
  MEM_WriteU32(0x40020074, 0x4U);
  // GPIO->DIR[2] = 1 << 12;
  MEM_WriteU32(0x40102008, 0x1000U);
  // GPIO->CLR[2] = 1 << 12;
  MEM_WriteU32(0x40102288, 0x1000U);
  // Delay 400us to reset external flash
  for(uint32_t i =0; i < 6000; i++)
    __NOP();
  // GPIO->SET[2] = 1 << 12;
  MEM_WriteU32(0x40102208, 0x1000U);
  // Clear FLASH status store register
  MEM_WriteU32(0x40002380, 0x0U);
}

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
	
  configBootClk();
	
  serial_nor_config_option_t configOption;
  configOption.option0.U = CONFIG_OPTION;
	
	memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));

  flexspi_nor_get_config(FLEXSPI_INSTANCE_SEL, &flashConfig, &configOption);
	
  return (int)flexspi_nor_flash_init(FLEXSPI_INSTANCE_SEL, &flashConfig);
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
  status_t status =  flexspi_nor_flash_erase_all(FLEXSPI_INSTANCE_SEL, &flashConfig);
  
  return (int)status;
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr) {

  /*Erase Sector*/
  status_t status =  flexspi_nor_flash_erase(FLEXSPI_INSTANCE_SEL, &flashConfig, adr - FLASH_BASE_ADDR, flashConfig.sectorSize);
  
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
    status =  flexspi_nor_flash_page_program(FLEXSPI_INSTANCE_SEL, &flashConfig, adr - FLASH_BASE_ADDR, (uint32_t *)buf);
  }

  return (int)status;
}
