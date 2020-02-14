/*************************************************************************
 *
 *   Used with ICCARM and AARM.
 *
 *    (c) Copyright IAR Systems 2015
 *        Copyright NXP 2016
 *
 *    File name   : Flash.c
 *    Description : iMX7ULP_SDK  QSPI flash loader
 *
 *    History :
 *    1. Date        : December, 2015
 *       Author      : Stoyan Choynev
 *       Description : Initial revision
 *
 *    2. Date        : December, 2017
 *       Author      : Atanas Uzunov
 *       Description : Updated support for Macronix MX25R. Add feature to
 *                     set the L/H Switch bit for faster operation.
 *
 *    $Revision: 5068 $
 **************************************************************************/
/** include files **/
#include <intrinsics.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <ctype.h>
#include "flash_loader.h"       // The flash loader framework API declarations.
#include "flash_loader_extra.h"
#include "bl_api.h"
#include "cmsis_iccarm.h"


/** local definitions **/

#define FLASH_BASE_ADDR 0x08000000
#define FLASH_CONTEXT (*(uint32_t*)0x50002380)
#define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)
/** default settings **/

/** external functions **/

/** external data **/

/** internal functions **/

/** public data **/

/** private data **/
flexspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;
/** internal functions **/
#if USE_ARGC_ARGV
static uint32_t strToUint(const char *str);
#endif
/** public functions **/

/*************************************************************************
 * Function Name: FlashInit
 * Parameters: Flash Base Address
 *
 * Return:  0 - Init Successful
 *          1 - Init Fail
 * Description: Init flash and build layout.
 *
 *************************************************************************/
#if USE_ARGC_ARGV
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags,
                   int argc, char const *argv[])
#else
uint32_t FlashInit(void *base_of_flash, uint32_t image_size,
                   uint32_t link_address, uint32_t flags)
#endif /* USE_ARGC_ARGV */
{
    status_t status = RESULT_OK;
  __disable_irq();
   uint32_t v;
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
  configOption.option0.U = 0xc1503051;
  configOption.option1.U = 0x20000014;
  
  
  flexspi_nor_set_clock_source(kFlexSpiClockSrc_FFRO_Clk);
  //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_MASK;
  MEM_WriteU32(0x40001040, 0x10000U);
  //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI0_OTFAD_MASK;
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

  if(RESULT_OK == flexspi_nor_auto_config(0, &flashConfig, &configOption))
  {
    if ((FLAG_ERASE_ONLY & flags) != 0)
    {
      if(flexspi_nor_flash_erase_all(0, &flashConfig) !=  RESULT_ERROR)
      {
        status = (RESULT_OK);
      }
      else
      {
        status = (RESULT_ERROR);
      }
    }
    else
    {
      status = (RESULT_OK);
    }
  }
  return status;
}

/*************************************************************************
 * Function Name: FlashWrite
 * Parameters: block base address, offet in block, data size, ram buffer
 *             pointer
 * Return:  0 - Write Successful
 *          1 - Write Fail
 * Description. Writes data to Flash
 *************************************************************************/
uint32_t FlashWrite(void *block_start,
                    uint32_t offset_into_block,
                    uint32_t count,
                    char const *buffer)
{
  uint32_t addr = (uint32_t)block_start+offset_into_block - FLASH_BASE_ADDR;
  status_t status = 0;

  for(uint32_t size = 0; size < count; size+=flashConfig.pageSize,
                                       buffer+=flashConfig.pageSize,
                                       addr+=flashConfig.pageSize)
  {
    if(flexspi_nor_flash_page_program(0,&flashConfig, addr, (uint32_t *)buffer) != RESULT_OK)
    {
      status = RESULT_ERROR;
      break;
    }
  }
  return status;
}

/*************************************************************************
 * Function Name: FlashErase
 * Parameters:  Block Address, Block Size
 *
 * Return: 0
 *
 * Description: Erase block
 *************************************************************************/
uint32_t FlashErase(void *block_start,
                    uint32_t block_size)
{
  uint32_t addr = (uint32_t)block_start - FLASH_BASE_ADDR;

  /*Erase Sector*/
  status_t status = RESULT_OK;
  if(flexspi_nor_flash_erase(0,&flashConfig, addr, block_size) != RESULT_OK)
  {
    status = RESULT_ERROR;
  }
  return status;
}

OPTIONAL_SIGNOFF

/*************************************************************************
 * Function Name: FlashSignoff
 * Parameters:  none
 *
 * Return: 0
 *
 * Description: Restore the performance mode.
 *************************************************************************/
uint32_t FlashSignoff(void)
{
  return (RESULT_OK);
}
/** private functions **/
#if USE_ARGC_ARGV
/*************************************************************************
* String to unsigned integer
*************************************************************************/
static uint32_t strToUint(const char *str)
{
    uint32_t base = 10, result = 0;
    char str_c;
    /* Check number base */
    if(strlen(str) > 0 && str[0] == '0')
    {
        if(strlen(str) > 1  && toupper(str[1]) == 'X')
        {
            base = 16;
            str += 2;
        }
        else
        {
            base = 8;
            str += 1;
        }
    }
    /* Convert string to unsigned integer */
    while((str_c = toupper(*str++)) != '\0')
    {
        if('0' <= str_c && str_c <= '9')
            result = result * base + str_c - '0';
        else if('A' <= str_c && str_c <= 'F')
            result = result * base + str_c - 'A' + 10;
        else
            return UINT32_MAX;
    }

    return result;
}
#endif
