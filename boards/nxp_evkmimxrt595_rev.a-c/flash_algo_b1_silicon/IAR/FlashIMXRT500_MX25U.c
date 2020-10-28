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

#define __enable_irq    __enable_interrupt
#define __disable_irq   __disable_interrupt
#define __NOP           __no_operation

/** local definitions **/

#define FLASH_BASE_ADDR 0x08000000
#define CONFIG_OPTION 0xc0403001
#define FLASH_CONTEXT (*(uint32_t*)0x50002380)

/** default settings **/

/** external functions **/

/** external data **/

/** internal functions **/

/** public data **/

/** private data **/
flexspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;
/** internal functions **/
static uint32_t strToUint(const char *str);
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

  #define MEM_WriteU32(addr, value)  (*((volatile uint32_t *)(addr)) = value)
  uint32_t v;
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

  configOption.option0.U = 0xC0000000;
  configOption.option1.U = 0x00000000;

#if USE_ARGC_ARGV
    for(int i = 0; i < argc; /*i++*/)
    {
        if((strcmp("--Opt0", argv[i]) == 0) && ((i+1) < argc))
        {
            /* Flash clock init */
            configOption.option0.U = strToUint(argv[++i]);
            i+=2;
        }
        else if((strcmp("--Opt1", argv[i]) == 0) && ((i+1) < argc))
        {
            /* Flash clock init */
            configOption.option1.U = strToUint(argv[++i]);
            i+=2;
        }
        else
        {
            if(strcmp("--Qspi", argv[i]) == 0 )
            {//default option setting
            }
            else if(strcmp("--QspiDDR", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0100000;
            }
            else if(strcmp("--Hyper1V8", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0233001;
            }
            else if(strcmp("--Hyper3V0", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0333000;
            }
            else if(strcmp("--MxicOpiDDR", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0433000;
            }
            else if(strcmp("--MxicOct", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0403004;
                flash_run_context_t flashCtx = {.U = 0};
                flashCtx.B.current_mode = kFlashInstMode_OPI_DDR;
                flashCtx.B.restore_sequence = kRestoreSequence_Send_6699_9966;
                flashCtx.B.por_mode = kFlashInstMode_ExtendedSpi;
                FLASH_CONTEXT = flashCtx.U;
            }
            else if(strcmp("--McrnOct", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0600000;
            }
            else if(strcmp("--McrnOpi", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0603000;
                flash_run_context_t flashCtx = {.U = 0};
                flashCtx.B.current_mode = kFlashInstMode_OPI_DDR;
                flashCtx.B.restore_sequence = kRestoreSequence_Send_66_99;
                flashCtx.B.por_mode = kFlashInstMode_ExtendedSpi;
                FLASH_CONTEXT = flashCtx.U;
            }
            else if(strcmp("--McrnOpiDDR", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0633000;
            }
            else if(strcmp("--AdstOpi", argv[i]) == 0 )
            {
                configOption.option0.U = 0xc0803000;
                flash_run_context_t flashCtx = {.U = 0};
                flashCtx.B.current_mode = kFlashInstMode_OPI_DDR;
                flashCtx.B.restore_sequence = kRestoreSequence_Send_06_FF;
                flashCtx.B.por_mode = kFlashInstMode_ExtendedSpi;
                FLASH_CONTEXT = flashCtx.U;
            }

            i+=1;
        }
    }
#endif
  flexspi_nor_set_clock_source(kFlexSpiClockSrc_FRO192M_Clk);
  //CLKCTL0->PSCCTL0_SET = CLKCTL0_PSCCTL0_SET_FLEXSPI0_OTFAD_MASK;
  *((volatile uint32_t *)0x40001040) = 0x10000U;
  //RSTCTL0->PRSTCTL0_CLR = RSTCTL0_PRSTCTL0_CLR_FLEXSPI0_OTFAD_MASK;
  *((volatile uint32_t *)0x40000070) = 0x10000U;

  /* NOTE: ROM API doesn't clear all LUT. IAR DAP access after software reset
   * may cause error in case the LUT contains wrong command.
   * Workaround: Clear LUT and set all commands to STOP.
   */
  MEM_WriteU32(0x40134018, 0x5AF05AF0);
  MEM_WriteU32(0x4013401C, 0x2);
  for (int i = 0; i < 64; i++)
  {
      // Srt AHB Write triggered Command to stop
      MEM_WriteU32(0x40134200 + i * 4, 0x00000000);
  }
  MEM_WriteU32(0x40134018, 0x5AF05AF0);
  MEM_WriteU32(0x4013401C, 0x1);

  /* Now start ROM API init for flash */
  if(RESULT_OK == flexspi_nor_auto_config(1, &flashConfig, &configOption))
  {
    if ((FLAG_ERASE_ONLY & flags) != 0)
    {
      if(flexspi_nor_flash_erase_all(1, &flashConfig) !=  RESULT_ERROR)
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
    if(flexspi_nor_flash_page_program(1, &flashConfig, addr, (uint32_t *)buffer) != RESULT_OK)
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
  if(flexspi_nor_flash_erase(1, &flashConfig, addr, block_size) != RESULT_OK)
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
