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

/** local definitions **/

#define FLEXSPI_NOR_INSTANCE 0
#define FLASH_BASE_ADDR 0x60000000
#define FLASH_CONTEXT (*(uint32_t*)0x50002380)
   
/** default settings **/

/** external functions **/

/** external data **/

/** internal functions **/

/** public data **/

/** private data **/
flexspi_nor_config_t config;
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
    serial_nor_config_option_t configOption;
    
    configOption.option0.U = 0xc0000006;
    configOption.option1.U = 0x00000000;
    
    /* Disable Watchdog Power Down Counter */
    WDOG1->WMCR &= ~WDOG_WMCR_PDE_MASK;
    WDOG2->WMCR &= ~WDOG_WMCR_PDE_MASK;
    /* Watchdog disable */

    if (WDOG1->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG1->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    if (WDOG2->WCR & WDOG_WCR_WDE_MASK)
    {
        WDOG2->WCR &= ~WDOG_WCR_WDE_MASK;
    }
    RTWDOG->CNT   = 0xD928C520U; /* 0xD928C520U is the update key */
    RTWDOG->TOVAL = 0xFFFF;
    RTWDOG->CS    = (uint32_t)((RTWDOG->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;
    
    if (CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_BYPASS_MASK)
    {
        // Configure ARM_PLL
        CCM_ANALOG->PLL_ARM =
            CCM_ANALOG_PLL_ARM_BYPASS(1) | CCM_ANALOG_PLL_ARM_ENABLE(1) | CCM_ANALOG_PLL_ARM_DIV_SELECT(24);
        // Wait Until clock is locked
        while ((CCM_ANALOG->PLL_ARM & CCM_ANALOG_PLL_ARM_LOCK_MASK) == 0)
        {
        }

        // Configure PLL_SYS
        CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_POWERDOWN_MASK;
        // Wait Until clock is locked
        while ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_LOCK_MASK) == 0)
        {
        }

        // Configure PFD_528
        CCM_ANALOG->PFD_528 = CCM_ANALOG_PFD_528_PFD0_FRAC(24) | CCM_ANALOG_PFD_528_PFD1_FRAC(24) |
                              CCM_ANALOG_PFD_528_PFD2_FRAC(19) | CCM_ANALOG_PFD_528_PFD3_FRAC(24);

        // Configure USB1_PLL
        CCM_ANALOG->PLL_USB1 =
            CCM_ANALOG_PLL_USB1_DIV_SELECT(0) | CCM_ANALOG_PLL_USB1_POWER(1) | CCM_ANALOG_PLL_USB1_ENABLE(1);
        while ((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_LOCK_MASK) == 0)
        {
        }
        CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;

        // Configure PFD_480
        CCM_ANALOG->PFD_480 = CCM_ANALOG_PFD_480_PFD0_FRAC(35) | CCM_ANALOG_PFD_480_PFD1_FRAC(35) |
                              CCM_ANALOG_PFD_480_PFD2_FRAC(26) | CCM_ANALOG_PFD_480_PFD3_FRAC(15);

        // Configure Clock PODF
        CCM->CACRR = CCM_CACRR_ARM_PODF(1);

        CCM->CBCDR = (CCM->CBCDR & (~(CCM_CBCDR_SEMC_PODF_MASK | CCM_CBCDR_AHB_PODF_MASK | CCM_CBCDR_IPG_PODF_MASK))) |
                     CCM_CBCDR_SEMC_PODF(2) | CCM_CBCDR_AHB_PODF(2) | CCM_CBCDR_IPG_PODF(2);

        // Configure FLEXSPI2 CLOCKS
        CCM->CBCMR =
            (CCM->CBCMR &
             (~(CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_CLK_SEL_MASK | CCM_CBCMR_FLEXSPI2_PODF_MASK))) |
            CCM_CBCMR_PRE_PERIPH_CLK_SEL(3) | CCM_CBCMR_FLEXSPI2_CLK_SEL(1) | CCM_CBCMR_FLEXSPI2_PODF(7);

        // Confgiure FLEXSPI CLOCKS
        CCM->CSCMR1 = ((CCM->CSCMR1 & ~(CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK | CCM_CSCMR1_FLEXSPI_PODF_MASK |
                                        CCM_CSCMR1_PERCLK_PODF_MASK | CCM_CSCMR1_PERCLK_CLK_SEL_MASK)) |
                       CCM_CSCMR1_FLEXSPI_CLK_SEL(3) | CCM_CSCMR1_FLEXSPI_PODF(7) | CCM_CSCMR1_PERCLK_PODF(1));

        // Finally, Enable PLL_ARM, PLL_SYS and PLL_USB1
        CCM_ANALOG->PLL_ARM &= ~CCM_ANALOG_PLL_ARM_BYPASS_MASK;
        CCM_ANALOG->PLL_SYS &= ~CCM_ANALOG_PLL_SYS_BYPASS_MASK;
        CCM_ANALOG->PLL_USB1 &= ~CCM_ANALOG_PLL_USB1_BYPASS_MASK;
    }
    
    bl_api_init();
    
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
            {//default configOption setting
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
    status = flexspi_nor_get_config(FLEXSPI_NOR_INSTANCE, &config, &configOption);
    if (status != 0)
    {
        return status;
    }
    
    status = flexspi_nor_flash_init(FLEXSPI_NOR_INSTANCE, &config);
    
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

  for(uint32_t size = 0; size < count; size+=config.pageSize,
                                       buffer+=config.pageSize,
                                       addr+=config.pageSize)
  {
    if(flexspi_nor_flash_page_program(FLEXSPI_NOR_INSTANCE, &config, addr, (uint32_t *)buffer) != RESULT_OK)
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
  if(flexspi_nor_flash_erase(FLEXSPI_NOR_INSTANCE, &config, addr, block_size) != RESULT_OK)
  {
    status = RESULT_ERROR;
  }
  return status;
}

/*************************************************************************
 * Function Name: FlashChecksum
 * Parameters:  none
 *
 * Return: 0
 *
 * Description: Restore the performance mode.
 *************************************************************************/
OPTIONAL_CHECKSUM

uint32_t FlashChecksum(void const *begin, uint32_t count)
{
  return Crc16((uint8_t const *)begin, count);
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
