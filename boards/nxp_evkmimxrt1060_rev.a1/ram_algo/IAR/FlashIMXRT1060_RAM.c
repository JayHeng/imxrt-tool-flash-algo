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
#include "fsl_device_registers.h"
#include "fsl_clock.h"
#include "fsl_rtwdog.h"
#include "fsl_wdog.h"

/** local definitions **/
   
/** default settings **/

/** external functions **/

/** external data **/

/** internal functions **/

/** public data **/

/** private data **/

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
  uint32_t addr = (uint32_t)block_start+offset_into_block;

  memcpy((void *)addr, buffer, count);
  
  return RESULT_OK;
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
  memset(block_start, 0xFF, block_size);
  
  return RESULT_OK;
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
