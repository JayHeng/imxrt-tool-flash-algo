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

#include <arm_compat.h>
#include <stdbool.h>
#include <stdint.h>
#include "../FlashOS.H"        // FlashOS Structures
#include "fsl_romapi.h"
#include <string.h>

/** local definitions **/
#define RESULT_OK       0
#define RESULT_ERROR    1

#define FSPI1_FLASH_BASE_ADDR 0x28000000
#define FSPI2_FLASH_BASE_ADDR 0x04000000
#define FLASH_SECT_SIZE 0x1000

#if defined(RT1180_FPGA)
#define CONFIG_FLEXSPI1_OPTION0 0xC0000007
#define CONFIG_FLEXSPI1_OPTION1 0x00000000
#define CONFIG_FLEXSPI2_OPTION0 0xC1000004
#define CONFIG_FLEXSPI2_OPTION1 0x20100000
#elif defined(RT1180_EVB_A)
#define CONFIG_FLEXSPI1_OPTION0 0xC0000007
#define CONFIG_FLEXSPI1_OPTION1 0x00000000
#define CONFIG_FLEXSPI2_OPTION0 0xC0000007
#define CONFIG_FLEXSPI2_OPTION1 0x00000000
#elif defined(RT1180_EVK_FSPI1_QSPI)
#define CONFIG_FLEXSPI1_OPTION0 0xC0000007
#define CONFIG_FLEXSPI1_OPTION1 0x00000000
#define CONFIG_FLEXSPI2_OPTION0 0xC0000007
#define CONFIG_FLEXSPI2_OPTION1 0x00000000
#endif

flexspi_nor_config_t flashConfig;
serial_nor_config_option_t configOption;
uint32_t fspi_instance, flashBase;

static void preInit(void) {
    /*
     * Prevent unexpected interrupt, only for FPGA?
     */
    if (NVIC_GetEnableIRQ(GPIO1_0_IRQn))
    {
        NVIC_DisableIRQ(GPIO1_0_IRQn);
    }
    
    /*
     * Disable the WDOG
     */
    if ((RTWDOG1->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG1->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG1->CNT = 0xC520U;
        RTWDOG1->CNT = 0xD928U;
    }
    RTWDOG1->TOVAL = 0xFFFF;
    RTWDOG1->CS    = (uint32_t)((RTWDOG1->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    if ((RTWDOG2->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG2->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG2->CNT = 0xC520U;
        RTWDOG2->CNT = 0xD928U;
    }
    RTWDOG2->TOVAL = 0xFFFF;
    RTWDOG2->CS    = (uint32_t)((RTWDOG2->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    if ((RTWDOG3->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG3->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG3->CNT = 0xC520U;
        RTWDOG3->CNT = 0xD928U;
    }
    RTWDOG3->TOVAL = 0xFFFF;
    RTWDOG3->CS    = (uint32_t)((RTWDOG3->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    if ((RTWDOG4->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG4->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG4->CNT = 0xC520U;
        RTWDOG4->CNT = 0xD928U;
    }
    RTWDOG4->TOVAL = 0xFFFF;
    RTWDOG4->CS    = (uint32_t)((RTWDOG4->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    if ((RTWDOG5->CS & RTWDOG_CS_CMD32EN_MASK) != 0U)
    {
        RTWDOG5->CNT = 0xD928C520U; /* 0xD928C520U is the update key */
    }
    else
    {
        RTWDOG5->CNT = 0xC520U;
        RTWDOG5->CNT = 0xD928U;
    }
    RTWDOG5->TOVAL = 0xFFFF;
    RTWDOG5->CS    = (uint32_t)((RTWDOG5->CS) & ~RTWDOG_CS_EN_MASK) | RTWDOG_CS_UPDATE_MASK;

    /* Disable Systick which might be enabled by bootrom */
    if ((SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) != 0U)
    {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    }

#if (__CORTEX_M == 33)
    if ((XCACHE_PC->CCR & XCACHE_CCR_ENCACHE_MASK) == 0U) /* set XCACHE if not configured */
    {
        /* set command to invalidate all ways and write GO bit to initiate command */
        XCACHE_PC->CCR = XCACHE_CCR_INVW1_MASK | XCACHE_CCR_INVW0_MASK;
        XCACHE_PC->CCR |= XCACHE_CCR_GO_MASK;
        /* Wait until the command completes */
        while ((XCACHE_PC->CCR & XCACHE_CCR_GO_MASK) != 0U)
        {
        }
        /* Enable cache */
        XCACHE_PC->CCR = XCACHE_CCR_ENCACHE_MASK;

        __ISB();
        __DSB();
    }
#elif (__CORTEX_M == 7)
    if (SCB_CCR_IC_Msk != (SCB_CCR_IC_Msk & SCB->CCR)) {
        SCB_EnableICache();
    }
#endif

    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    __DSB();
    __ISB();
}

/*  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */
int Init (unsigned long adr, unsigned long clk, unsigned long fnc) {
    status_t status = RESULT_OK;

    preInit();

    ROM_API_Init();

    memset((void *)&flashConfig, 0U, sizeof(flexspi_nor_config_t));
    memset((void *)&configOption, 0U, sizeof(serial_nor_config_option_t));

    if (FSPI1_FLASH_BASE_ADDR == adr) 
    {
        fspi_instance = 1;
        flashBase = FSPI1_FLASH_BASE_ADDR;
        configOption.option0.U = CONFIG_FLEXSPI1_OPTION0;
        configOption.option1.U = CONFIG_FLEXSPI1_OPTION1;
        CCM->CLOCK_ROOT[21].CONTROL = 0x0;
        __DSB();
        __ISB();
    }
    else if (FSPI2_FLASH_BASE_ADDR == adr)
    {
        fspi_instance = 2;
        flashBase = FSPI2_FLASH_BASE_ADDR;
        configOption.option0.U = CONFIG_FLEXSPI2_OPTION0;
        configOption.option1.U = CONFIG_FLEXSPI2_OPTION1;
        CCM->CLOCK_ROOT[22].CONTROL = 0x0;
        __DSB();
        __ISB();
    }

    status = ROM_FLEXSPI_NorFlash_GetConfig(fspi_instance, &flashConfig, &configOption);
    status = ROM_FLEXSPI_NorFlash_Init(fspi_instance, &flashConfig);

    return status;
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

int EraseChip (void)
{
    status_t status;

    status =  ROM_FLEXSPI_NorFlash_EraseAll(fspi_instance, &flashConfig);
    return (int)status;
}


/*  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector (unsigned long adr)
{
    /*Erase Sector*/
    status_t status; 

    status =  ROM_FLEXSPI_NorFlash_Erase(fspi_instance, &flashConfig, adr - flashBase, flashConfig.sectorSize);
    return (int)status;
}


/*  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage (unsigned long adr, unsigned long sz, unsigned char *buf)
{
    status_t status = 0;

    for(uint32_t size = 0; size < sz; size+=flashConfig.pageSize,
            buf+=flashConfig.pageSize,
            adr+=flashConfig.pageSize)
    {
        status =  ROM_FLEXSPI_NorFlash_ProgramPage(fspi_instance, &flashConfig, adr - flashBase, (uint32_t *)buf);
    }

  return (int)status;
}

int FlashEraseDone(void)
{
    return 0;
}

int FlashProgramDone(void)
{
    return 0;
}

int DebugCodeMemRemap(void)
{
    return 0;
}