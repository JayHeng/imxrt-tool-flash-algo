/***********************************************************************/
/*  This file is part of the ARM Toolchain package                     */
/*  Copyright (c) 2010 Keil - An ARM Company. All rights reserved.     */
/***********************************************************************/
/*                                                                     */
/*  FlashDev.C:  Flash Programming Functions adapted                   */
/*               for New Device 256kB Flash                            */
/*                                                                     */
/***********************************************************************/

#include "FlashOS.H" // FlashOS Structures
#include "bl_api.h"

#define FLEXSPI_NOR_INSTANCE 0
#define SECTOR_SIZE (4096)
#define BASE_ADDRESS (0x60000000)

flexspi_nor_config_t config = {1};

/*
 *  Initialize Flash Programming Functions
 *    Parameter:      adr:  Device Base Address
 *                    clk:  Clock Frequency (Hz)
 *                    fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int Init(unsigned long adr, unsigned long clk, unsigned long fnc)
{
    status_t status;
    serial_nor_config_option_t option;
    option.option0.U = 0xc1000405;
    option.option1.U = 0x00010000;
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

    bl_api_init();
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
    status = flexspi_nor_get_config(FLEXSPI_NOR_INSTANCE, &config, &option);
    if (status != kStatus_Success)
    {
        return (1);
    }
    status = flexspi_nor_flash_init(FLEXSPI_NOR_INSTANCE, &config);
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0); // Finished without Errors
    }
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit(unsigned long fnc)
{
    /* Add your Code */
    return (0); // Finished without Errors
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip(void)
{
    status_t status;
    status = flexspi_nor_flash_erase_all(FLEXSPI_NOR_INSTANCE, &config); // Erase all
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0); // Finished without Errors
    }
}

/*
 *  Erase Sector in Flash Memory
 *    Parameter:      adr:  Sector Address
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseSector(unsigned long adr)
{
    status_t status;
    adr    = adr - BASE_ADDRESS;
    status = flexspi_nor_flash_erase(FLEXSPI_NOR_INSTANCE, &config, adr, SECTOR_SIZE); // Erase 1 sector
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}

/*
 *  Program Page in Flash Memory
 *    Parameter:      adr:  Page Start Address
 *                    sz:   Page Size
 *                    buf:  Page Data
 *    Return Value:   0 - OK,  1 - Failed
 */

int ProgramPage(unsigned long adr, unsigned long sz, unsigned char *buf)
{
    status_t status;
    adr = adr - BASE_ADDRESS;
    // Program data to destination
    status = flexspi_nor_flash_page_program(FLEXSPI_NOR_INSTANCE, &config, adr, (uint32_t *)buf); // program 1 page
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
}
