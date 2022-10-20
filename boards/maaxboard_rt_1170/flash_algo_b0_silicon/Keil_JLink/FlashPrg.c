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
#ifdef USE_ROM_API
#include "fsl_romapi.h"
#else
#include "clock_config.h"
#include "fsl_iomuxc.h"
#endif

#define FLEXSPI_NOR_INSTANCE   1
#define FLEXSPIx               ((FLEXSPI_NOR_INSTANCE == 1) ? FLEXSPI1 : FLEXSPI2)
#define FLASH_ALGO_SECTOR_SIZE (0x1000)

#ifndef LOG_ENABLE
#define LOG_ENABLE 0
#endif

#ifdef USE_ROM_API
/* Init this global variable to workaround of the issue to running this flash algo in Segger */
static flexspi_nor_config_t *config = (flexspi_nor_config_t *)0x20230000;
#include "fsl_clock.h"
#else
#include "fsl_clock.h"
#include "fsl_flexspi.h"
#include "app.h"
extern status_t flexspi_nor_flash_erase_sector(FLEXSPI_Type *base, uint32_t address);
extern status_t flexspi_nor_flash_page_program(FLEXSPI_Type *base, uint32_t dstAddr, const uint32_t *src);
extern status_t flexspi_nor_get_vendor_id(FLEXSPI_Type *base, uint8_t *vendorId);
extern status_t flexspi_nor_enable_quad_mode(FLEXSPI_Type *base);
extern status_t flexspi_nor_erase_chip(FLEXSPI_Type *base);
extern void flexspi_nor_flash_init(FLEXSPI_Type *base);
#define SECTOR_MIN_SIZE 0x1000

/*${function:start}*/
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 132000000,
    .flashSize            = FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
    /* Normal read mode -SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Fast read mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Fast read quad mode - SDR */
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
        kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06, kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04),

    /* Read extend parameters */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Write Enable */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Erase Sector  */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xD7, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),

    /* Page Program - single mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Page Program - quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32, kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18),
    [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read ID */
    [4 * NOR_CMD_LUT_SEQ_IDX_READID] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enable Quad mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01, kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04),

    /* Enter QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Exit QPI mode */
    [4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),

    /* Read status register */
    [4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05, kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04),

    /* Erase whole chip */
    [4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] =
        FLEXSPI_LUT_SEQ(kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7, kFLEXSPI_Command_STOP, kFLEXSPI_1PAD, 0),
};

void init_flexspi_pins(unsigned int instance)
{
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_05_FLEXSPI1_A_DQS, /* GPIO_SD_B2_05 is configured as FLEXSPI1_A_DQS */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_05 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_06_FLEXSPI1_A_SS0_B, /* GPIO_SD_B2_06 is configured as FLEXSPI1_A_SS0_B */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_06 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_07_FLEXSPI1_A_SCLK, /* GPIO_SD_B2_07 is configured as FLEXSPI1_A_SCLK */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_07 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_08_FLEXSPI1_A_DATA00, /* GPIO_SD_B2_08 is configured as FLEXSPI1_A_DATA00 */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_08 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_09_FLEXSPI1_A_DATA01, /* GPIO_SD_B2_09 is configured as FLEXSPI1_A_DATA01 */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_09 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_10_FLEXSPI1_A_DATA02, /* GPIO_SD_B2_10 is configured as FLEXSPI1_A_DATA02 */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_10 */
    IOMUXC_SetPinMux(IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03, /* GPIO_SD_B2_11 is configured as FLEXSPI1_A_DATA03 */
                     1U); /* Software Input On Field: Force input path of pad GPIO_SD_B2_11 */

    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_05_FLEXSPI1_A_DQS,    /* GPIO_SD_B2_05 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_06_FLEXSPI1_A_SS0_B,  /* GPIO_SD_B2_06 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_07_FLEXSPI1_A_SCLK,   /* GPIO_SD_B2_07 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_08_FLEXSPI1_A_DATA00, /* GPIO_SD_B2_08 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_09_FLEXSPI1_A_DATA01, /* GPIO_SD_B2_09 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_10_FLEXSPI1_A_DATA02, /* GPIO_SD_B2_10 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
    IOMUXC_SetPinConfig(IOMUXC_GPIO_SD_B2_11_FLEXSPI1_A_DATA03, /* GPIO_SD_B2_11 PAD functional properties : */
                        0x0AU);                                 /* PDRV Field: normal driver
                                                                   Pull Down Pull Up Field: PD
                                                                   Open Drain Field: Disabled */
}
#endif

#define ERROR_LOG_ADDR  0x20240000
#define INIT_OFF        0x0000
#define ERASESECTOR_OFF 0x1000
#define PROG_OFF        0x8000
#define ERASECHIP_OFF   0x0004
#define VERIFY_OFF      0x18000

void log_result(unsigned long adr, unsigned long data)
{
#if LOG_ENABLE
    *(unsigned int *)adr = data;
#endif
}

static void restore_clock()
{
    unsigned int i = 0;
    for (i = 0; i < 79; i++)
    {
        CCM->CLOCK_ROOT[i].CONTROL = 0;
    }
}

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
    static unsigned int index = 0;
    unsigned int state        = 0;
    log_result(ERROR_LOG_ADDR + INIT_OFF, index++);

    if ((*(unsigned int *)0x40C84800) == 0x1170A0)
    {
#if __CORTEX_M == 7
        if (SCB_CCR_DC_Msk == (SCB_CCR_DC_Msk & SCB->CCR))
        {
            SCB_DisableDCache();
        }
#endif
        restore_clock();
        CCM->CLOCK_ROOT[kCLOCK_Root_M7].CONTROL = 0x200;
        CCM->CLOCK_ROOT[kCLOCK_Root_M4].CONTROL = 0x201;
        /* PLL LDO shall be enabled first before enable PLLs */
        CLOCK_EnableOsc24M();
        /* SYS PLL2 528MHz. */
        const clock_sys_pll_config_t sysPllConfig = {
            .loopDivider = 1,
            /* Using 24Mhz OSC */
            .mfn = 0,
            .mfi = 22,
        };
        CLOCK_InitSysPll2(&sysPllConfig);
        const clock_sys_pll3_config_t sysPll3Config = {
            .divSelect = 3,
        };
        CLOCK_InitSysPll3(&sysPll3Config);

        CCM->CLOCK_ROOT[kCLOCK_Root_Flexspi1].CONTROL_SET = 0x503;
    }

    SRC->GPR[9]            = 0;
    IOMUXC_LPSR_GPR->GPR26 = 0x4200;

#ifdef USE_ROM_API
    ROM_API_Init();
    serial_nor_config_option_t option;
    option.option0.U = 0xc0000007U;

    status = ROM_FLEXSPI_NorFlash_GetConfig(FLEXSPI_NOR_INSTANCE, config, &option);
    log_result(ERROR_LOG_ADDR + INIT_OFF + (state++) * 4, status);
    if (status != kStatus_Success)
    {
        return (1);
    }

    status = ROM_FLEXSPI_NorFlash_Init(FLEXSPI_NOR_INSTANCE, config);
    log_result(ERROR_LOG_ADDR + INIT_OFF + (state++) * 4, status);
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0); // Finished without Errors
    }
#else

    init_flexspi_pins(FLEXSPI_NOR_INSTANCE);
    flexspi_nor_flash_init(FLEXSPIx);

    /* Enter quad mode. */
    status = flexspi_nor_enable_quad_mode(FLEXSPIx);
    log_result(ERROR_LOG_ADDR + INIT_OFF + (state++) * 4, status);
    if (status != kStatus_Success)
    {
        return 1;
    }
    return 0;
#endif
}

/*
 *  De-Initialize Flash Programming Functions
 *    Parameter:      fnc:  Function Code (1 - Erase, 2 - Program, 3 - Verify)
 *    Return Value:   0 - OK,  1 - Failed
 */

int UnInit(unsigned long fnc)
{
    return (0); // Finished without Errors
}

/*
 *  Erase complete Flash Memory
 *    Return Value:   0 - OK,  1 - Failed
 */

int EraseChip(void)
{
    status_t status;
#ifdef USE_ROM_API
    status = ROM_FLEXSPI_NorFlash_EraseAll(FLEXSPI_NOR_INSTANCE, config); // Erase all
#else
    status = flexspi_nor_erase_chip(FLEXSPIx);
#endif
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
#ifndef USE_ROM_API
    unsigned int i = 0;
#endif
    adr = adr - FLASH_BASE_ADDRESS;
    if (adr % FLASH_ALGO_SECTOR_SIZE)
    {
        return (1);
    }

#ifdef USE_ROM_API
    status = ROM_FLEXSPI_NorFlash_Erase(FLEXSPI_NOR_INSTANCE, config, adr, FLASH_ALGO_SECTOR_SIZE); // Erase 1 sector
#else
    for (i = 0; i < FLASH_ALGO_SECTOR_SIZE / FLASH_SECTOR_SIZE; i++)
    {
        status = flexspi_nor_flash_erase_sector(FLEXSPIx, adr + (i * FLASH_SECTOR_SIZE));
#endif

#ifdef USE_ROM_API
    if (status != kStatus_Success)
    {
        return (1);
    }
    else
    {
        return (0);
    }
#else
        if (status != kStatus_Success)
        {
            return (1);
        }
    }
    return (0);
#endif
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
    unsigned int i                          = 0;
    unsigned char page_buf[FLASH_PAGE_SIZE] = {0xFF};

    adr = adr - FLASH_BASE_ADDRESS;
    if (adr % FLASH_PAGE_SIZE)
    {
        memcpy(page_buf, (const void*)(adr + FLASH_BASE_ADDRESS), adr % FLASH_PAGE_SIZE);
        memcpy(page_buf + adr % FLASH_PAGE_SIZE, (const void *)(buf), FLASH_PAGE_SIZE - adr % FLASH_PAGE_SIZE);
#ifdef USE_ROM_API
        status = ROM_FLEXSPI_NorFlash_ProgramPage(FLEXSPI_NOR_INSTANCE, config, adr - adr % FLASH_PAGE_SIZE, (const uint32_t *)page_buf); // program 1 page
#else
        status = flexspi_nor_flash_page_program(FLEXSPIx, adr - adr % FLASH_PAGE_SIZE, (const uint32_t*)page_buf);
#endif
        if (status != kStatus_Success)
        {
            return (1);
        }
        adr += FLASH_PAGE_SIZE - (adr % FLASH_PAGE_SIZE);
        buf += FLASH_PAGE_SIZE - adr % FLASH_PAGE_SIZE;
        sz -= FLASH_PAGE_SIZE - adr % FLASH_PAGE_SIZE;
    }
    // Program data to destination
    for (i = 0; i < sz / FLASH_PAGE_SIZE; i++)
    {
#ifdef USE_ROM_API
        status = ROM_FLEXSPI_NorFlash_ProgramPage(FLEXSPI_NOR_INSTANCE, config, adr + i * FLASH_PAGE_SIZE, (const uint32_t *)(buf + i * FLASH_PAGE_SIZE)); // program 1 page
#else
        status =
            flexspi_nor_flash_page_program(FLEXSPIx, adr + i * FLASH_PAGE_SIZE, (void *)(buf + i * FLASH_PAGE_SIZE));
#endif
        if (status != kStatus_Success)
        {
            return (1);
        }
    }

    if (sz - (i * FLASH_PAGE_SIZE))
    {
        memcpy(page_buf, buf + i * FLASH_PAGE_SIZE, sz % FLASH_PAGE_SIZE);
        memcpy(page_buf + sz % FLASH_PAGE_SIZE, (const void *)(adr + i * FLASH_PAGE_SIZE + FLASH_BASE_ADDRESS + sz % FLASH_PAGE_SIZE), FLASH_PAGE_SIZE - sz % FLASH_PAGE_SIZE);
#ifdef USE_ROM_API
        status = ROM_FLEXSPI_NorFlash_ProgramPage(FLEXSPI_NOR_INSTANCE, config, adr + i * FLASH_PAGE_SIZE, (const uint32_t *)page_buf); // program 1 page
#else
        status = flexspi_nor_flash_page_program(FLEXSPIx, adr + i * FLASH_PAGE_SIZE, (void *)page_buf);
#endif
        if (status != kStatus_Success)
        {
            return (1);
        }
    }

    return (0);
}
