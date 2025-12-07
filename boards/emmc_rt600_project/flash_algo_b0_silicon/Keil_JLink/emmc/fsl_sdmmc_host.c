/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_sdmmc_host.h"
//#include "board.h"
//#include "fsl_gpio.h"
//#include "fsl_sdmmc_event.h"
#ifdef BOARD_USDHC_CD_PORT_BASE
#include "fsl_port.h"
#endif
#include "microseconds.h"
#include "sdmmc_init.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* ADMA table length united as word.
 *
 * One ADMA2 table item occupy two words which can transfer maximum 0xFFFFU bytes one time.
 * The more data to be transferred in one time, the bigger value of SDHC_ADMA_TABLE_WORDS need to be set.
 */
#if defined(BOARD_USDHC_ADMA_TABLE_MAX_ENTRIES)
#define USDHC_ADMA_TABLE_WORDS (BOARD_USDHC_ADMA_TABLE_MAX_ENTRIES * 2) /* define the ADMA descriptor table length */
#else
#define USDHC_ADMA_TABLE_WORDS (8U) /* define the ADMA descriptor table length */
#endif
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief SDMMCHOST detect card by GPIO.
 */
static void SDMMCHOST_DetectCardByGpio(const sdmmchost_detect_card_t *cd);

/*!
 * @brief SDMMCHOST detect card insert status by host controller.
 * @param base host base address.
 */
static void SDMMCHOST_DetectCardInsertByHost(SDMMCHOST_TYPE *base);

/*!
 * @brief SDMMCHOST transfer function.
 * @param base host base address.
 * @param content transfer configurations.
 */
static status_t SDMMCHOST_TransferFunction(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER *content);

/*!
 * @brief host controller error recovery.
 * @param host base address.
 */
static void SDMMCHOST_ErrorRecovery(SDMMCHOST_TYPE *base);

/*!
 * @brief card detect deinit function.
 */
static void SDMMCHOST_CardDetectDeinit(void);

/*!
 * @brief card detect deinit function.
 * @param host base address.
 * @param host detect card configuration.
 */
static status_t SDMMCHOST_CardDetectInit(SDMMCHOST_TYPE *base, const sdmmchost_detect_card_t *cd);
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* DMA descriptor should allocate at non-cached memory */
AT_NONCACHEABLE_SECTION_ALIGN(uint32_t g_usdhcAdma2Table[USDHC_ADMA_TABLE_WORDS], USDHC_ADMA2_ADDR_ALIGN);
static volatile bool s_sdInsertedFlag = false;
/*******************************************************************************
 * Code
 ******************************************************************************/
static void SDMMCHOST_DetectCardByGpio(const sdmmchost_detect_card_t *cd)
{
    if (SDMMCHOST_CARD_DETECT_GPIO_STATUS() != SDMMCHOST_CARD_INSERT_CD_LEVEL)
    {
        s_sdInsertedFlag = false;
        if (cd && (cd->cardRemoved))
        {
            cd->cardRemoved(false, cd->userData);
        }
    }
    else
    {
        s_sdInsertedFlag = true;
        if (cd && (cd->cardInserted))
        {
            cd->cardInserted(true, cd->userData);
        }
    }
}

static void SDMMCHOST_DetectCardInsertByHost(SDMMCHOST_TYPE *base)
{
    if (SDMMCHOST_CARD_DETECT_INSERT_STATUS(base))
    {
        s_sdInsertedFlag = true;
    }
}

static status_t SDMMCHOST_TransferFunction(SDMMCHOST_TYPE *base, SDMMCHOST_TRANSFER *content)
{
    usdhc_adma_config_t dmaConfig;
    status_t error = kStatus_Success;

    if (content->data != NULL)
    {
        memset(&dmaConfig, 0, sizeof(usdhc_adma_config_t));
        /* config adma */
        dmaConfig.dmaMode = USDHC_DMA_MODE;
#if !(defined(FSL_FEATURE_USDHC_HAS_NO_RW_BURST_LEN) && FSL_FEATURE_USDHC_HAS_NO_RW_BURST_LEN)
        dmaConfig.burstLen = kUSDHC_EnBurstLenForINCR;
#endif // !FSL_FEATURE_USDHC_HAS_NO_RW_BURST_LEN
        dmaConfig.admaTable = g_usdhcAdma2Table;
        dmaConfig.admaTableWords = USDHC_ADMA_TABLE_WORDS;
    }

    error = USDHC_TransferBlocking(base, &dmaConfig, content);

    if (error == kStatus_Fail)
    {
        /* host error recovery */
        SDMMCHOST_ErrorRecovery(base);
    }

    return error;
}

static void SDMMCHOST_ErrorRecovery(SDMMCHOST_TYPE *base)
{
    uint32_t status = 0U;
    /* get host present status */
    status = USDHC_GetPresentStatusFlags(base);
    /* check command inhibit status flag */
    if ((status & kUSDHC_CommandInhibitFlag) != 0U)
    {
        /* reset command line */
        USDHC_Reset(base, kUSDHC_ResetCommand, 10000U);
    }
    /* check data inhibit status flag */
    if ((status & kUSDHC_DataInhibitFlag) != 0U)
    {
        /* reset data line */
        USDHC_Reset(base, kUSDHC_ResetData, 10000U);
    }
}

static status_t SDMMCHOST_CardDetectInit(SDMMCHOST_TYPE *base, const sdmmchost_detect_card_t *cd)
{
    sdmmchost_detect_card_type_t cdType = kSDMMCHOST_DetectCardByGpioCD;

    if (cd != NULL)
    {
        cdType = cd->cdType;
    }

    if (cdType == kSDMMCHOST_DetectCardByGpioCD)
    {
        SDMMCHOST_CARD_DETECT_GPIO_INIT();
        /* check card insert or not */
        SDMMCHOST_DetectCardByGpio(cd);
    }
    else
    {
        /* enable card detect through DATA3 */
        if (cdType == kSDMMCHOST_DetectCardByHostDATA3)
        {
            SDMMCHOST_CARD_DETECT_DATA3_ENABLE(base, true);
        }
        /* enable card detect status */
        SDMMCHOST_CARD_DETECT_INSERT_ENABLE(base);
        SDMMCHOST_CARD_DETECT_REMOVE_ENABLE(base);
    }

    return kStatus_Success;
}

#if 0
static void SDMMCHOST_CardDetectDeinit(void)
{
    SDMMCEVENT_Delete(kSDMMCEVENT_CardDetect);
    s_sdInsertedFlag = false;
}
#endif

void SDMMCHOST_Delay(uint32_t milliseconds)
{
    // SDMMCEVENT_Delay(milliseconds);
    microseconds_delay(milliseconds * 1000);
}

#if 1
status_t SDMMCHOST_WaitCardDetectStatus(SDMMCHOST_TYPE *base, const sdmmchost_detect_card_t *cd, bool waitCardStatus)
{
    sdmmchost_detect_card_type_t cdType = kSDMMCHOST_DetectCardByGpioCD;

    if (cd != NULL)
    {
        cdType = cd->cdType;
    }

    if (waitCardStatus != s_sdInsertedFlag)
    {
        /* Wait card inserted. */
        do
        {
            if (cdType != kSDMMCHOST_DetectCardByGpioCD)
            {
                SDMMCHOST_DetectCardInsertByHost(base);
            }
            else
            {
                SDMMCHOST_DetectCardByGpio(cd);
            }
        } while (waitCardStatus != s_sdInsertedFlag);
    }

    return kStatus_Success;
}

bool SDMMCHOST_IsCardPresent(void)
{
    return s_sdInsertedFlag;
}
#endif

void SDMMCHOST_PowerOffCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr)
{
    if (pwr != NULL)
    {
        pwr->powerOff();
        SDMMCHOST_Delay(pwr->powerOffDelay_ms);
    }
    else
    {
        /* only SD card need card detect*/
        SDMMCHOST_ENABLE_SD_POWER(false);
        /* Delay several milliseconds to make card stable. */
        SDMMCHOST_Delay(500U);
    }
}

void SDMMCHOST_PowerOnCard(SDMMCHOST_TYPE *base, const sdmmchost_pwr_card_t *pwr)
{
    /* use user define the power on function  */
    if (pwr != NULL)
    {
        pwr->powerOn();
        SDMMCHOST_Delay(pwr->powerOnDelay_ms);
    }
    else
    {
        /* card power on */
        SDMMCHOST_ENABLE_SD_POWER(true);
        /* Delay several milliseconds to make card stable. */
        SDMMCHOST_Delay(500U);
    }
}

status_t SDMMCHOST_Init(SDMMCHOST_CONFIG *host, void *userData)
{
    usdhc_host_t *usdhcHost = (usdhc_host_t *)host;

    /* init card power control */
    // SDMMCHOST_INIT_SD_POWER();
    // SDMMCHOST_INIT_MMC_POWER();

    /* Initializes SDHC. */
    usdhcHost->config.dataTimeout = USDHC_DATA_TIMEOUT;
    usdhcHost->config.endianMode = USDHC_ENDIAN_MODE;
    usdhcHost->config.readWatermarkLevel = USDHC_READ_WATERMARK_LEVEL;
    usdhcHost->config.writeWatermarkLevel = USDHC_WRITE_WATERMARK_LEVEL;
#if !(defined(FSL_FEATURE_USDHC_HAS_NO_RW_BURST_LEN) && FSL_FEATURE_USDHC_HAS_NO_RW_BURST_LEN)
    usdhcHost->config.readBurstLen = USDHC_READ_BURST_LEN;
    usdhcHost->config.writeBurstLen = USDHC_WRITE_BURST_LEN;
#endif
    USDHC_Init(usdhcHost->base, &(usdhcHost->config));

    /* Define transfer function. */
    usdhcHost->transfer = SDMMCHOST_TransferFunction;
    /* event init timer */
    // SDMMCEVENT_InitTimer();
    /* card detect init */
    // SDMMCHOST_CardDetectInit(usdhcHost->base, (sdmmchost_detect_card_t *)userData);

    return kStatus_Success;
}

void SDMMCHOST_Reset(SDMMCHOST_TYPE *base)
{
    /* voltage switch to normal but not 1.8V */
    SDMMCHOST_SWITCH_VOLTAGE180V(base, false);
    /* Disable DDR mode */
    SDMMCHOST_ENABLE_DDR_MODE(base, false, 0U);
    /* disable tuning */
    SDMMCHOST_EXECUTE_STANDARD_TUNING_ENABLE(base, 0, 0, false);
    /* Disable HS400 mode */
    SDMMCHOST_ENABLE_HS400_MODE(base, false);
    /* Disable DLL */
    SDMMCHOST_ENABLE_STROBE_DLL(base, false);
    /* Clear data transfer direction bit */
    base->MIX_CTRL &= ~USDHC_MIX_CTRL_DTDSEL_MASK;
}

void SDMMCHOST_Deinit(void *host)
{
    usdhc_host_t *usdhcHost = (usdhc_host_t *)host;
    SDMMCHOST_Reset(usdhcHost->base);
    USDHC_Deinit(usdhcHost->base);
    // SDMMCHOST_CardDetectDeinit();
}
