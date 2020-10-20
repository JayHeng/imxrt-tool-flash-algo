/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_device_registers.h"
#include "flexspi/fsl_flexspi.h"
#include "flexspi_nor/flexspi_nor_flash.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FREQ_396MHz (396000000U)
#define FREQ_480MHz (480000000U)
#define FREQ_528MHz (528000000U)
#define FREQ_24MHz  (24000000U)

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_00
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_01
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_02
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_03
#define SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04
#define SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04

#define SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_05
#define SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_06
#define SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_07
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_08
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_09
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_10
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_11
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04

#define SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_05
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_00
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_01
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_02
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_03
#define SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_04
#define SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_01
#define SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04

#define SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_05
#define SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_06
#define SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B0_00
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_07
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_08
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_09
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_10
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_11
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_B1_04

#define FLEXSPIA_MUX_VAL     IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIB_MUX_VAL     IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIA_SS1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS0_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)
#define FLEXSPIB_DQS_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Acutal R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Pull
// 100k ohm pull down resistor
#define FLEXSPI_DQS_SW_PAD_CTL_VAL                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
     IOMUXC_SW_PAD_CTL_PAD_HYS(1))

/*******************************************************************************
 * Codes
 ******************************************************************************/
//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue   = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue  = FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;

    if (flexspi_is_padsetting_override_enable(config))
    {
        csPadCtlValue   = config->csPadSettingOverride;
        dqsPadCtlValue  = config->dqsPadSettingOverride;
        sclkPadCtlValue = config->sclkPadSettingOverride;
        dataPadCtlValue = config->dataPadSettingOverride;
    }

    // Pinmux configuration for FLEXSPI PortA
    if (config->sflashA1Size || config->sflashA2Size)
    {
        if (config->sflashA2Size)
        {
            // FLEXSPIA_SS1_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX] = FLEXSPIA_SS1_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX] = csPadCtlValue;
        }

        // Basic pinmux configuration for FLEXSPI
        if (config->sflashA1Size)
        {
            // FLEXSPIA_SS0_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX] = csPadCtlValue;
        }

        // FLEXSPIA_SCLK
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX] = FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX] = sclkPadCtlValue;

        // FLEXSPIA_DATA0
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX] = FLEXSPIA_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX] = dataPadCtlValue;

        // FLEXSPIA_DATA1
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX] = FLEXSPIA_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX] = dataPadCtlValue;

        // FLEXSPIA_DATA2
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX] = FLEXSPIA_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX] = dataPadCtlValue;

        // FLEXSPIA_DATA3
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX] = FLEXSPIA_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX] = dataPadCtlValue;

        if ((config->sflashPadType == kSerialFlash_8Pads))
        {
            // FLEXSPIA_DATA4 / FLEXSPIB_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA5 / FLEXSPIB_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA6 / FLEXSPIB_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA7 / FLEXSPIB_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;
        }

        // Configure DQS pad
        if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
            (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
        {
            // FLEXSPIA_DQS
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX] = FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX] = dqsPadCtlValue;
        }

        // Configure Differential Clock pin
        if (flexspi_is_differential_clock_enable(config))
        {
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = sclkPadCtlValue;
        }
    }

    // Pinmux configuration for FLEXSPI PortB
    if (config->sflashB1Size || config->sflashB2Size)
    {
        if (config->sflashB2Size)
        {
            // FLEXSPIB_SS1_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX] = FLEXSPIB_SS1_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX] = csPadCtlValue;
        }

        // Basic pinmux configuration for FLEXSPI
        if (config->sflashB1Size)
        {
            // FLEXSPIB_SS0_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX] = FLEXSPIB_SS0_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX] = csPadCtlValue;
        }

        // FLEXSPIB_SCLK
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX] = FLEXSPIB_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX] = sclkPadCtlValue;

        // FLEXSPIB_DATA0
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIB_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

        // FLEXSPIB_DATA1
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIB_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

        // FLEXSPIB_DATA2
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIB_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

        // FLEXSPIB_DATA3
        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIB_MUX_VAL;
        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;

        // Configure DQS pad
        if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
            (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
        {
            // FLEXSPIB_DQS
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX] =
                FLEXSPIB_DQS_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX] = dqsPadCtlValue;
        }
    }
}

// Set failsafe settings
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
