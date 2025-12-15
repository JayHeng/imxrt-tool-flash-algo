/*
* Copyright 2014-2016 Freescale Semiconductor, Inc.
* Copyright 2016-2018 NXP
* All rights reserved.
*
* SPDX-License-Identifier: BSD-3-Clause
*
*/

#include "bootloader_common.h"
#include "fsl_device_registers.h"
#include "sdmmc_init.h"
#include "fsl_mmc.h"
#include "fsl_gpio.h"
#include "mmc_memory.h"
/*******************************************************************************
 * Definitons
 ******************************************************************************/

// SDHC0_PWR_POL
#define OTP_SDHC0_PWR_POL_FUSE_IDX (0x62)
#define OTP_SDHC0_PWR_POL_SHIFT (6u)
#define OTP_SDHC0_PWR_POL_MASK (0x1u << OTP_SDHC0_PWR_POL_SHIFT)
#define OTP_SDHC0_PWR_POL_VALUE() \
    ((OCOTP->OTP_SHADOW[OTP_SDHC0_PWR_POL_FUSE_IDX] & OTP_SDHC0_PWR_POL_MASK) >> OTP_SDHC0_PWR_POL_SHIFT)

// SDHC1_PWR_POL
#define OTP_SDHC1_PWR_POL_FUSE_IDX (0x62)
#define OTP_SDHC1_PWR_POL_SHIFT (7u)
#define OTP_SDHC1_PWR_POL_MASK (0x1u << OTP_SDHC1_PWR_POL_SHIFT)
#define OTP_SDHC1_PWR_POL_VALUE() \
    ((OCOTP->OTP_SHADOW[OTP_SDHC1_PWR_POL_FUSE_IDX] & OTP_SDHC1_PWR_POL_MASK) >> OTP_SDHC1_PWR_POL_SHIFT)

/*******************************************************************************
 * Variables
 ******************************************************************************/
const uint32_t usdhc_reset_pin_settings = IOPCTL_PIO_IBENA(1) | IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);
const uint32_t usdhc_vselect_pin_settings = IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1) |
                                            IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);
const uint32_t usdhc_cmd_pin_settings = IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1) |
                                        IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);
const uint32_t usdhc_clk_pin_settings = IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1) |
                                        IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);
const uint32_t usdhc_data_pin_settings = IOPCTL_PIO_PUPDENA(1) | IOPCTL_PIO_PUPDSEL(1) | IOPCTL_PIO_IBENA(1) |
                                         IOPCTL_PIO_SLEWRATE(0) | IOPCTL_PIO_FULLDRIVE(1);
/*******************************************************************************
 * Code
 ******************************************************************************/
static inline void IOPAD_Set(IOPCTL_Type *base, uint8_t port, uint8_t pin, uint32_t fsel, uint32_t setting)
{
    base->PIO[port][pin] = IOPCTL_PIO_FSEL(fsel) | ((setting) & (~IOPCTL_PIO_FSEL_MASK));
}

void usdhc_power_control_init(USDHC_Type *base)
{
    if (base == BOARD_USDHC0_BASEADDR)
    {
        IOPAD_Set(BOARD_USDHC0_RESET_B_IOPAD, usdhc_reset_pin_settings);
        gpio_pin_config_t sw_config = { kGPIO_DigitalOutput, !OTP_SDHC0_PWR_POL_VALUE() };
        GPIO_PortInit(BOARD_USDHC0_RESET_B_PORT);
        GPIO_PinInit(BOARD_USDHC0_RESET_B_GPIO, &sw_config);
    }
    else if (base == BOARD_USDHC1_BASEADDR)
    {
        IOPAD_Set(BOARD_USDHC1_RESET_B_IOPAD, usdhc_reset_pin_settings);
        gpio_pin_config_t sw_config = { kGPIO_DigitalOutput, !OTP_SDHC1_PWR_POL_VALUE() };
        GPIO_PortInit(BOARD_USDHC1_RESET_B_PORT);
        GPIO_PinInit(BOARD_USDHC1_RESET_B_GPIO, &sw_config);
    }
}

void usdhc_power_control(USDHC_Type *base, bool state)
{
    if (base == BOARD_USDHC0_BASEADDR)
    {
        GPIO_PinWrite(BOARD_USDHC0_RESET_B_GPIO, state);
    }
    else if (base == BOARD_USDHC1_BASEADDR)
    {
        GPIO_PinWrite(BOARD_USDHC1_RESET_B_GPIO, state);
    }
}

void usdhc_vselect_init(USDHC_Type *base)
{
    if (base == BOARD_USDHC0_BASEADDR)
    {
        IOPAD_Set(BOARD_USDHC0_VSELECT_IOPAD, usdhc_vselect_pin_settings);
    }
    else if (base == BOARD_USDHC1_BASEADDR)
    {
        IOPAD_Set(BOARD_USDHC1_VSELECT_IOPAD, usdhc_vselect_pin_settings);
    }
}

void mmc_pinmux_config(USDHC_Type *base, mmc_data_bus_width_t busWidth)
{
    if (base == BOARD_USDHC0_BASEADDR)
    {
        switch (busWidth)
        {
            case kMMC_DataBusWidth8bitDDR:
            case kMMC_DataBusWidth8bit:
                IOPAD_Set(BOARD_USDHC0_DATA4_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA5_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA6_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA7_IOPAD, usdhc_data_pin_settings);
            case kMMC_DataBusWidth4bitDDR:
            case kMMC_DataBusWidth4bit:
                IOPAD_Set(BOARD_USDHC0_DATA1_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA2_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA3_IOPAD, usdhc_data_pin_settings);
            case kMMC_DataBusWidth1bit:
            default:
                IOPAD_Set(BOARD_USDHC0_CMD_IOPAD, usdhc_cmd_pin_settings);
                IOPAD_Set(BOARD_USDHC0_CLK_IOPAD, usdhc_clk_pin_settings);
                IOPAD_Set(BOARD_USDHC0_DATA0_IOPAD, usdhc_data_pin_settings);
        }
    }
    else if (base == BOARD_USDHC1_BASEADDR)
    {
        switch (busWidth)
        {
            case kMMC_DataBusWidth8bitDDR:
            case kMMC_DataBusWidth8bit:
                IOPAD_Set(BOARD_USDHC1_DATA4_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA5_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA6_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA7_IOPAD, usdhc_data_pin_settings);
            case kMMC_DataBusWidth4bitDDR:
            case kMMC_DataBusWidth4bit:
                IOPAD_Set(BOARD_USDHC1_DATA1_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA2_IOPAD, usdhc_data_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA3_IOPAD, usdhc_data_pin_settings);
            case kMMC_DataBusWidth1bit:
            default:
                IOPAD_Set(BOARD_USDHC1_CMD_IOPAD, usdhc_cmd_pin_settings);
                IOPAD_Set(BOARD_USDHC1_CLK_IOPAD, usdhc_clk_pin_settings);
                IOPAD_Set(BOARD_USDHC1_DATA0_IOPAD, usdhc_data_pin_settings);
        }
    }
}

status_t get_mmc_default_configuration(mmc_card_t *card)
{
    // Used by memory interface. For LPCNext0, always failed.
    status_t status = kStatus_Fail;
    return status;
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
