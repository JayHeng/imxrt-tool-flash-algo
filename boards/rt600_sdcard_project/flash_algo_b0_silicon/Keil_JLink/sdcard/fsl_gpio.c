/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_gpio.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
/*! @brief Array to map FGPIO instance number to clock name. */
static const clock_ip_name_t s_gpioClockName[] = GPIO_CLOCKS;
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */

#if FSL_FEATURE_SOC_RSTCTL0_COUNT | FSL_FEATURE_SOC_RSTCTL1_COUNT
/*! @brief GPIO reset control bit arrary */
static RSTCTL_RSTn_t const s_gpioReset[] = GPIO_RSTS_N;
#endif

/*******************************************************************************
* Prototypes
************ ******************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
void GPIO_PortInit(GPIO_Type *base, uint32_t port)
{
#if !(defined(FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL) && FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL)
    assert(port < ARRAY_SIZE(s_gpioClockName));

    /* Upgate the GPIO clock */
    CLOCK_EnableClock(s_gpioClockName[port]);
#endif /* FSL_SDK_DISABLE_DRIVER_CLOCK_CONTROL */
#if FSL_FEATURE_SOC_RSTCTL0_COUNT | FSL_FEATURE_SOC_RSTCTL1_COUNT
    RESET_PeripheralReset(s_gpioReset[port]);
#endif
}

void GPIO_PinInit(GPIO_Type *base, uint32_t port, uint32_t pin, const gpio_pin_config_t *config)
{
    if (config->pinDirection == kGPIO_DigitalInput)
    {
        base->DIR[port] &= ~(1U << pin);
    }
    else
    {
        /* Set default output value */
        if (config->outputLogic == 0U)
        {
            base->CLR[port] = (1U << pin);
        }
        else
        {
            base->SET[port] = (1U << pin);
        }
        /* Set pin direction */
        base->DIR[port] |= 1U << pin;
    }
}
