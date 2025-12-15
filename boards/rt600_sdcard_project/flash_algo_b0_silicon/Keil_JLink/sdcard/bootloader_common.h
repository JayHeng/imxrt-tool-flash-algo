/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOTLOADER_COMMON_H__
#define __BOOTLOADER_COMMON_H__

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "fsl_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////

//! @name Alignment macros
//@{
/* @{ */
#if (defined(__ICCARM__))
#define BL_PRAGMA(x) _Pragma(#x)
/*! Macro to define a variable with alignbytes alignment */
#define BL_ALIGN(alignbytes) BL_PRAGMA(data_alignment = alignbytes)
#define BL_SECTION(x) @x
#elif defined(__CC_ARM)
/*! Macro to define a variable with alignbytes alignment */
#define BL_ALIGN(alignbytes) __align(alignbytes)
#define BL_SECTION(x) __attribute__((section(x)))
#elif defined(__GNUC__)
/*! Macro to define a variable with alignbytes alignment */
#define BL_ALIGN(alignbytes) __attribute__((aligned(alignbytes)))
#define BL_SECTION(x) __attribute__((section(x)))
#else
#error Toolchain not supported
#define BL_ALIGN(alignbytes)
#define BL_SECTION(x)
#endif

/*! @brief Alignment(down) utility. */
#if !defined(ALIGN_DOWN)
#define ALIGN_DOWN(x, a) (((uint32_t)(x)) & ~((uint32_t)(a)-1u))
#endif

/*! @brief Alignment(up) utility. */
#if !defined(ALIGN_UP)
#define ALIGN_UP(x, a) ALIGN_DOWN((uint32_t)(x) + (uint32_t)(a)-1u, a)
#endif
//@}

//! @brief Bootloader status group numbers.
//!
//! @ingroup bl_core
enum _bl_status_groups
{
    kStatusGroup_Bootloader = 100,      //!< Bootloader status group number (100).
    kStatusGroup_MemoryInterface = 102, //!< Memory interface status group number (102).
};

#if defined(__CC_ARM)
#pragma anon_unions
#endif

//! @brief Bootloader clock option
typedef enum _bootloader_clock_option
{
    kClockOption_EnterBootloader = 0, //!< Clock option for entering bootloader
    kClockOption_ExitBootloader = 1,  //!< Clock option for exiting bootloader
} bootloader_clock_option_t;

////////////////////////////////////////////////////////////////////////////////
// Prototypes
////////////////////////////////////////////////////////////////////////////////

//! @addtogroup bl_hw
//! @{

//! @brief Initialize the hardware such as pinmux.
void init_hardware(void);

//! @brief Configure hardware clocks.
void configure_clocks(bootloader_clock_option_t option);


//! @}

#endif // __BOOTLOADER_COMMON_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
