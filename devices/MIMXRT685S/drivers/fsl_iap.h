/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __FSL_IAP_H_
#define __FSL_IAP_H_

#include "fsl_common.h"
/*!
 * @addtogroup rom_api
 * @{
*/

/*! @file */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif

status_t IAP_OtpInit(uint32_t src_clk_freq);

status_t IAP_OtpDeinit(void);

status_t IAP_OtpFuseRead(uint32_t addr, uint32_t *data);

status_t IAP_OtpFuseProgram(uint32_t addr, uint32_t data, bool lock);

status_t IAP_OtpCrcCalc(uint32_t *src, uint32_t numberOfWords, uint32_t *crcChecksum);

status_t IAP_OtpShadowRegisterReload(void);

status_t IAP_OtpCrcCheck(uint32_t start_addr, uint32_t end_addr, uint32_t crc_addr);

#if defined(__cplusplus)
}
#endif

#endif /* __FSL_IAP_H_ */
