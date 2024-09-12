/*
 * Copyright 2018 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

#include "FlashOS.h" // FlashOS Structures

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI           FLEXSPI1
#define FLASH_SIZE                (FLASH_BASE_SIZE/0x400)
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI1_AMBA_BASE

#define EXAMPLE_FLEXSPI_CLOCK     kCLOCK_Flexspi1

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL        7
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          13
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD     0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS         1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE        2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR        3
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE 6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD   4
#define NOR_CMD_LUT_SEQ_IDX_READID             8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG     9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI           10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI            11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP          5

#define CUSTOM_LUT_LENGTH        60

///////////////////////////////////////////////
// IS25WP128, SR1[6]
/*
#define FLASH_QUAD_ENABLE        0x40
#define FLASH_QENABLE_CMD        0x01
#define FLASH_QPROG_CMD          0x32
#define FLASH_SECERASE_CMD       0xD7
#define FLASH_QREAD_DUMMY        0x06
*/

///////////////////////////////////////////////
/*
// W25Q128JW, SR2[1]
#define FLASH_QUAD_ENABLE        0x02
#define FLASH_QENABLE_CMD        0x31
#define FLASH_QPROG_CMD          0x32
#define FLASH_SECERASE_CMD       0xD7
#define FLASH_QREAD_DUMMY        0x06
*/

///////////////////////////////////////////////
// S25HS512T, CFR1[1]

#define FLASH_QUAD_ENABLE        0x0200
#define FLASH_QENABLE_CMD        0x01
#define FLASH_QPROG_CMD          0xFF  // doesn't support
#define FLASH_SECERASE_CMD       0xD8
#define FLASH_QREAD_DUMMY        0x0A  // MODE bit cycles + dummy cycles


#define FLASH_BUSY_STATUS_POL    1
#define FLASH_BUSY_STATUS_OFFSET 0

/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
static inline void flexspi_clock_init(void)
{
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(kCLOCK_Root_Flexspi1, 4);
    CLOCK_SetRootClockMux(kCLOCK_Root_Flexspi1, 5);
}
/*${prototype:end}*/

#endif /* _APP_H_ */
