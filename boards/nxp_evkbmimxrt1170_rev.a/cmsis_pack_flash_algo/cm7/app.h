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
#include "fsl_clock.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define EXAMPLE_FLEXSPI           FLEXSPI1
#define FLASH_SIZE                (FLASH_BASE_SIZE/0x400)
#define EXAMPLE_FLEXSPI_AMBA_BASE FlexSPI1_AMBA_BASE
#define EXAMPLE_FLEXSPI_CLOCK     kCLOCK_Flexspi1
#define EXAMPLE_FLEXSPI_PORT      kFLEXSPI_PortA1
#define EXAMPLE_FLEXSPI_SAMP_SRC  kFLEXSPI_ReadSampleClkLoopbackInternally // kFLEXSPI_ReadSampleClkLoopbackFromDqsPad
#define EXAMPLE_FLEXSPI_PIN_SEL   1

#define FLASH_MODEL_IS25WP128                   (0x00)
#define FLASH_MODEL_W25Q128JW                   (0x10)
#define FLASH_MODEL_W25Q512NW_ADDR4B            (0x11)
#define FLASH_MODEL_W25Q40CV                    (0x12)
#define FLASH_MODEL_MX25L25645G_ADDR4B          (0x20)
#define FLASH_MODEL_S25HS512T_ADDR3B            (0x30)
#define FLASH_MODEL_S25HS512T_ADDR4B            (0x31)
#define CONFIG_FLASH_MODEL                      FLASH_MODEL_W25Q40CV
#define CONFIG_FLASH_FREQ_MHz                   (30)

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
#if (CONFIG_FLASH_MODEL == FLASH_MODEL_IS25WP128)
#define FLASH_QUAD_ENABLE        0x40
#define FLASH_QENABLE_CMD        0x01
#define FLASH_ADDR_BITS          (0x18)
#define FLASH_SPROG_CMD          0x02
#define FLASH_QPROG_CMD          0x32
#define FLASH_SECERASE_CMD       0xD8
#define FLASH_QREAD_CMD          0xEB
#define FLASH_QREAD_DUMMY        0x06
///////////////////////////////////////////////
// W25Q128JW, SR2[1]
// W25Q128JV, SR2[1]
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_W25Q128JW)
#define FLASH_QUAD_ENABLE        0x02
#define FLASH_QENABLE_CMD        0x31
#define FLASH_ADDR_BITS          (0x18)
#define FLASH_SPROG_CMD          0x02
#define FLASH_QPROG_CMD          0x32
#define FLASH_SECERASE_CMD       0xD8
#define FLASH_QREAD_CMD          0xEB
#define FLASH_QREAD_DUMMY        0x06
///////////////////////////////////////////////
// W25Q512NW, SR2[1]
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_W25Q512NW_ADDR4B)
#define FLASH_QUAD_ENABLE        0x02
#define FLASH_QENABLE_CMD        0x31
#define FLASH_ADDR_BITS          (0x20)
#define FLASH_SPROG_CMD          0x12
#define FLASH_QPROG_CMD          0x34
#define FLASH_SECERASE_CMD       0xDC
#define FLASH_QREAD_CMD          0xEC
#define FLASH_QREAD_DUMMY        0x06
///////////////////////////////////////////////
// W25Q40CV
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_W25Q40CV)
#define FLASH_QUAD_ENABLE        0xFF  // doesn't support
#define FLASH_QENABLE_CMD        0xFF  // doesn't support
#define FLASH_ADDR_BITS          (0x18)
#define FLASH_SPROG_CMD          0x02
#define FLASH_QPROG_CMD          0xFF  // doesn't support
#define FLASH_SECERASE_CMD       0xD8
#define FLASH_QREAD_CMD          0xFF  // doesn't support
#define FLASH_QREAD_DUMMY        0xFF  // doesn't support
///////////////////////////////////////////////
// MX25L25645G, SR1[6]
// MX25U25643G, SR1[6]
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_MX25L25645G_ADDR4B)
#define FLASH_QUAD_ENABLE        0x40
#define FLASH_QENABLE_CMD        0x01
#define FLASH_ADDR_BITS          (0x20)
#define FLASH_SPROG_CMD          0x12
#define FLASH_QPROG_CMD          0x3E
#define FLASH_SECERASE_CMD       0xDC
#define FLASH_QREAD_CMD          0xEC
#define FLASH_QREAD_DUMMY        0x06
///////////////////////////////////////////////
// S25HS512T, CFR1[1]
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_S25HS512T_ADDR3B)
#define FLASH_QUAD_ENABLE        0x0200
#define FLASH_QENABLE_CMD        0x01
#define FLASH_ADDR_BITS          (0x18)
#define FLASH_SPROG_CMD          0x02
#define FLASH_QPROG_CMD          0xFF  // doesn't support
#define FLASH_SECERASE_CMD       0xD8
#define FLASH_QREAD_CMD          0xEB
#elif (CONFIG_FLASH_MODEL == FLASH_MODEL_S25HS512T_ADDR4B)
#define FLASH_QUAD_ENABLE        0x0200
#define FLASH_QENABLE_CMD        0x01
#define FLASH_ADDR_BITS          (0x20)
#define FLASH_SPROG_CMD          0x12
#define FLASH_QPROG_CMD          0xFF  // doesn't support
#define FLASH_SECERASE_CMD       0xDC
#define FLASH_QREAD_CMD          0xEC
#define FLASH_QREAD_DUMMY        0x0A  // MODE bit cycles + dummy cycles
#endif

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
	  clock_root_t root;
#if (EXAMPLE_FLEXSPI_AMBA_BASE == FlexSPI1_AMBA_BASE)
    root = kCLOCK_Root_Flexspi1;
#elif (EXAMPLE_FLEXSPI_AMBA_BASE == FlexSPI2_AMBA_BASE)
	  root = kCLOCK_Root_Flexspi2;
#endif
#if (CONFIG_FLASH_FREQ_MHz == 133)
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(root, 4);
	  // SysPLL2 - 528MHz
    CLOCK_SetRootClockMux(root, 5);
#elif (CONFIG_FLASH_FREQ_MHz == 80)
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(root, 7);
	  // SysPLL2 - 528MHz
    CLOCK_SetRootClockMux(root, 5);
#elif (CONFIG_FLASH_FREQ_MHz == 60)
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(root, 9);
	  // SysPLL2 - 528MHz
    CLOCK_SetRootClockMux(root, 5);
#elif (CONFIG_FLASH_FREQ_MHz == 30)
    /*Clock setting for flexspi1*/
    CLOCK_SetRootClockDiv(root, 18);
	  // SysPLL2 - 528MHz
    CLOCK_SetRootClockMux(root, 5);
#endif
}
/*${prototype:end}*/

#endif /* _APP_H_ */
