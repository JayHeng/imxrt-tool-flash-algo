/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _FSL_MIPI_DSI_SMARTDMA_H_
#define _FSL_MIPI_DSI_SMARTDMA_H_

#include "fsl_mipi_dsi.h"
#include "fsl_smartdma.h"

/*!
 * @addtogroup mipi_dsi_smartdma
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @name Driver version */
/*@{*/
#define FSL_MIPI_DSI_SMARTDMA_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/* Forward declaration of the handle typedef. */
typedef struct _dsi_smartdma_handle dsi_smartdma_handle_t;

/*!
 * @brief MIPI DSI callback for finished transfer.
 *
 * When transfer finished, one of these status values will be passed to the user:
 * - @ref kStatus_Success Data transfer finished with no error.
 */
typedef void (*dsi_smartdma_callback_t)(MIPI_DSI_HOST_Type *base,
                                        dsi_smartdma_handle_t *handle,
                                        status_t status,
                                        void *userData);

/*! @brief The pixel format feed SMARTDMA. */
typedef enum _dsi_smartdma_input_pixel_format
{
    kDSI_SMARTDMA_InputPixelFormatRGB565, /*!< RGB565. */
    kDSI_SMARTDMA_InputPixelFormatRGB888, /*!< RGB888. */
} dsi_smartdma_input_pixel_format_t;

/*! @brief The pixel format sent on MIPI DSI data lanes. */
typedef enum _dsi_smartdma_output_pixel_format
{
    kDSI_SMARTDMA_OutputPixelFormatRGB565, /*!< RGB565. */
    kDSI_SMARTDMA_OutputPixelFormatRGB888, /*!< RGB888. */
} dsi_smartdma_output_pixel_format_t;

/*! @brief The pixel format sent on MIPI DSI data lanes. */
typedef struct _dsi_smartdma_write_mem_transfer
{
    dsi_smartdma_input_pixel_format_t inputFormat;   /*!< Input format. */
    dsi_smartdma_output_pixel_format_t outputFormat; /*!< Output format. */
    const uint8_t *data;                             /*!< Data to send. */
    size_t dataSize;                                 /*!< The byte count to be write. */
} dsi_smartdma_write_mem_transfer_t;

/*! @brief MIPI DSI transfer handle structure */
struct _dsi_smartdma_handle
{
    MIPI_DSI_HOST_Type *dsi;          /*!< MIPI DSI peripheral. */
    volatile bool isBusy;             /*!< MIPI DSI is busy with data transfer. */
    dsi_smartdma_callback_t callback; /*!< DSI callback */
    void *userData;                   /*!< Callback parameter */
    smartdma_dsi_param_t param;       /*!< Parameter for smartdma function. */
    uint32_t smartdmaStack[16];       /*!< Stack for smartdma function. */
};

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Transactional
 * @{
 */

/*!
 * @brief Create the MIPI DSI SMARTDMA handle.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle Handle pointer.
 * @param callback Callback function.
 * @param userData User data.
 */
status_t DSI_TransferCreateHandleSMARTDMA(MIPI_DSI_HOST_Type *base,
                                          dsi_smartdma_handle_t *handle,
                                          dsi_smartdma_callback_t callback,
                                          void *userData);

/*!
 * @brief Write display controller video memory using SMARTDMA.
 *
 * Perform data transfer using SMARTDMA, when transfer finished,
 * upper layer could be informed through callback function.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle pointer to dsi_smartdma_handle_t structure which stores the transfer state.
 * @param xfer Pointer to the transfer structure.
 *
 * @retval kStatus_Success Data transfer started successfully.
 * @retval kStatus_DSI_Busy Failed to start transfer because DSI is busy with pervious transfer.
 * @retval kStatus_DSI_NotSupported Transfer format not supported.
 */
status_t DSI_TransferWriteMemorySMARTDMA(MIPI_DSI_HOST_Type *base,
                                         dsi_smartdma_handle_t *handle,
                                         dsi_smartdma_write_mem_transfer_t *xfer);

/*!
 * @brief Abort current APB data transfer.
 *
 * @param base MIPI DSI host peripheral base address.
 * @param handle pointer to dsi_smartdma_handle_t structure which stores the transfer state.
 */
void DSI_TransferAbortSMARTDMA(MIPI_DSI_HOST_Type *base, dsi_smartdma_handle_t *handle);

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* _FSL_MIPI_DSI_SMARTDMA_H_ */
