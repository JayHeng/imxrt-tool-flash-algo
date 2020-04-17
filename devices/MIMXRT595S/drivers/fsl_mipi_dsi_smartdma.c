/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_mipi_dsi_smartdma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Component ID definition, used by tools. */
#ifndef FSL_COMPONENT_ID
#define FSL_COMPONENT_ID "platform.drivers.mipi_dsi_smartdma"
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*!
 * @brief Callback when SMARTDMA done.
 *
 * @param param Callback parameter passed to SMARTDMA.
 */
static void DSI_SMARTDMA_Callback(void *param);

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * brief Create the MIPI DSI handle.
 *
 * This function initializes the MIPI DSI handle which can be used for other transactional APIs.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle Handle pointer.
 * param callback Callback function.
 * param userData User data.
 */
status_t DSI_TransferCreateHandleSMARTDMA(MIPI_DSI_HOST_Type *base,
                                          dsi_smartdma_handle_t *handle,
                                          dsi_smartdma_callback_t callback,
                                          void *userData)
{
    assert(handle);

    /* Zero the handle */
    memset(handle, 0, sizeof(*handle));

    /* Initialize the handle */
    handle->dsi      = base;
    handle->callback = callback;
    handle->userData = userData;
    handle->isBusy   = false;

    SMARTDMA_InstallFirmware(SMARTDMA_FLEXIO_MCULCD_MEM_ADDR, s_smartdmaFlexioMcuLcdFirmware,
                             SMARTDMA_FLEXIO_MCULCD_FIRMWARE_SIZE);

    SMARTDMA_InstallCallback(DSI_SMARTDMA_Callback, handle);

    return kStatus_Success;
}

/*!
 * brief Abort current APB data transfer.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle pointer to dsi_smartdma_handle_t structure which stores the transfer state.
 */
void DSI_TransferAbortSMARTDMA(MIPI_DSI_HOST_Type *base, dsi_smartdma_handle_t *handle)
{
    assert(handle);

    if (handle->isBusy)
    {
        SMARTDMA_Reset();
        /* Reset the state to idle. */
        handle->isBusy = false;
    }
}

/*!
 * brief Write display controller video memory using SMARTDMA.
 *
 * Perform data transfer using SMARTDMA, when transfer finished,
 * upper layer could be informed through callback function.
 *
 * param base MIPI DSI host peripheral base address.
 * param handle pointer to dsi_smartdma_handle_t structure which stores the transfer state.
 * param xfer Pointer to the transfer structure.
 *
 * retval kStatus_Success Data transfer started successfully.
 * retval kStatus_DSI_Busy Failed to start transfer because DSI is busy with pervious transfer.
 * retval kStatus_DSI_NotSupported Transfer format not supported.
 */
status_t DSI_TransferWriteMemorySMARTDMA(MIPI_DSI_HOST_Type *base,
                                         dsi_smartdma_handle_t *handle,
                                         dsi_smartdma_write_mem_transfer_t *xfer)
{
    assert(handle);

    status_t status;
    uint32_t smartdmaApi;

    if (handle->isBusy)
    {
        status = kStatus_DSI_Busy;
    }
    else
    {
        if (((xfer->inputFormat == kDSI_SMARTDMA_InputPixelFormatRGB565) &&
             (xfer->outputFormat == kDSI_SMARTDMA_OutputPixelFormatRGB565)) ||
            ((xfer->inputFormat == kDSI_SMARTDMA_InputPixelFormatRGB888) &&
             (xfer->outputFormat == kDSI_SMARTDMA_OutputPixelFormatRGB888)))
        {
            if (xfer->inputFormat == kDSI_SMARTDMA_InputPixelFormatRGB565)
            {
                smartdmaApi = kSMARTDMA_MIPI_RGB565_DMA;
            }
            else
            {
                smartdmaApi = kSMARTDMA_MIPI_RGB888_DMA;
            }

            handle->param.p_buffer       = (uint32_t *)xfer->data;
            handle->param.buffersize     = xfer->dataSize;
            handle->param.smartdma_stack = handle->smartdmaStack;

            DSI_EnableInterrupts(base, kDSI_InterruptGroup1ApbTxDone | kDSI_InterruptGroup1HtxTo, 0U);
            SMARTDMA_Reset();
            SMARTDMA_Boot(smartdmaApi, &handle->param, 0);

            status = kStatus_Success;
        }
        else
        {
            status = kStatus_DSI_NotSupported;
        }
    }

    return status;
}

/*!
 * brief Callback when SMARTDMA done.
 *
 * param param Callback parameter passed to SMARTDMA.
 */
static void DSI_SMARTDMA_Callback(void *param)
{
    dsi_smartdma_handle_t *handle = (dsi_smartdma_handle_t *)param;

    uint32_t intFlags1, intFlags2;

    DSI_DisableInterrupts(handle->dsi, kDSI_InterruptGroup1ApbTxDone | kDSI_InterruptGroup1HtxTo, 0U);

    DSI_GetAndClearInterruptStatus(handle->dsi, &intFlags1, &intFlags2);

    handle->isBusy = false;

    if (handle->callback)
    {
        handle->callback(handle->dsi, handle, kStatus_Success, handle->userData);
    }
}
