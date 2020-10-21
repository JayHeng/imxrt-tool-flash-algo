/*
 * The Clear BSD License
 * Copyright 2018 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the
 * disclaimer below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
 * GRANTED BY THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
 * HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "bl_api.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static bootloader_api_entry_t *g_bootloaderTree;

/*******************************************************************************
 * Codes
 ******************************************************************************/

void bl_api_init(void)
{
    g_bootloaderTree = (bootloader_api_entry_t *)*(uint32_t *)0x0020001c;
}

/*******************************************************************************
 * Clock driver
 ******************************************************************************/
void CLOCK_SetMux(clock_mux_t mux, uint32_t value)
{
    g_bootloaderTree->clockDriver->CLOCK_SetMux(mux, value);
}

uint32_t CLOCK_GetMux(clock_mux_t mux)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetMux(mux);
}

void CLOCK_SetDiv(clock_div_t divider, uint32_t value)
{
    g_bootloaderTree->clockDriver->CLOCK_SetDiv(divider, value);
}

uint32_t CLOCK_GetDiv(clock_div_t divider)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetDiv(divider);
}

void CLOCK_ControlGate(clock_ip_name_t name, clock_gate_value_t value)
{
    g_bootloaderTree->clockDriver->CLOCK_ControlGate(name, value);
}

void CLOCK_EnableClock(clock_ip_name_t name)
{
    g_bootloaderTree->clockDriver->CLOCK_EnableClock(name);
}

void CLOCK_DisableClock(clock_ip_name_t name)
{
    g_bootloaderTree->clockDriver->CLOCK_DisableClock(name);
}

void CLOCK_SetMode(clock_mode_t mode)
{
    g_bootloaderTree->clockDriver->CLOCK_SetMode(mode);
}

void CLOCK_SetPllBypass(CCM_ANALOG_Type *base, clock_pll_t pll, bool bypass)
{
    g_bootloaderTree->clockDriver->CLOCK_SetPllBypass(base, pll, bypass);
}

uint32_t CLOCK_GetCpuClkFreq(void)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetCpuClkFreq();
}

uint32_t CLOCK_GetRtcFreq(void)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetRtcFreq();
}

void CLOCK_SetXtalFreq(uint32_t freq)
{
    g_bootloaderTree->clockDriver->CLOCK_SetXtalFreq(freq);
}

void CLOCK_SetRtcXtalFreq(uint32_t freq)
{
    g_bootloaderTree->clockDriver->CLOCK_SetRtcXtalFreq(freq);
}

void CLOCK_InitExternalClk(bool bypassXtalOsc)
{
    g_bootloaderTree->clockDriver->CLOCK_InitExternalClk(bypassXtalOsc);
}

void CLOCK_DeinitExternalClk(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitExternalClk();
}

void CLOCK_SwitchOsc(clock_osc_t osc)
{
    g_bootloaderTree->clockDriver->CLOCK_SwitchOsc(osc);
}

void CLOCK_InitRcOsc24M(void)
{
    g_bootloaderTree->clockDriver->CLOCK_InitRcOsc24M();
}

void CLOCK_DeinitRcOsc24M(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitRcOsc24M();
}

uint32_t CLOCK_GetFreq(clock_name_t name)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetFreq(name);
}

bool CLOCK_EnableUsbhs0Clock(clock_usb_src_t src, uint32_t freq)
{
    return g_bootloaderTree->clockDriver->CLOCK_EnableUsbhs0Clock(src, freq);
}

bool CLOCK_EnableUsbhs1Clock(clock_usb_src_t src, uint32_t freq)
{
    return g_bootloaderTree->clockDriver->CLOCK_EnableUsbhs1Clock(src, freq);
}

bool CLOCK_EnableUsbhs0PhyPllClock(clock_usb_phy_src_t src, uint32_t freq)
{
    return g_bootloaderTree->clockDriver->CLOCK_EnableUsbhs0PhyPllClock(src, freq);
}

void CLOCK_DisableUsbhs0PhyPllClock(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DisableUsbhs0PhyPllClock();
}

void CLOCK_InitArmPll(const clock_arm_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitArmPll(config);
}

void CLOCK_DeinitArmPll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitArmPll();
}

void CLOCK_InitSysPll(const clock_sys_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitSysPll(config);
}

void CLOCK_DeinitSysPll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitSysPll();
}

void CLOCK_InitUsb1Pll(const clock_usb_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitUsb1Pll(config);
}

void CLOCK_DeinitUsb1Pll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitUsb1Pll();
}

void CLOCK_InitUsb2Pll(const clock_usb_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitUsb2Pll(config);
}

void CLOCK_DeinitUsb2Pll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitUsb2Pll();
}

void CLOCK_InitAudioPll(const clock_audio_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitAudioPll(config);
}

void CLOCK_DeinitAudioPll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitAudioPll();
}

void CLOCK_InitVideoPll(const clock_video_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitVideoPll(config);
}

void CLOCK_DeinitVideoPll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitVideoPll();
}

void CLOCK_InitEnetPll(const clock_enet_pll_config_t *config)
{
    g_bootloaderTree->clockDriver->CLOCK_InitEnetPll(config);
}

void CLOCK_DeinitEnetPll(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitEnetPll();
}

uint32_t CLOCK_GetPllFreq(clock_pll_t pll)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetPllFreq(pll);
}

void CLOCK_InitSysPfd(clock_pfd_t pfd, uint8_t pfdFrac)
{
    g_bootloaderTree->clockDriver->CLOCK_InitSysPfd(pfd, pfdFrac);
}

void CLOCK_DeinitSysPfd(clock_pfd_t pfd)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitSysPfd(pfd);
}

void CLOCK_InitUsb1Pfd(clock_pfd_t pfd, uint8_t pfdFrac)
{
    g_bootloaderTree->clockDriver->CLOCK_InitUsb1Pfd(pfd, pfdFrac);
}

void CLOCK_DeinitUsb1Pfd(clock_pfd_t pfd)
{
    g_bootloaderTree->clockDriver->CLOCK_DeinitUsb1Pfd(pfd);
}

uint32_t CLOCK_GetSysPfdFreq(clock_pfd_t pfd)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetSysPfdFreq(pfd);
}

uint32_t CLOCK_GetUsb1PfdFreq(clock_pfd_t pfd)
{
    return g_bootloaderTree->clockDriver->CLOCK_GetUsb1PfdFreq(pfd);
}

bool CLOCK_EnableUsbhs1PhyPllClock(clock_usb_phy_src_t src, uint32_t freq)
{
    return g_bootloaderTree->clockDriver->CLOCK_EnableUsbhs1PhyPllClock(src, freq);
}

void CLOCK_DisableUsbhs1PhyPllClock(void)
{
    g_bootloaderTree->clockDriver->CLOCK_DisableUsbhs1PhyPllClock();
}

/*******************************************************************************
 * RTWDOG driver
 ******************************************************************************/
void RTWDOG_ClearStatusFlags(RTWDOG_Type *base, uint32_t mask)
{
    g_bootloaderTree->rtwdogDriver->RTWDOG_ClearStatusFlags(base, mask);
}

void RTWDOG_GetDefaultConfig(rtwdog_config_t *config)
{
    g_bootloaderTree->rtwdogDriver->RTWDOG_GetDefaultConfig(config);
}

void RTWDOG_Init(RTWDOG_Type *base, const rtwdog_config_t *config)
{
    g_bootloaderTree->rtwdogDriver->RTWDOG_Init(base, config);
}

void RTWDOG_Deinit(RTWDOG_Type *base)
{
    g_bootloaderTree->rtwdogDriver->RTWDOG_Deinit(base);
}

/*******************************************************************************
 * WDOG driver
 ******************************************************************************/

void WDOG_GetDefaultConfig(wdog_config_t *config)
{
    g_bootloaderTree->wdogDriver->WDOG_GetDefaultConfig(config);
}

void WDOG_Init(WDOG_Type *base, const wdog_config_t *config)
{
    g_bootloaderTree->wdogDriver->WDOG_Init(base, config);
}

void WDOG_Deinit(WDOG_Type *base)
{
    g_bootloaderTree->wdogDriver->WDOG_Deinit(base);
}

uint16_t WDOG_GetStatusFlags(WDOG_Type *base)
{
    return g_bootloaderTree->wdogDriver->WDOG_GetStatusFlags(base);
}

void WDOG_ClearInterruptStatus(WDOG_Type *base, uint16_t mask)
{
    g_bootloaderTree->wdogDriver->WDOG_ClearInterruptStatus(base, mask);
}

void WDOG_Refresh(WDOG_Type *base)
{
    g_bootloaderTree->wdogDriver->WDOG_Refresh(base);
}

/*******************************************************************************
 * FlexSPI NOR driver
 ******************************************************************************/
status_t flexspi_nor_flash_init(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->init(instance, config);
}

status_t flexspi_nor_flash_page_program(uint32_t instance,
                                        flexspi_nor_config_t *config,
                                        uint32_t dstAddr,
                                        const uint32_t *src)
{
    return g_bootloaderTree->flexSpiNorDriver->program(instance, config, dstAddr, src);
}

status_t flexspi_nor_flash_erase_all(uint32_t instance, flexspi_nor_config_t *config)
{
    return g_bootloaderTree->flexSpiNorDriver->erase_all(instance, config);
}

status_t flexspi_nor_get_config(uint32_t instance, flexspi_nor_config_t *config, serial_nor_config_option_t *option)
{
    return g_bootloaderTree->flexSpiNorDriver->get_config(instance, config, option);
}

status_t flexspi_nor_flash_erase(uint32_t instance, flexspi_nor_config_t *config, uint32_t start, uint32_t length)
{
    return g_bootloaderTree->flexSpiNorDriver->erase(instance, config, start, length);
}

status_t flexspi_nor_flash_read(
    uint32_t instance, flexspi_nor_config_t *config, uint32_t *dst, uint32_t start, uint32_t bytes)
{
    return g_bootloaderTree->flexSpiNorDriver->read(instance, config, dst, start, bytes);
}

status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t numberOfSeq)
{
    return g_bootloaderTree->flexSpiNorDriver->update_lut(instance, seqIndex, lutBase, numberOfSeq);
}

status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    return g_bootloaderTree->flexSpiNorDriver->xfer(instance, xfer);
}

void flexspi_clear_cache(uint32_t instance)
{
    g_bootloaderTree->flexSpiNorDriver->clear_cache(instance);
}
