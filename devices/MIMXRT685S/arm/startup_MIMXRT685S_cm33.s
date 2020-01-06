;/*****************************************************************************
; * @file:    startup_MIMXRT685S_cm33.s
; * @purpose: CMSIS Cortex-M33 Core Device Startup File for the
; *           MIMXRT685S_cm33
; * @version: 1.0
; * @date:    2018-6-19
; *
; * Copyright 1997-2016 Freescale Semiconductor, Inc.
; * Copyright 2016-2019 NXP
; * All rights reserved.
; *
; * SPDX-License-Identifier: BSD-3-Clause
; *
; *------- <<< Use Configuration Wizard in Context Menu >>> ------------------
; *
; *****************************************************************************/


                PRESERVE8
                THUMB

; Vector Table Mapped to Address 0 at Reset
                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Limit|

__Vectors       DCD     |Image$$ARM_LIB_STACK$$ZI$$Limit| ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler

                DCD     NMI_Handler
                DCD     HardFault_Handler
                DCD     MemManage_Handler
                DCD     BusFault_Handler
                DCD     UsageFault_Handler
__vector_table_0x1c
                DCD     SecureFault_Handler
                DCD     0x180000                  ;Image length
                DCD     0
                DCD     0
                DCD     SVC_Handler
                DCD     DebugMon_Handler
                DCD     __Vectors                 ;Image load address
                DCD     PendSV_Handler
                DCD     SysTick_Handler

                ; External Interrupts
                DCD     WDT0_IRQHandler  ; Windowed watchdog timer 0 (CM33 watchdog)
                DCD     DMA0_IRQHandler  ; DMA controller 0 (secure or CM33 DMA)
                DCD     GPIO_INTA_IRQHandler  ; GPIO interrupt A
                DCD     GPIO_INTB_IRQHandler  ; GPIO interrupt B
                DCD     PIN_INT0_IRQHandler  ; Pin interrupt 0 or pattern match engine slice 0 int
                DCD     PIN_INT1_IRQHandler  ; Pin interrupt 1 or pattern match engine slice 1 int
                DCD     PIN_INT2_IRQHandler  ; Pin interrupt 2 or pattern match engine slice 2 int
                DCD     PIN_INT3_IRQHandler  ; Pin interrupt 3 or pattern match engine slice 3 int
                DCD     UTICK0_IRQHandler  ; Micro-tick Timer
                DCD     MRT0_IRQHandler  ; Multi-Rate Timer
                DCD     CTIMER0_IRQHandler  ; Standard counter/timer CTIMER0
                DCD     CTIMER1_IRQHandler  ; Standard counter/timer CTIMER1
                DCD     SCT0_IRQHandler  ; SCTimer/PWM
                DCD     CTIMER3_IRQHandler  ; Standard counter/timer CTIMER3
                DCD     FLEXCOMM0_IRQHandler  ; Flexcomm Interface 0 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM1_IRQHandler  ; Flexcomm Interface 1 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM2_IRQHandler  ; Flexcomm Interface 2 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM3_IRQHandler  ; Flexcomm Interface 3 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM4_IRQHandler  ; Flexcomm Interface 4 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM5_IRQHandler  ; Flexcomm Interface 5 (USART, SPI, I2C, I2S)
                DCD     FLEXCOMM14_IRQHandler  ; Flexcomm Interface 14 (SPI only)
                DCD     FLEXCOMM15_IRQHandler  ; Flexcomm Interface 15 (I2C only)
                DCD     ADC0_IRQHandler  ; ADC0
                DCD     Reserved39_IRQHandler  ; Reserved interrupt
                DCD     ACMP_IRQHandler  ; Analog comparator
                DCD     DMIC0_IRQHandler  ; Digital microphone and DMIC subsystem
                DCD     HW_WAKE_IRQHandler  ; Hardware wake-up
                DCD     HYPERVISOR_IRQHandler  ; Hypervisor
                DCD     SECUREVIOLATION_IRQHandler  ; Secure violation
                DCD     HWVAD0_IRQHandler  ; Hardware Voice Activity Detector
                DCD     Reserved46_IRQHandler  ; Reserved interrupt
                DCD     RNG_IRQHandler  ; Random number Generator
                DCD     RTC_IRQHandler  ; RTC alarm and wake-up
                DCD     DSPWAKE_IRQHandler  ; Wake-up from DSP
                DCD     MU_A_IRQHandler  ; Messaging Unit port A for CM33
                DCD     PIN_INT4_IRQHandler  ; Pin interrupt 4 or pattern match engine slice 4 int
                DCD     PIN_INT5_IRQHandler  ; Pin interrupt 5 or pattern match engine slice 5 int
                DCD     PIN_INT6_IRQHandler  ; Pin interrupt 6 or pattern match engine slice 6 int
                DCD     PIN_INT7_IRQHandler  ; Pin interrupt 7 or pattern match engine slice 7 int
                DCD     CTIMER2_IRQHandler  ; Standard counter/timer CTIMER2
                DCD     CTIMER4_IRQHandler  ; Standard counter/timer CTIMER4
                DCD     OS_EVENT_IRQHandler  ; OS event timer
                DCD     QSPI_IRQHandler  ; Quad/Octal SPI interface
                DCD     Reserved59_IRQHandler  ; Reserved interrupt
                DCD     Reserved60_IRQHandler  ; Reserved interrupt
                DCD     USDHC0_IRQHandler  ; USDHC0 (Enhanced SDHC) interrupt request
                DCD     Reserved62_IRQHandler  ; Reserved interrupt
                DCD     SGPIO_INTA_IRQHandler  ; Secure GPIO interrupt A
                DCD     SGPIO_INTB_IRQHandler  ; Secure GPIO interrupt B
                DCD     I3C_IRQHandler  ; I3C interface 0
                DCD     USB0_IRQHandler  ; High-speed USB device/host
                DCD     USB0_WAKEUP_IRQHandler  ; USB Activity Wake-up Interrupt
                DCD     WDT1_IRQHandler  ; Windowed watchdog timer 1 (HiFi 4 watchdog)
                DCD     USB_PHYDCD_IRQHandler  ; USBPHY DCD
                DCD     DMA1_IRQHandler  ; DMA controller 1 (non-secure or HiFi 4 DMA)
                DCD     PUF_IRQHandler  ; Physical Unclonable Function
                DCD     POWERQUAD_IRQHandler  ; PowerQuad math coprocessor
                DCD     CASPER_IRQHandler  ; Casper cryptographic coprocessor
                DCD     PMC_IRQHandler  ; Power management IC
                DCD     HASHCRYPT_IRQHandler  ; Hash-AES unit

                AREA    |.text|, CODE, READONLY

; Reset Handler
Reset_Handler   PROC
                EXPORT  Reset_Handler               [WEAK]
                IMPORT  SystemInit
                IMPORT  __main
                IMPORT  |Image$$ARM_LIB_STACK$$ZI$$Base|

                CPSID   I               ; Mask interrupts
                LDR     R0, =0xE000ED08
                LDR     R1, =__Vectors
                STR     R1, [R0]
                LDR     R2, [R1]
                MSR     MSP, R2
                LDR     R0, =|Image$$ARM_LIB_STACK$$ZI$$Base|
                MSR     MSPLIM, R0
                LDR     R0, =SystemInit
                BLX     R0
                CPSIE   i               ; Unmask interrupts
                LDR     R0, =__main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)
NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP

HardFault_Handler \
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP

MemManage_Handler     PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP

BusFault_Handler PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP

UsageFault_Handler PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP

SecureFault_Handler PROC
                EXPORT  SecureFault_Handler       [WEAK]
                B       .
                ENDP

SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP

DebugMon_Handler PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP

PendSV_Handler  PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP

SysTick_Handler PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

WDT0_IRQHandler\
                PROC
                EXPORT     WDT0_IRQHandler        [WEAK]
                LDR        R0, =WDT0_DriverIRQHandler
                BX         R0
                ENDP

DMA0_IRQHandler\
                PROC
                EXPORT     DMA0_IRQHandler        [WEAK]
                LDR        R0, =DMA0_DriverIRQHandler
                BX         R0
                ENDP

GPIO_INTA_IRQHandler\
                PROC
                EXPORT     GPIO_INTA_IRQHandler        [WEAK]
                LDR        R0, =GPIO_INTA_DriverIRQHandler
                BX         R0
                ENDP

GPIO_INTB_IRQHandler\
                PROC
                EXPORT     GPIO_INTB_IRQHandler        [WEAK]
                LDR        R0, =GPIO_INTB_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT0_IRQHandler\
                PROC
                EXPORT     PIN_INT0_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT0_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT1_IRQHandler\
                PROC
                EXPORT     PIN_INT1_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT1_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT2_IRQHandler\
                PROC
                EXPORT     PIN_INT2_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT2_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT3_IRQHandler\
                PROC
                EXPORT     PIN_INT3_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT3_DriverIRQHandler
                BX         R0
                ENDP

UTICK0_IRQHandler\
                PROC
                EXPORT     UTICK0_IRQHandler        [WEAK]
                LDR        R0, =UTICK0_DriverIRQHandler
                BX         R0
                ENDP

MRT0_IRQHandler\
                PROC
                EXPORT     MRT0_IRQHandler        [WEAK]
                LDR        R0, =MRT0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER0_IRQHandler\
                PROC
                EXPORT     CTIMER0_IRQHandler        [WEAK]
                LDR        R0, =CTIMER0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER1_IRQHandler\
                PROC
                EXPORT     CTIMER1_IRQHandler        [WEAK]
                LDR        R0, =CTIMER1_DriverIRQHandler
                BX         R0
                ENDP

SCT0_IRQHandler\
                PROC
                EXPORT     SCT0_IRQHandler        [WEAK]
                LDR        R0, =SCT0_DriverIRQHandler
                BX         R0
                ENDP

CTIMER3_IRQHandler\
                PROC
                EXPORT     CTIMER3_IRQHandler        [WEAK]
                LDR        R0, =CTIMER3_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM0_IRQHandler\
                PROC
                EXPORT     FLEXCOMM0_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM0_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM1_IRQHandler\
                PROC
                EXPORT     FLEXCOMM1_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM1_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM2_IRQHandler\
                PROC
                EXPORT     FLEXCOMM2_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM2_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM3_IRQHandler\
                PROC
                EXPORT     FLEXCOMM3_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM3_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM4_IRQHandler\
                PROC
                EXPORT     FLEXCOMM4_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM4_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM5_IRQHandler\
                PROC
                EXPORT     FLEXCOMM5_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM5_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM14_IRQHandler\
                PROC
                EXPORT     FLEXCOMM14_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM14_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM15_IRQHandler\
                PROC
                EXPORT     FLEXCOMM15_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM15_DriverIRQHandler
                BX         R0
                ENDP

ADC0_IRQHandler\
                PROC
                EXPORT     ADC0_IRQHandler        [WEAK]
                LDR        R0, =ADC0_DriverIRQHandler
                BX         R0
                ENDP

Reserved39_IRQHandler\
                PROC
                EXPORT     Reserved39_IRQHandler        [WEAK]
                LDR        R0, =Reserved39_DriverIRQHandler
                BX         R0
                ENDP

ACMP_IRQHandler\
                PROC
                EXPORT     ACMP_IRQHandler        [WEAK]
                LDR        R0, =ACMP_DriverIRQHandler
                BX         R0
                ENDP

DMIC0_IRQHandler\
                PROC
                EXPORT     DMIC0_IRQHandler        [WEAK]
                LDR        R0, =DMIC0_DriverIRQHandler
                BX         R0
                ENDP

HW_WAKE_IRQHandler\
                PROC
                EXPORT     HW_WAKE_IRQHandler        [WEAK]
                LDR        R0, =HW_WAKE_DriverIRQHandler
                BX         R0
                ENDP

HYPERVISOR_IRQHandler\
                PROC
                EXPORT     HYPERVISOR_IRQHandler        [WEAK]
                LDR        R0, =HYPERVISOR_DriverIRQHandler
                BX         R0
                ENDP

SECUREVIOLATION_IRQHandler\
                PROC
                EXPORT     SECUREVIOLATION_IRQHandler        [WEAK]
                LDR        R0, =SECUREVIOLATION_DriverIRQHandler
                BX         R0
                ENDP

HWVAD0_IRQHandler\
                PROC
                EXPORT     HWVAD0_IRQHandler        [WEAK]
                LDR        R0, =HWVAD0_DriverIRQHandler
                BX         R0
                ENDP

Reserved46_IRQHandler\
                PROC
                EXPORT     Reserved46_IRQHandler        [WEAK]
                LDR        R0, =Reserved46_DriverIRQHandler
                BX         R0
                ENDP

RNG_IRQHandler\
                PROC
                EXPORT     RNG_IRQHandler        [WEAK]
                LDR        R0, =RNG_DriverIRQHandler
                BX         R0
                ENDP

RTC_IRQHandler\
                PROC
                EXPORT     RTC_IRQHandler        [WEAK]
                LDR        R0, =RTC_DriverIRQHandler
                BX         R0
                ENDP

DSPWAKE_IRQHandler\
                PROC
                EXPORT     DSPWAKE_IRQHandler        [WEAK]
                LDR        R0, =DSPWAKE_DriverIRQHandler
                BX         R0
                ENDP

MU_A_IRQHandler\
                PROC
                EXPORT     MU_A_IRQHandler        [WEAK]
                LDR        R0, =MU_A_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT4_IRQHandler\
                PROC
                EXPORT     PIN_INT4_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT4_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT5_IRQHandler\
                PROC
                EXPORT     PIN_INT5_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT5_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT6_IRQHandler\
                PROC
                EXPORT     PIN_INT6_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT6_DriverIRQHandler
                BX         R0
                ENDP

PIN_INT7_IRQHandler\
                PROC
                EXPORT     PIN_INT7_IRQHandler        [WEAK]
                LDR        R0, =PIN_INT7_DriverIRQHandler
                BX         R0
                ENDP

CTIMER2_IRQHandler\
                PROC
                EXPORT     CTIMER2_IRQHandler        [WEAK]
                LDR        R0, =CTIMER2_DriverIRQHandler
                BX         R0
                ENDP

CTIMER4_IRQHandler\
                PROC
                EXPORT     CTIMER4_IRQHandler        [WEAK]
                LDR        R0, =CTIMER4_DriverIRQHandler
                BX         R0
                ENDP

OS_EVENT_IRQHandler\
                PROC
                EXPORT     OS_EVENT_IRQHandler        [WEAK]
                LDR        R0, =OS_EVENT_DriverIRQHandler
                BX         R0
                ENDP

QSPI_IRQHandler\
                PROC
                EXPORT     QSPI_IRQHandler        [WEAK]
                LDR        R0, =QSPI_DriverIRQHandler
                BX         R0
                ENDP

Reserved59_IRQHandler\
                PROC
                EXPORT     Reserved59_IRQHandler        [WEAK]
                LDR        R0, =Reserved59_DriverIRQHandler
                BX         R0
                ENDP

Reserved60_IRQHandler\
                PROC
                EXPORT     Reserved60_IRQHandler        [WEAK]
                LDR        R0, =Reserved60_DriverIRQHandler
                BX         R0
                ENDP

USDHC0_IRQHandler\
                PROC
                EXPORT     USDHC0_IRQHandler        [WEAK]
                LDR        R0, =USDHC0_DriverIRQHandler
                BX         R0
                ENDP

Reserved62_IRQHandler\
                PROC
                EXPORT     Reserved62_IRQHandler        [WEAK]
                LDR        R0, =Reserved62_DriverIRQHandler
                BX         R0
                ENDP

SGPIO_INTA_IRQHandler\
                PROC
                EXPORT     SGPIO_INTA_IRQHandler        [WEAK]
                LDR        R0, =SGPIO_INTA_DriverIRQHandler
                BX         R0
                ENDP

SGPIO_INTB_IRQHandler\
                PROC
                EXPORT     SGPIO_INTB_IRQHandler        [WEAK]
                LDR        R0, =SGPIO_INTB_DriverIRQHandler
                BX         R0
                ENDP

I3C_IRQHandler\
                PROC
                EXPORT     I3C_IRQHandler        [WEAK]
                LDR        R0, =I3C_DriverIRQHandler
                BX         R0
                ENDP

USB0_IRQHandler\
                PROC
                EXPORT     USB0_IRQHandler        [WEAK]
                LDR        R0, =USB0_DriverIRQHandler
                BX         R0
                ENDP

USB0_WAKEUP_IRQHandler\
                PROC
                EXPORT     USB0_WAKEUP_IRQHandler        [WEAK]
                LDR        R0, =USB0_WAKEUP_DriverIRQHandler
                BX         R0
                ENDP

WDT1_IRQHandler\
                PROC
                EXPORT     WDT1_IRQHandler        [WEAK]
                LDR        R0, =WDT1_DriverIRQHandler
                BX         R0
                ENDP

USB_PHYDCD_IRQHandler\
                PROC
                EXPORT     USB_PHYDCD_IRQHandler        [WEAK]
                LDR        R0, =USB_PHYDCD_DriverIRQHandler
                BX         R0
                ENDP

DMA1_IRQHandler\
                PROC
                EXPORT     DMA1_IRQHandler        [WEAK]
                LDR        R0, =DMA1_DriverIRQHandler
                BX         R0
                ENDP

PUF_IRQHandler\
                PROC
                EXPORT     PUF_IRQHandler        [WEAK]
                LDR        R0, =PUF_DriverIRQHandler
                BX         R0
                ENDP

POWERQUAD_IRQHandler\
                PROC
                EXPORT     POWERQUAD_IRQHandler        [WEAK]
                LDR        R0, =POWERQUAD_DriverIRQHandler
                BX         R0
                ENDP

CASPER_IRQHandler\
                PROC
                EXPORT     CASPER_IRQHandler        [WEAK]
                LDR        R0, =CASPER_DriverIRQHandler
                BX         R0
                ENDP

PMC_IRQHandler\
                PROC
                EXPORT     PMC_IRQHandler        [WEAK]
                LDR        R0, =PMC_DriverIRQHandler
                BX         R0
                ENDP

HASHCRYPT_IRQHandler\
                PROC
                EXPORT     HASHCRYPT_IRQHandler        [WEAK]
                LDR        R0, =HASHCRYPT_DriverIRQHandler
                BX         R0
                ENDP

Default_Handler PROC
                EXPORT     WDT0_DriverIRQHandler        [WEAK]
                EXPORT     DMA0_DriverIRQHandler        [WEAK]
                EXPORT     GPIO_INTA_DriverIRQHandler        [WEAK]
                EXPORT     GPIO_INTB_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT0_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT1_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT2_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT3_DriverIRQHandler        [WEAK]
                EXPORT     UTICK0_DriverIRQHandler        [WEAK]
                EXPORT     MRT0_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER0_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER1_DriverIRQHandler        [WEAK]
                EXPORT     SCT0_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER3_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM0_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM1_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM2_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM3_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM4_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM5_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM14_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM15_DriverIRQHandler        [WEAK]
                EXPORT     ADC0_DriverIRQHandler        [WEAK]
                EXPORT     Reserved39_DriverIRQHandler        [WEAK]
                EXPORT     ACMP_DriverIRQHandler        [WEAK]
                EXPORT     DMIC0_DriverIRQHandler        [WEAK]
                EXPORT     HW_WAKE_DriverIRQHandler        [WEAK]
                EXPORT     HYPERVISOR_DriverIRQHandler        [WEAK]
                EXPORT     SECUREVIOLATION_DriverIRQHandler        [WEAK]
                EXPORT     HWVAD0_DriverIRQHandler        [WEAK]
                EXPORT     Reserved46_DriverIRQHandler        [WEAK]
                EXPORT     RNG_DriverIRQHandler        [WEAK]
                EXPORT     RTC_DriverIRQHandler        [WEAK]
                EXPORT     DSPWAKE_DriverIRQHandler        [WEAK]
                EXPORT     MU_A_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT4_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT5_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT6_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT7_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER2_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER4_DriverIRQHandler        [WEAK]
                EXPORT     OS_EVENT_DriverIRQHandler        [WEAK]
                EXPORT     QSPI_DriverIRQHandler        [WEAK]
                EXPORT     Reserved59_DriverIRQHandler        [WEAK]
                EXPORT     Reserved60_DriverIRQHandler        [WEAK]
                EXPORT     USDHC0_DriverIRQHandler        [WEAK]
                EXPORT     Reserved62_DriverIRQHandler        [WEAK]
                EXPORT     SGPIO_INTA_DriverIRQHandler        [WEAK]
                EXPORT     SGPIO_INTB_DriverIRQHandler        [WEAK]
                EXPORT     I3C_DriverIRQHandler        [WEAK]
                EXPORT     USB0_DriverIRQHandler        [WEAK]
                EXPORT     USB0_WAKEUP_DriverIRQHandler        [WEAK]
                EXPORT     WDT1_DriverIRQHandler        [WEAK]
                EXPORT     USB_PHYDCD_DriverIRQHandler        [WEAK]
                EXPORT     DMA1_DriverIRQHandler        [WEAK]
                EXPORT     PUF_DriverIRQHandler        [WEAK]
                EXPORT     POWERQUAD_DriverIRQHandler        [WEAK]
                EXPORT     CASPER_DriverIRQHandler        [WEAK]
                EXPORT     PMC_DriverIRQHandler        [WEAK]
                EXPORT     HASHCRYPT_DriverIRQHandler        [WEAK]

WDT0_DriverIRQHandler
DMA0_DriverIRQHandler
GPIO_INTA_DriverIRQHandler
GPIO_INTB_DriverIRQHandler
PIN_INT0_DriverIRQHandler
PIN_INT1_DriverIRQHandler
PIN_INT2_DriverIRQHandler
PIN_INT3_DriverIRQHandler
UTICK0_DriverIRQHandler
MRT0_DriverIRQHandler
CTIMER0_DriverIRQHandler
CTIMER1_DriverIRQHandler
SCT0_DriverIRQHandler
CTIMER3_DriverIRQHandler
FLEXCOMM0_DriverIRQHandler
FLEXCOMM1_DriverIRQHandler
FLEXCOMM2_DriverIRQHandler
FLEXCOMM3_DriverIRQHandler
FLEXCOMM4_DriverIRQHandler
FLEXCOMM5_DriverIRQHandler
FLEXCOMM14_DriverIRQHandler
FLEXCOMM15_DriverIRQHandler
ADC0_DriverIRQHandler
Reserved39_DriverIRQHandler
ACMP_DriverIRQHandler
DMIC0_DriverIRQHandler
HW_WAKE_DriverIRQHandler
HYPERVISOR_DriverIRQHandler
SECUREVIOLATION_DriverIRQHandler
HWVAD0_DriverIRQHandler
Reserved46_DriverIRQHandler
RNG_DriverIRQHandler
RTC_DriverIRQHandler
DSPWAKE_DriverIRQHandler
MU_A_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER4_DriverIRQHandler
OS_EVENT_DriverIRQHandler
QSPI_DriverIRQHandler
Reserved59_DriverIRQHandler
Reserved60_DriverIRQHandler
USDHC0_DriverIRQHandler
Reserved62_DriverIRQHandler
SGPIO_INTA_DriverIRQHandler
SGPIO_INTB_DriverIRQHandler
I3C_DriverIRQHandler
USB0_DriverIRQHandler
USB0_WAKEUP_DriverIRQHandler
WDT1_DriverIRQHandler
USB_PHYDCD_DriverIRQHandler
DMA1_DriverIRQHandler
PUF_DriverIRQHandler
POWERQUAD_DriverIRQHandler
CASPER_DriverIRQHandler
PMC_DriverIRQHandler
HASHCRYPT_DriverIRQHandler

                B       .

                ENDP


                ALIGN


                END

