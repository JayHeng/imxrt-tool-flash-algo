;/*****************************************************************************
; * @file:    startup_MIMXRT595S_cm33.s
; * @purpose: CMSIS Cortex-M33 Core Device Startup File for the
; *           MIMXRT595S_cm33
; * @version: 2.0
; * @date:    2019-7-22
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
                DCD     0x280000                  ;Image length
                DCD     0
                DCD     0
                DCD     SVC_Handler
                DCD     DebugMon_Handler
                DCD     __Vectors                 ;Image load address
                DCD     PendSV_Handler
                DCD     SysTick_Handler

                ; External Interrupts
                DCD     WDT0_IRQHandler  ; Watchdog timer interrupt
                DCD     DMA0_IRQHandler  ; DMA interrupt
                DCD     GPIO_INTA_IRQHandler  ; GPIO Interrupt A
                DCD     GPIO_INTB_IRQHandler  ; GPIO Interrupt B
                DCD     PIN_INT0_IRQHandler  ; General Purpose Input/Output interrupt 0
                DCD     PIN_INT1_IRQHandler  ; General Purpose Input/Output interrupt 1
                DCD     PIN_INT2_IRQHandler  ; General Purpose Input/Output interrupt 2
                DCD     PIN_INT3_IRQHandler  ; General Purpose Input/Output interrupt 3
                DCD     UTICK0_IRQHandler  ; Micro-tick Timer
                DCD     MRT0_IRQHandler  ; Multi-Rate Timer
                DCD     CTIMER0_IRQHandler  ; Standard counter/timer CTIMER0
                DCD     CTIMER1_IRQHandler  ; Standard counter/timer CTIMER1
                DCD     SCT0_IRQHandler  ; SCTimer/PWM
                DCD     CTIMER3_IRQHandler  ; Standard counter/timer CTIMER3
                DCD     FLEXCOMM0_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM1_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM2_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM3_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM4_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM5_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM14_IRQHandler  ; FlexComm interrupt. Standalone SPI
                DCD     FLEXCOMM15_IRQHandler  ; FlexComm interrupt. Standalone I2C
                DCD     ADC0_IRQHandler  ; Analog-to-Digital Converter interrupt
                DCD     MRT1_IRQHandler  ; Multirate Timer interrupt
                DCD     ACMP_IRQHandler  ; Analog comparator Interrupts
                DCD     DMIC0_IRQHandler  ; Digital Microphone Interface interrupt
                DCD     Reserved42_IRQHandler  ; Reserved interrupt
                DCD     HYPERVISOR_IRQHandler  ; Hypervisor interrupt
                DCD     SECURE_VIOLATION_IRQHandler  ; Secure violation interrupt
                DCD     HWVAD0_IRQHandler  ; Hardware Voice Activity Detector interrupt
                DCD     Reserved46_IRQHandler  ; Reserved interrupt
                DCD     RNG_IRQHandler  ; Random Number Generator interrupt
                DCD     RTC_IRQHandler  ; Real Time Clock Alarm interrupt OR Wakeup timer interrupt
                DCD     DSP_TIE_EXPSTATE1_IRQHandler  ; DSP interrupt
                DCD     MU_A_IRQHandler  ; Messaging Unit - Side A
                DCD     PIN_INT4_IRQHandler  ; General Purpose Input/Output interrupt 4
                DCD     PIN_INT5_IRQHandler  ; General Purpose Input/Output interrupt 5
                DCD     PIN_INT6_IRQHandler  ; General Purpose Input/Output interrupt 6
                DCD     PIN_INT7_IRQHandler  ; General Purpose Input/Output interrupt 7
                DCD     CTIMER2_IRQHandler  ; Standard counter/timer CTIMER2
                DCD     CTIMER4_IRQHandler  ; Standard counter/timer CTIMER4
                DCD     OS_EVENT_IRQHandler  ; Event timer M33 Wakeup/interrupt
                DCD     FLEXSPI0_FLEXSPI1_IRQHandler  ; FlexSPI0_IRQ OR FlexSPI1_IRQ
                DCD     FLEXCOMM6_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM7_IRQHandler  ; FlexComm interrupt
                DCD     USDHC0_IRQHandler  ; USDHC interrupt
                DCD     USDHC1_IRQHandler  ; USDHC interrupt
                DCD     SGPIO_INTA_IRQHandler  ; Secure GPIO HS interrupt 0
                DCD     SGPIO_INTB_IRQHandler  ; Secure GPIO HS interrupt 1
                DCD     I3C0_IRQHandler  ; Improved Inter Integrated Circuit 0 interrupt
                DCD     USB0_IRQHandler  ; USB device
                DCD     USB0_NEEDCLK_IRQHandler  ; USB Activity Wake-up Interrupt
                DCD     WDT1_IRQHandler  ; Watchdog timer 1 interrupt
                DCD     USB_PHYDCD_IRQHandler  ; USBPHY DCD interrupt
                DCD     DMA1_IRQHandler  ; DMA interrupt
                DCD     PUF_IRQHandler  ; QuidKey interrupt
                DCD     POWERQUAD_IRQHandler  ; Powerquad interrupt
                DCD     CASPER_IRQHandler  ; Caspar interrupt
                DCD     PMU_PMIC_IRQHandler  ; Power Management Control interrupt
                DCD     HASHCRYPT_IRQHandler  ; SHA interrupt
                DCD     FLEXCOMM8_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM9_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM10_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM11_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM12_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM13_IRQHandler  ; FlexComm interrupt
                DCD     FLEXCOMM16_IRQHandler  ; FlexComm interrupt
                DCD     I3C1_IRQHandler  ; Improved Inter Integrated Circuit 1 interrupt
                DCD     FLEXIO_IRQHandler  ; Flexible I/O interrupt
                DCD     LCDIF_IRQHandler  ; Liquid Crystal Display interface interrupt
                DCD     GPU_IRQHandler  ; Graphics Processor Unit interrupt
                DCD     MIPI_IRQHandler  ; MIPI interrupt
                DCD     Reserved88_IRQHandler  ;
                DCD     SDMA_IRQHandler  ; Smart DMA Engine Controller interrupt

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

MRT1_IRQHandler\
                PROC
                EXPORT     MRT1_IRQHandler        [WEAK]
                LDR        R0, =MRT1_DriverIRQHandler
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

Reserved42_IRQHandler\
                PROC
                EXPORT     Reserved42_IRQHandler        [WEAK]
                LDR        R0, =Reserved42_DriverIRQHandler
                BX         R0
                ENDP

HYPERVISOR_IRQHandler\
                PROC
                EXPORT     HYPERVISOR_IRQHandler        [WEAK]
                LDR        R0, =HYPERVISOR_DriverIRQHandler
                BX         R0
                ENDP

SECURE_VIOLATION_IRQHandler\
                PROC
                EXPORT     SECURE_VIOLATION_IRQHandler        [WEAK]
                LDR        R0, =SECURE_VIOLATION_DriverIRQHandler
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

DSP_TIE_EXPSTATE1_IRQHandler\
                PROC
                EXPORT     DSP_TIE_EXPSTATE1_IRQHandler        [WEAK]
                LDR        R0, =DSP_TIE_EXPSTATE1_DriverIRQHandler
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

FLEXSPI0_FLEXSPI1_IRQHandler\
                PROC
                EXPORT     FLEXSPI0_FLEXSPI1_IRQHandler        [WEAK]
                LDR        R0, =FLEXSPI0_FLEXSPI1_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM6_IRQHandler\
                PROC
                EXPORT     FLEXCOMM6_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM6_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM7_IRQHandler\
                PROC
                EXPORT     FLEXCOMM7_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM7_DriverIRQHandler
                BX         R0
                ENDP

USDHC0_IRQHandler\
                PROC
                EXPORT     USDHC0_IRQHandler        [WEAK]
                LDR        R0, =USDHC0_DriverIRQHandler
                BX         R0
                ENDP

USDHC1_IRQHandler\
                PROC
                EXPORT     USDHC1_IRQHandler        [WEAK]
                LDR        R0, =USDHC1_DriverIRQHandler
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

I3C0_IRQHandler\
                PROC
                EXPORT     I3C0_IRQHandler        [WEAK]
                LDR        R0, =I3C0_DriverIRQHandler
                BX         R0
                ENDP

USB0_IRQHandler\
                PROC
                EXPORT     USB0_IRQHandler        [WEAK]
                LDR        R0, =USB0_DriverIRQHandler
                BX         R0
                ENDP

USB0_NEEDCLK_IRQHandler\
                PROC
                EXPORT     USB0_NEEDCLK_IRQHandler        [WEAK]
                LDR        R0, =USB0_NEEDCLK_DriverIRQHandler
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

PMU_PMIC_IRQHandler\
                PROC
                EXPORT     PMU_PMIC_IRQHandler        [WEAK]
                LDR        R0, =PMU_PMIC_DriverIRQHandler
                BX         R0
                ENDP

HASHCRYPT_IRQHandler\
                PROC
                EXPORT     HASHCRYPT_IRQHandler        [WEAK]
                LDR        R0, =HASHCRYPT_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM8_IRQHandler\
                PROC
                EXPORT     FLEXCOMM8_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM8_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM9_IRQHandler\
                PROC
                EXPORT     FLEXCOMM9_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM9_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM10_IRQHandler\
                PROC
                EXPORT     FLEXCOMM10_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM10_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM11_IRQHandler\
                PROC
                EXPORT     FLEXCOMM11_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM11_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM12_IRQHandler\
                PROC
                EXPORT     FLEXCOMM12_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM12_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM13_IRQHandler\
                PROC
                EXPORT     FLEXCOMM13_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM13_DriverIRQHandler
                BX         R0
                ENDP

FLEXCOMM16_IRQHandler\
                PROC
                EXPORT     FLEXCOMM16_IRQHandler        [WEAK]
                LDR        R0, =FLEXCOMM16_DriverIRQHandler
                BX         R0
                ENDP

I3C1_IRQHandler\
                PROC
                EXPORT     I3C1_IRQHandler        [WEAK]
                LDR        R0, =I3C1_DriverIRQHandler
                BX         R0
                ENDP

FLEXIO_IRQHandler\
                PROC
                EXPORT     FLEXIO_IRQHandler        [WEAK]
                LDR        R0, =FLEXIO_DriverIRQHandler
                BX         R0
                ENDP

LCDIF_IRQHandler\
                PROC
                EXPORT     LCDIF_IRQHandler        [WEAK]
                LDR        R0, =LCDIF_DriverIRQHandler
                BX         R0
                ENDP

GPU_IRQHandler\
                PROC
                EXPORT     GPU_IRQHandler        [WEAK]
                LDR        R0, =GPU_DriverIRQHandler
                BX         R0
                ENDP

MIPI_IRQHandler\
                PROC
                EXPORT     MIPI_IRQHandler        [WEAK]
                LDR        R0, =MIPI_DriverIRQHandler
                BX         R0
                ENDP

Reserved88_IRQHandler\
                PROC
                EXPORT     Reserved88_IRQHandler        [WEAK]
                LDR        R0, =Reserved88_DriverIRQHandler
                BX         R0
                ENDP

SDMA_IRQHandler\
                PROC
                EXPORT     SDMA_IRQHandler        [WEAK]
                LDR        R0, =SDMA_DriverIRQHandler
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
                EXPORT     MRT1_DriverIRQHandler        [WEAK]
                EXPORT     ACMP_DriverIRQHandler        [WEAK]
                EXPORT     DMIC0_DriverIRQHandler        [WEAK]
                EXPORT     Reserved42_DriverIRQHandler        [WEAK]
                EXPORT     HYPERVISOR_DriverIRQHandler        [WEAK]
                EXPORT     SECURE_VIOLATION_DriverIRQHandler        [WEAK]
                EXPORT     HWVAD0_DriverIRQHandler        [WEAK]
                EXPORT     Reserved46_DriverIRQHandler        [WEAK]
                EXPORT     RNG_DriverIRQHandler        [WEAK]
                EXPORT     RTC_DriverIRQHandler        [WEAK]
                EXPORT     DSP_TIE_EXPSTATE1_DriverIRQHandler        [WEAK]
                EXPORT     MU_A_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT4_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT5_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT6_DriverIRQHandler        [WEAK]
                EXPORT     PIN_INT7_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER2_DriverIRQHandler        [WEAK]
                EXPORT     CTIMER4_DriverIRQHandler        [WEAK]
                EXPORT     OS_EVENT_DriverIRQHandler        [WEAK]
                EXPORT     FLEXSPI0_FLEXSPI1_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM6_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM7_DriverIRQHandler        [WEAK]
                EXPORT     USDHC0_DriverIRQHandler        [WEAK]
                EXPORT     USDHC1_DriverIRQHandler        [WEAK]
                EXPORT     SGPIO_INTA_DriverIRQHandler        [WEAK]
                EXPORT     SGPIO_INTB_DriverIRQHandler        [WEAK]
                EXPORT     I3C0_DriverIRQHandler        [WEAK]
                EXPORT     USB0_DriverIRQHandler        [WEAK]
                EXPORT     USB0_NEEDCLK_DriverIRQHandler        [WEAK]
                EXPORT     WDT1_DriverIRQHandler        [WEAK]
                EXPORT     USB_PHYDCD_DriverIRQHandler        [WEAK]
                EXPORT     DMA1_DriverIRQHandler        [WEAK]
                EXPORT     PUF_DriverIRQHandler        [WEAK]
                EXPORT     POWERQUAD_DriverIRQHandler        [WEAK]
                EXPORT     CASPER_DriverIRQHandler        [WEAK]
                EXPORT     PMU_PMIC_DriverIRQHandler        [WEAK]
                EXPORT     HASHCRYPT_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM8_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM9_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM10_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM11_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM12_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM13_DriverIRQHandler        [WEAK]
                EXPORT     FLEXCOMM16_DriverIRQHandler        [WEAK]
                EXPORT     I3C1_DriverIRQHandler        [WEAK]
                EXPORT     FLEXIO_DriverIRQHandler        [WEAK]
                EXPORT     LCDIF_DriverIRQHandler        [WEAK]
                EXPORT     GPU_DriverIRQHandler        [WEAK]
                EXPORT     MIPI_DriverIRQHandler        [WEAK]
                EXPORT     Reserved88_DriverIRQHandler        [WEAK]
                EXPORT     SDMA_DriverIRQHandler        [WEAK]

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
MRT1_DriverIRQHandler
ACMP_DriverIRQHandler
DMIC0_DriverIRQHandler
Reserved42_DriverIRQHandler
HYPERVISOR_DriverIRQHandler
SECURE_VIOLATION_DriverIRQHandler
HWVAD0_DriverIRQHandler
Reserved46_DriverIRQHandler
RNG_DriverIRQHandler
RTC_DriverIRQHandler
DSP_TIE_EXPSTATE1_DriverIRQHandler
MU_A_DriverIRQHandler
PIN_INT4_DriverIRQHandler
PIN_INT5_DriverIRQHandler
PIN_INT6_DriverIRQHandler
PIN_INT7_DriverIRQHandler
CTIMER2_DriverIRQHandler
CTIMER4_DriverIRQHandler
OS_EVENT_DriverIRQHandler
FLEXSPI0_FLEXSPI1_DriverIRQHandler
FLEXCOMM6_DriverIRQHandler
FLEXCOMM7_DriverIRQHandler
USDHC0_DriverIRQHandler
USDHC1_DriverIRQHandler
SGPIO_INTA_DriverIRQHandler
SGPIO_INTB_DriverIRQHandler
I3C0_DriverIRQHandler
USB0_DriverIRQHandler
USB0_NEEDCLK_DriverIRQHandler
WDT1_DriverIRQHandler
USB_PHYDCD_DriverIRQHandler
DMA1_DriverIRQHandler
PUF_DriverIRQHandler
POWERQUAD_DriverIRQHandler
CASPER_DriverIRQHandler
PMU_PMIC_DriverIRQHandler
HASHCRYPT_DriverIRQHandler
FLEXCOMM8_DriverIRQHandler
FLEXCOMM9_DriverIRQHandler
FLEXCOMM10_DriverIRQHandler
FLEXCOMM11_DriverIRQHandler
FLEXCOMM12_DriverIRQHandler
FLEXCOMM13_DriverIRQHandler
FLEXCOMM16_DriverIRQHandler
I3C1_DriverIRQHandler
FLEXIO_DriverIRQHandler
LCDIF_DriverIRQHandler
GPU_DriverIRQHandler
MIPI_DriverIRQHandler
Reserved88_DriverIRQHandler
SDMA_DriverIRQHandler

                B       .

                ENDP


                ALIGN


                END

