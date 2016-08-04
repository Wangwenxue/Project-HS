;******************************************************************************
;
;  Copyright(C) 2015 NXP Semiconductors N.V.
;  All rights reserved.
;
;******************************************************************************
; * @file     startup.s
; * @brief    CMSIS Cortex-M0 Core Device Startup File for
; *           Device QN9020
; * @version  V3.01
; * @date     06. March 2012
; *
; * @note
; * Copyright (C) 2012 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/

SYS_MODE_REG              EQU     0x40000080
REMAP_BIT                 EQU     0x40000000

; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Stack_Size      EQU     0x00000200
 
                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

Heap_Size       EQU     0x00000000

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base     
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
; ToDo:  Add here the vectors for the device specific external interrupts handler
                DCD     GPIO_IRQHandler           ;  0:  GPIO Event
                DCD     ACMP0_IRQHandler          ;  1:  ACMP0
                DCD     ACMP1_IRQHandler          ;  2:  ACMP1
                DCD     BLE_IRQHandler            ;  3:  BLE
                DCD     RTC_CAP_IRQHandler        ;  4:  RTC capture
                DCD     OSC_EN_IRQHandler         ;  5:  BLE IP OSC_EN output
                DCD     RTC_IRQHandler            ;  6:  RTC
                DCD     ADC_IRQHandler            ;  7:  ADC
                DCD     DMA_IRQHandler            ;  8:  DMA
                DCD     0                         ;  9:  Reserved
                DCD     UART0_TX_IRQHandler       ; 10:  UART0 TX
                DCD     UART0_RX_IRQHandler       ; 11:  UART0 RX
                DCD     SPI0_IRQHandler           ; 12:  SPI0 TX
                DCD     SPI0_IRQHandler           ; 13:  SPI0 RX
                DCD     UART1_TX_IRQHandler       ; 14:  UART1 TX
                DCD     UART1_RX_IRQHandler       ; 15:  UART1 RX
                DCD     SPI1_IRQHandler           ; 16:  SPI1 TX
                DCD     SPI1_IRQHandler           ; 17:  SPI1 RX
                DCD     I2C_IRQHandler            ; 18:  I2C
                DCD     TIMER0_IRQHandler         ; 19:  Timer 0
                DCD     TIMER1_IRQHandler         ; 20:  Timer 1
                DCD     TIMER2_IRQHandler         ; 21:  Timer 2
                DCD     TIMER3_IRQHandler         ; 22:  Timer 3
                DCD     WDT_IRQHandler            ; 23:  Watch Dog
                DCD     PWM0_IRQHandler           ; 24:  PWM CH0
                DCD     PWM1_IRQHandler           ; 25:  PWM CH1
                DCD     CALIB_IRQHandler          ; 26:  Calibration
                DCD     0                         ; 27:  Reserved
                DCD     0                         ; 28:  Reserved
                DCD     TUNER_RX_IRQHandler       ; 29:  RF RX Setting
                DCD     TUNER_TX_IRQHandler       ; 30:  RF TX Setting
                DCD     TUNER_SETTING_IRQHandler  ; 31:  RF Setting
__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                LDR R1,=SYS_MODE_REG
                LDR R0,[R1]
                LDR R2,=REMAP_BIT
                ORRS R0,R0,R2
                STR R0,[R1]

                IMPORT  __main
                LDR     R0, = __main
                BX      R0
                ENDP

; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                EXPORT  HardFault_Handler         [WEAK]
                B       .
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC
; ToDo:  Add here the export definition for the device specific external interrupts handler
                EXPORT  GPIO_IRQHandler           [WEAK]
                EXPORT  ACMP0_IRQHandler          [WEAK]
                EXPORT  ACMP1_IRQHandler          [WEAK]
                EXPORT  BLE_IRQHandler            [WEAK]
                EXPORT  RTC_CAP_IRQHandler        [WEAK]
                EXPORT  OSC_EN_IRQHandler         [WEAK]
                EXPORT  RTC_IRQHandler            [WEAK]
                EXPORT  ADC_IRQHandler            [WEAK]
                EXPORT  DMA_IRQHandler            [WEAK]
                EXPORT  UART0_TX_IRQHandler       [WEAK]
                EXPORT  UART0_RX_IRQHandler       [WEAK]
                EXPORT  SPI0_IRQHandler           [WEAK]
                EXPORT  UART1_TX_IRQHandler       [WEAK]
                EXPORT  UART1_RX_IRQHandler       [WEAK]
                EXPORT  SPI1_IRQHandler           [WEAK]
                EXPORT  I2C_IRQHandler            [WEAK]
                EXPORT  TIMER0_IRQHandler         [WEAK]
                EXPORT  TIMER1_IRQHandler         [WEAK]
                EXPORT  TIMER2_IRQHandler         [WEAK]
                EXPORT  TIMER3_IRQHandler         [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  PWM0_IRQHandler           [WEAK]
                EXPORT  PWM1_IRQHandler           [WEAK]
                EXPORT  CALIB_IRQHandler          [WEAK]
                EXPORT  TUNER_RX_IRQHandler       [WEAK]
                EXPORT  TUNER_TX_IRQHandler       [WEAK]
                EXPORT  TUNER_SETTING_IRQHandler  [WEAK]

; ToDo:  Add here the names for the device specific external interrupts handler
GPIO_IRQHandler
ACMP0_IRQHandler
ACMP1_IRQHandler
BLE_IRQHandler
RTC_CAP_IRQHandler
OSC_EN_IRQHandler
RTC_IRQHandler
ADC_IRQHandler
DMA_IRQHandler
UART0_TX_IRQHandler
UART0_RX_IRQHandler
SPI0_IRQHandler
UART1_TX_IRQHandler
UART1_RX_IRQHandler
SPI1_IRQHandler
I2C_IRQHandler
TIMER0_IRQHandler
TIMER1_IRQHandler
TIMER2_IRQHandler
TIMER3_IRQHandler
WDT_IRQHandler
PWM0_IRQHandler
PWM1_IRQHandler
CALIB_IRQHandler
TUNER_RX_IRQHandler
TUNER_TX_IRQHandler
TUNER_SETTING_IRQHandler

                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF


                END
