/**************************************************
 *
 * Part one of the system initialization code, contains low-level
 * initialization, plain thumb variant.
 *
 * Copyright 2009 IAR Systems. All rights reserved.
 *
 * $Revision: 47021 $
 *
 **************************************************/

;
; The modules in this file are included in the libraries, and may be replaced
; by any user-defined modules that define the PUBLIC symbol _program_start or
; a user defined start symbol.
; To override the cstartup defined in the library, simply add your modified
; version to the workbench project.
;
; The vector table is normally located at address 0.
; When debugging in RAM, it can be located in RAM, aligned to at least 2^6.
; The name "__vector_table" has special meaning for C-SPY:
; it is where the SP start value is found, and the NVIC vector
; table register (VTOR) is initialized to this address if != 0.
;
; Cortex-M version
;

SYS_MODE_REG              EQU     0x40000080
REMAP_BIT                 EQU     0x40000000



        MODULE  ?cstartup

        ;; Forward declaration of sections.
        SECTION CSTACK:DATA:NOROOT(3)

        SECTION .intvec:CODE:NOROOT(2)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table
        PUBLIC  __vector_table_0x1c
        DATA
__vector_table
        DCD     sfe(CSTACK)                 ; Top of Stack
        DCD     Reset_Handler               ; Reset Handler
        DCD     NMI_Handler                 ; NMI Handler
        DCD     HardFault_Handler           ; Hard Fault Handler
        DCD     MemManage_Handler           ; MPU Fault Handler
        DCD     BusFault_Handler            ; Bus Fault Handler
        DCD     UsageFault_Handler          ; Usage Fault Handler
__vector_table_0x1c
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     0                           ; Reserved
        DCD     SVC_Handler                 ; SVCall Handler
        DCD     DebugMon_Handler            ; Debug Monitor Handler
        DCD     0                           ; Reserved
        DCD     PendSV_Handler              ; PendSV Handler
        DCD     SysTick_Handler             ; SysTick Handler

        ; External Interrupts
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



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Default interrupt handlers.
;;

        ;;PUBWEAK Reset_Handler
        SECTION .RESET:CODE:REORDER(2)
	    THUMB
Reset_Handler
        ;; re-map
        LDR       R1, = SYS_MODE_REG
        LDR       R0, [R1]
        LDR       R2, = REMAP_BIT
        ORRS      R0, R0, R2
        STR       R0, [R1]

        LDR     R0, = sfe(CSTACK)
        MSR     MSP, R0

        LDR     R0, = __iar_program_start
        BX      R0

        SECTION .text:CODE:REORDER:NOROOT(2)
        THUMB
        PUBWEAK NMI_Handler
        PUBWEAK HardFault_Handler
        PUBWEAK MemManage_Handler
        PUBWEAK BusFault_Handler
        PUBWEAK UsageFault_Handler
        PUBWEAK SVC_Handler
        PUBWEAK DebugMon_Handler
        PUBWEAK PendSV_Handler
        PUBWEAK SysTick_Handler
        PUBWEAK GPIO_IRQHandler
        PUBWEAK ACMP0_IRQHandler
        PUBWEAK ACMP1_IRQHandler
        PUBWEAK BLE_IRQHandler
        PUBWEAK RTC_CAP_IRQHandler
        PUBWEAK OSC_EN_IRQHandler
        PUBWEAK RTC_IRQHandler
        PUBWEAK ADC_IRQHandler
        PUBWEAK DMA_IRQHandler
        PUBWEAK UART0_TX_IRQHandler
        PUBWEAK UART0_RX_IRQHandler
        PUBWEAK SPI0_IRQHandler
        PUBWEAK UART1_TX_IRQHandler
        PUBWEAK UART1_RX_IRQHandler
        PUBWEAK SPI1_IRQHandler
        PUBWEAK I2C_IRQHandler
        PUBWEAK TIMER0_IRQHandler
        PUBWEAK TIMER1_IRQHandler
        PUBWEAK TIMER2_IRQHandler
        PUBWEAK TIMER3_IRQHandler
        PUBWEAK WDT_IRQHandler
        PUBWEAK PWM0_IRQHandler
        PUBWEAK PWM1_IRQHandler
        PUBWEAK CALIB_IRQHandler
        PUBWEAK TUNER_RX_IRQHandler       ; 29:  RF RX Setting
        PUBWEAK TUNER_TX_IRQHandler       ; 30:  RF TX Setting
        PUBWEAK TUNER_SETTING_IRQHandler

NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
        B .

SVC_Handler
DebugMon_Handler
PendSV_Handler
SysTick_Handler
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
TUNER_RX_IRQHandler       ; 29:  RF RX Setting
TUNER_TX_IRQHandler       ; 30:  RF TX Setting
TUNER_SETTING_IRQHandler

Default_Handler
        B Default_Handler


        END
