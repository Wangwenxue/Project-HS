/**
 ****************************************************************************************
 *
 * @file intc.h
 *
 * @brief Declaration of the Interrupt Controller API.
 *
 * Copyright (C) RivieraWaves 2009-2012
 *
 * $Rev: 5769 $
 *
 ****************************************************************************************
 */
/**
 ****************************************************************************************
 *
 * @file intc.h
 *
 * @brief NXP revised and cut.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _INTC_H_
#define _INTC_H_

/**
 ****************************************************************************************
 * @addtogroup INTC INTC
 * @ingroup DRIVERS
 *
 * @brief Declaration of the Interrupt Controller API.
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "driver_config.h"

/*
 * DEFINES
 ****************************************************************************************
 */

/** @brief Enable interrupts globally in the system.
 * This macro must be used when the initialization phase is over and the interrupts
 * can start being handled by the system.
 */                                              

/*
#define GLOBAL_INT_START()          \
do{                                    \
    NVIC->ISER[0] = 0xe0000e00;        \
}while(0)    
*/
#define GLOBAL_INT_START() __set_PRIMASK(0)

/** @brief Disable interrupts globally in the system.
 * This macro must be used when the system wants to disable all the interrupt
 * it could handle.
 */
/*
#define GLOBAL_INT_STOP()           \
do{                                    \
    NVIC->ICER[0] = 0xffffffff;        \
}while(0)    
*/
#define GLOBAL_INT_STOP() __set_PRIMASK(1)

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */

/** @brief Disable interrupts globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE macro since this
 * last one will close the brace that the current macro opens.  This means that both
 * macros must be located at the same scope level.
 */

#define GLOBAL_INT_DISABLE()            \
do {                                    \
    uint32_t int_restore;                \
    int_restore = NVIC->ISER[0];        \
    NVIC->ICER[0] = 0xffffffff;                    

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */

#define GLOBAL_INT_RESTORE()            \
    NVIC->ISER[0] = int_restore;        \
} while(0)

/** @brief Disable interrupts without tuner tx&rx interrupt globally in the system.
 * This macro must be used in conjunction with the @ref GLOBAL_INT_RESTORE_WITHOUT_TUNER macro since this
 * last one will close the brace that the current macro opens.  This means that both
 * macros must be located at the same scope level.
 */
#define GLOBAL_INT_DISABLE_WITHOUT_TUNER()  \
do {                                        \
    uint32_t int_restore;                   \
    int_restore = NVIC->ISER[0];            \
    NVIC->ICER[0] = 0x1fffffff;

/** @brief Restore interrupts from the previous global disable.
 * @sa GLOBAL_INT_DISABLE
 */

#define GLOBAL_INT_RESTORE_WITHOUT_TUNER()  \
    NVIC->ISER[0] = int_restore;            \
} while(0)

/** @name Mapping of the peripherals interrupts in the interrupt controller.
 * @{
 */
#define INTC_GPIO           (0)
#define INTC_COMPARATOR0    (1)
#define INTC_COMPARATOR1    (2)
#define INTC_RWBLE          (3)
#define INTC_RTCCAP         (4)
#define INTC_RTCSLPT        (5)
#define INTC_RTC            (6)
#define INTC_ADC            (7)
#define INTC_DMA            (8)
#define INTC_ANT            (9)
#define INTC_UART0TX        (10)
#define INTC_UART0RX        (11)
#define INTC_SPI0TX         (12)
#define INTC_SPI0RX         (13)
#define INTC_UART1TX        (14)
#define INTC_UART1RX        (15)
#define INTC_SPI1TX         (16)
#define INTC_SPI1RX         (17)
#define INTC_I2C            (18)
#define INTC_TIMER0         (19)
#define INTC_TIMER1         (20)
#define INTC_TIMER2         (21)
#define INTC_TIMER3         (22)
#define INTC_WDT            (23)
#define INTC_RVSD24         (24)
#define INTC_RVSD25         (25)
#define INTC_RVSD26         (26)
#define INTC_RVSD27         (27)
#define INTC_RVSD28         (28)
#define INTC_TUNERRX        (29)
#define INTC_TUNERTX        (30)
#define INTC_TUNERSETTING   (31)
/// @} INTC mapping

#define INTC_IRQACK_MASK    (0xFFFFFFFFUL)


// zfq
/******************************************* 

0 = Thread mode
1 = Reserved
2 = NMI
3 = HardFault
4-10 = Reserved
11 = SVCall
12, 13 = Reserved
14 = PendSV
15 = SysTick, if implementeda
16 = IRQ0
.
.
.
n+15 = IRQ(n-1)b

*******************************************/

#define IRQ_N_INDEX_START        16

enum qnble_int_index_enum
{                    
    INTC_GPIO_IDX,          
    INTC_COMPARATOR0_IDX,   
    INTC_COMPARATOR1_IDX,   
    INTC_BLE_IDX,         
    INTC_RTC_CAP_IDX,       
    INTC_RTC_SLPT_IDX,      
    INTC_RTC_IDX,           
    INTC_ADC_IDX,           
    INTC_DMA_IDX,           
    INTC_ANT_IDX,           
    INTC_UART0_TX_IDX,      
    INTC_UART0_RX_IDX,      
    INTC_SPI0_TX_IDX,       
    INTC_SPI0_RX_IDX,       
    INTC_UART1_TX_IDX,      
    INTC_UART1_RX_IDX,      
    INTC_SPI1_TX_IDX,       
    INTC_SPI1_RX_IDX,       
    INTC_I2C_IDX,           
    INTC_TIMER0_IDX,        
    INTC_TIMER1_IDX,        
    INTC_TIMER2_IDX,        
    INTC_TIMER3_IDX,        
    INTC_WDT_IDX,           
    INTC_RSVD24_IDX,     
    INTC_RSVD25_IDX,     
    INTC_RSVD26_IDX,     
    INTC_RSVD27_IDX,     
    INTC_RSVD28_IDX,     
    INTC_TUNER_RX_IDX,      
    INTC_TUNER_TX_IDX,      
    INTC_TUNER_SETTING_IDX, 
    INTC_MAX_IDX,
};  
typedef void (*void_fn)(void);

extern void_fn qnble_intc_isr[INTC_MAX_IDX];
   
__INLINE void intc_irqenableclear_set(uint32_t value)
{
    NVIC->ICER[0] = value;    
}

__INLINE void intc_irqenableset_set(uint32_t value)
{
    NVIC->ISER[0] = value;    
}

__INLINE void intc_set_priority()
{
#if 0
    NVIC_SetPriority(GPIO_IRQn, 0x3);
    NVIC_SetPriority(COMPARATOR0_IRQn, 0x3);
    NVIC_SetPriority(COMPARATOR1_IRQn, 0x3);
    NVIC_SetPriority(BLE_IRQn, 0x3);
    NVIC_SetPriority(RTC_CAP_IRQn, 0x3);
    NVIC_SetPriority(RTC_SLPT_IRQn, 0x3);
    NVIC_SetPriority(RTC_IRQn, 0x3);
    NVIC_SetPriority(ADC_IRQn, 0x3);
    NVIC_SetPriority(DMA_IRQn, 0x3);
    NVIC_SetPriority(ANT_IRQn, 0x3);
    NVIC_SetPriority(UART0_TX_IRQn, 0x3);
    NVIC_SetPriority(UART0_RX_IRQn, 0x3);
    NVIC_SetPriority(SPI0_TX_IRQn, 0x3);
    NVIC_SetPriority(SPI0_RX_IRQn, 0x3);
    NVIC_SetPriority(UART1_TX_IRQn, 0x3);
    NVIC_SetPriority(UART1_RX_IRQn, 0x3);
    NVIC_SetPriority(SPI1_TX_IRQn, 0x3);
    NVIC_SetPriority(SPI1_RX_IRQn, 0x3);
    NVIC_SetPriority(I2C_IRQn, 0x3);
    NVIC_SetPriority(TIMER0_IRQn, 0x3);
    NVIC_SetPriority(TIMER1_IRQn, 0x3);
    NVIC_SetPriority(TIMER2_IRQn, 0x3);
    NVIC_SetPriority(TIMER3_IRQn, 0x3);
    NVIC_SetPriority(WDT_IRQn, 0x3);
    NVIC_SetPriority(RSVD24_IRQn, 0x3);
    NVIC_SetPriority(RSVD25_IRQn, 0x3);
    NVIC_SetPriority(RSVD26_IRQn, 0x3);
    NVIC_SetPriority(RSVD27_IRQn, 0x3);
    NVIC_SetPriority(RSVD28_IRQn, 0x3);
    NVIC_SetPriority(TUNER_RX_IRQn, 0x3);
    NVIC_SetPriority(TUNER_TX_IRQn, 0x3);
    NVIC_SetPriority(TUNER_SETTING_IRQn, 0x3); 
#else
    NVIC->IP[0] = 0xc0c0c0c0;
    NVIC->IP[1] = 0xc0c0c0c0;
    NVIC->IP[2] = 0xc0c0c0c0;
    NVIC->IP[3] = 0xc0c0c0c0;
    NVIC->IP[4] = 0xc0c0c0c0;
    NVIC->IP[5] = 0xc0c0c0c0;
    NVIC->IP[6] = 0xc0c0c0c0;
    NVIC->IP[7] = 0xc0c0c0c0;
#endif
}

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
/// @} INTC

#endif // _INTC_H_
