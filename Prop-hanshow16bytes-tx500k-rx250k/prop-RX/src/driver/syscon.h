/**
 ****************************************************************************************
 *
 * @file syscon.h
 *
 * @brief System controller driver API.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef SYSCON_H
#define SYSCON_H
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_SYSCON==TRUE

/** 
 ****************************************************************************************
 * @addtogroup SYSTEM_CONTROLLER System Controller Driver
 * @ingroup DRIVERS
 * @brief System Controller Driver
 *
 *  QN9020 System Controller mainly contains Reset Management Unit (RMU), Clock Management Unit (CMU) 
 *  and Power Management Unit (PMU). The following functions are included in these units: 
 *    - System registers management and module functional reset
 *    - Clock generator
 *    - System clock and peripherals clock
 *    - Low Power mode
 *    - PIN MUX
 *
 * @{
 **************************************************************************************** 
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// AHB_CLK = SYS_CLK/(2*(AHB_DIVIDER+1)), n is AHB_CLK; 
#define AHB_CLK_DIV(n)      (g_SystemClock/(2*n) - 1)

/// APB_CLK = AHB_CLK/(2*(APB_DIVIDER+1)), n is APB_CLK; 
#define APB_CLK_DIV(n)      (g_AhbClock/(2*n) - 1)

/// TIMER_CLK = AHB_CLK/(2*(TIMER_DIVIDER+1)), n is TIMER_CLK;
#define TIMER_CLK_DIV(n)    (g_AhbClock/(2*n) - 1)

/// USARTx_CLK = AHB_CLK/(2*(USARTx_DIVIDER+1)), n is USARTx_CLK;
#define USARTx_CLK_DIV(n)   (g_AhbClock/(2*n) - 1)

/// BLE_CLK = AHB_CLK/(2*(BLE_DIVIDER+1)), n is BLE_CLK;
#define BLE_CLK_DIV(n)      (g_AhbClock/(2*n) - 1)


#if (QN_32K_RCO || CONFIG_ENABLE_DRIVER_RTC)
#define CLOCK_32K_CORRECTION_EN                         TRUE        /*!< Enable/Disable 32K clock correction */
#else
#define CLOCK_32K_CORRECTION_EN                         FALSE       /*!< Enable/Disable 32K clock correction */
#endif

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */
 
/// Clock mux
enum CLK_MUX
{
    CLK_XTAL      = 0,          /*!< External High frequency 16MHz or 32MHz     */
    CLK_INT_20M   = 1,          /*!< 20MHz internal high frequency              */
    CLK_INT_32M   = 2,          /*!< 32MHz PLL output                           */
    CLK_LOW_32K   = 3           /*!< 32KHz low speed clock                      */
};

/// Reset cause
enum RESET_CAUSE
{
    NONE_RST      = 0,          /*!< Not reset or reset clear                   */
    POWER_ON_RST  = (1<<0),     /*!< Power-on Reset (POR)                       */
    BROWN_OUT_RST = (1<<1),     /*!< Brown-out Detection (BOD)                  */
    EXT_PIN_RST   = (1<<2),     /*!< RESET pin reset                            */
    WDT_RST       = (1<<3),     /*!< Watchdog reset                             */
    LOCK_UP_RST   = (1<<4),     /*!< ARM M0 Lockup signal output                */
    REBOOT_RST    = (1<<5),     /*!< Software triggered reset for system reboot */
    CPU_SYS_RST   = (1<<6),     /*!< ARM M0 system reset requirement output     */
    CPU_SOFT_RST  = (1<<7)      /*!< CPU Software reset                         */
};

/// Clock type
enum CLK_TYPE
{
    XTAL_16M      = 0,          /*!< External XTAL frequency 16MHz              */
    XTAL_32M      = 1,          /*!< External XTAL frequency 32MHz              */
    PLL_32M       = 2,          /*!< Internal PLL 32MHz                         */
    INT_20M       = 3,          /*!< Internal 20MHz                             */
    RCO_32K       = 4,          /*!< 32KHz clock from RCO32                     */
    XTAL_32K      = 5           /*!< 32KHz clock from XTAL32                    */
};

/// Memory block
enum MEM_BLOCK
{
    MEM_BLOCK0    = 0,          /*!< Memory Block1:  0K ~  8K */
    MEM_BLOCK1    = (1 << 1),   /*!< Memory Block1:  8K ~ 16K */
    MEM_BLOCK2    = (1 << 2),   /*!< Memory Block1: 16K ~ 24K */
    MEM_BLOCK3    = (1 << 3),   /*!< Memory Block1: 24K ~ 32K */
    MEM_BLOCK4    = (1 << 4),   /*!< Memory Block1: 32K ~ 40K */
    MEM_BLOCK5    = (1 << 5),   /*!< Memory Block1: 40K ~ 48K */
    MEM_BLOCK6    = (1 << 6),   /*!< Memory Block1: 48K ~ 56K */
    MEM_BLOCK7    = (1 << 7),   /*!< Memory Block1: 56K ~ 64K */
    MEM_ALL       = (0xFE)      /*!< Memory Block1: 56K ~ 64K */
};

/// XTAL clock source
enum XTAL_CLK_SRC
{
    CRYSTAL       = 0,          /*!< Use crystal oscillator between XTAL1/XTAL2 */
    DIGIT_CLOCK   = 1,          /*!< Digital clock injection to XTAL1           */
    SINGLE_SINE   = 2,          /*!< Single-end sine wave injection to XTAL1    */
    DIFF_SINE     = 3           /*!< Differential sine wave injection to XTAL1/XTAL2 */
};

/**
 ****************************************************************************************
 * @brief  Enable 32K clock 
 * @param[in]    flag       XTAL_32K or RCO_32K
 * @return
 * @description
 *  This function is used to enable 32K clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void clk32k_enable(int flag)
{
    if (flag == RCO_32K) {
        // Enable 32k RCO
        syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RCO, MASK_DISABLE);
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_ENABLE);
    }
    else {
        // Enable 32k XTAL
        syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_XTAL32, MASK_DISABLE);
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_DISABLE);
    }
}

/**
 ****************************************************************************************
 * @brief   Power off memory enable or disable
 * @param[in]    memblk     MEM_BLOCK1 ~ MEM_BLOCK7
 * @param[in]    able       MASK_ENABLE or MASK_DISABLE
 * @return
 * @description
 *  This function is used to control memory power
 *
 *****************************************************************************************
 */
__STATIC_INLINE void memory_power_off(int memblk, int able)
{
    syscon_SetPGCR1WithMask(QN_SYSCON, memblk, able);
}

/**
 ****************************************************************************************
 * @brief   Power off 32K clock enable or disable
 * @param[in]    flag       XTAL_32K or RCO_32K
 * @param[in]    able       MASK_ENABLE or MASK_DISABLE
 * @return
 * @description
 *  This function is used to control 32K clock power
 *
 *****************************************************************************************
 */
__STATIC_INLINE void clk32k_power_off(int flag, int able)
{
    syscon_SetPGCR1WithMask(QN_SYSCON, flag, able);
}

/**
 ****************************************************************************************
 * @brief  set XTAL clock source
 * @param[in]    flag       XTAL_32M or XTAL_32K
 * @param[in]    src        XTAL clock source
 * @return 
 * @description
 *  This function is used to set XTAL clock source.
 *****************************************************************************************
 */ 
__STATIC_INLINE void syscon_set_xtal_src(int flag, int src)
{
    if (flag == XTAL_32M) {
        syscon_SetXtalBuckWithMask(QN_SYSCON, SYSCON_MASK_XTAL_INJ, src<<SYSCON_POS_XTAL_INJ);
    }
    else if (flag == XTAL_32K) {
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32INJ, src<<SYSCON_POS_X32INJ);
    }
}

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern uint32_t g_SystemClock;
extern uint32_t g_AhbClock;
extern uint32_t g_ApbClock;

extern void syscon_set_sysclk_src(enum CLK_MUX clk_src, int flag);
extern void syscon_set_ahb_clk(int clk);
extern void syscon_set_apb_clk(int clk);
extern void syscon_set_timer_clk(int clk);
extern void syscon_set_usart_clk(uint32_t usart, int clk);
extern void syscon_set_ble_clk(int clk);
extern enum RESET_CAUSE syscon_get_reset_cause(void);
extern void syscon_enable_transceiver(uint32_t able);
#if CLOCK_32K_CORRECTION_EN==TRUE
extern void clock_32k_correction_init(void);
extern void clock_32k_correction_enable(void (*callback)(void));
extern void clock_32k_correction_cb(void);
#endif


/// @} SYSTEM_CONTROLLER
#endif /* CONFIG_ENABLE_DRIVER_SYSCON */
#endif /* SYSCON_H */
