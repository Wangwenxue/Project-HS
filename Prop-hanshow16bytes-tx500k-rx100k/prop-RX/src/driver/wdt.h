/**
 ****************************************************************************************
 *
 * @file wdt.h
 *
 * @brief Header file of WDT for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _WDT_H_
#define _WDT_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_WDT==TRUE

/**
 ****************************************************************************************
 * @defgroup WDT WDT Driver
 * @ingroup DRIVERS
 * @brief Watchdog timer driver
 *
 *  The purpose of Watchdog Timer (WDT) is to perform a system reset after the software
 *  running into a problem. This prevents system from hanging for an infinite period of time.
 *  The main features of QN9020 WDT are listed as follow:
 *    - 32-bit down counter with a programmable timeout interval
 *    - 32KHz clock(WDOGCLK=PCLK, WDOGCLKEN=32K)
 *    - Interrupt output generation on timeout
 *    - Reset signal generation on timeout if the interrupt from the previous timeout remains unserviced by software
 *    - Lock register to protect registers from being altered by runaway software
 *
 * @{
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/** Watchdog timer work mode */
enum WDT_MODE
{
    WDT_NO_ACTION_MOD = 0,          /*!< Set watchdog timer work at no action mode */
    WDT_INT_MOD       = 1,          /*!< Set watchdog timer work at interrupt mode */
    WDT_RESET_MOD     = 2           /*!< Set watchdog timer work at reset mode */
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Enable WDT module clock
 * @description
 *  This function is used to enable WDT module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void wdt_clock_on(void)
{
    // enable 32K clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_32K_CLK);
}

/**
 ****************************************************************************************
 * @brief   Disable WDT module clock
 * @description
 *  This function is used to disable WDT module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void wdt_clock_off(void)
{
    // disable 32K clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_32K_CLK);
}

/**
 ****************************************************************************************
 * @brief   Reset WDT module
 * @description
 *  This function is used to reset WDT module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void wdt_reset(void)
{
    // Reset WDT module
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_WDOG_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_WDOG_RST);
}

/**
 ****************************************************************************************
 * @brief Unlock watchdog timer access
 *
 ****************************************************************************************
 */
__STATIC_INLINE void wdt_unlock(void)
{
    wdt_wdt_SetLKR(QN_WDT, 0x1ACCE551); 
}

/**
 ****************************************************************************************
 * @brief Lock watchdog timer access
 *
 ****************************************************************************************
 */
__STATIC_INLINE void wdt_lock(void)
{
    wdt_wdt_SetLKR(QN_WDT, 0); 
}

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if CONFIG_WDT_DEFAULT_IRQHANDLER==TRUE
void WDT_IRQHandler(void);
#endif

extern void wdt_init(unsigned int cycle, enum WDT_MODE mode);
extern void wdt_set(unsigned int cycle);


/// @} WDT
#endif /* CONFIG_ENABLE_DRIVER_WDT==TRUE */
#endif /* _WDT_H_ */
