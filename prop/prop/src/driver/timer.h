/**
 ****************************************************************************************
 *
 * @file timer.h
 *
 * @brief Header file of TIMER for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _TIMER_H_
#define _TIMER_H_
#include "driver_config.h"
#if (CONFIG_ENABLE_DRIVER_TIMER0==TRUE || CONFIG_ENABLE_DRIVER_TIMER1==TRUE \
    || CONFIG_ENABLE_DRIVER_TIMER2==TRUE || CONFIG_ENABLE_DRIVER_TIMER3==TRUE)
#include "sleep.h"

/**
 ****************************************************************************************
 * @defgroup TIMER Timer Driver
 * @ingroup DRIVERS
 * @brief Timer driver
 *
 *  QN9020 has two 32-bit timers Timer0/1, and two 16-bit timers Timer2/3. All the Timers support
 *  four operation modes, which allow user to easily implement a counting scheme. The Timers can
 *  perform functions like frequency measurement, event counting, interval measurement, clock generation,
 *  delay timing, and so on. The Timers also can generate an interrupt signal upon timeout, or provide the current
 *  value of count during operation, and support external count and capture functions.
 *
 *  The Features of Timer0/1/2/3 are listed as follow:
 *    - 32/16-bit up counter timer with a 10-bit programmable prescaler
 *    - Programmable clock sources, PCLK, 32KHz and external input
 *    - Support four operation modes, which are free running mode, input capture timer mode, input capture event mode input capture counter mode
 *    - Free running timer mode
 *      - Programmable interrupt period by setting the TOP register
 *      - Generate compare interrupt if the interrupt is enabled
 *      - Generate PWM waveform if PWM output  enable (PWM_OE) bit is set
 *      - Programmable PWM period, duty, and PWM polarity
 *    - Input capture timer mode
 *      - Pulse width, duty and period measurement
 *      - Capture on either positive or negative edge or both
 *      - Optional digital noise filtering on capture input
 *      - Programmable interrupt generation
 *    - Input capture event mode
 *      - 16/8-bit event counter
 *    - Input capture counter mode
 *      - 16/8-bit event number register shared with TOP register
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example timer_example.c
 * This is an example of how to use the Timer driver.
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
#define TIMER_PWM_INT_EN               FALSE
#define TIMER_CAP_INT_EN               TRUE

/// TIMER_CLK = AHB_CLK / (2*(TIMER_DIV + 1));
#define TIMER_CLK(x)                   __TIMER_CLK
//#define TIMER_CLK(x)                   g_AhbClock                // timer divider bypass defaultly
//#define TIMER_CLK(x)                   (g_AhbClock / (DIV(x)))   // timer divider not bypass

/// Timer prescaler divider algorithm
#define PSCL_DIV(n)                    ((n) + 1)
// Calculate timer prescaler divider value
#define PSCL_CLK(x, n)                 (TIMER_CLK(x) / (PSCL_DIV(n)))
/// Default timer divider value is 0x01
#define TIMER_DIV                       0x1
/// Set prescaler divider value
//#define TIMER_PSCAL_DIV                 0x0
#define TIMER_PSCAL_DIV                 0x7


/// Set timer counter top value  (TOPR = delayInS * (PSCL_CLK))
#define TIMER_COUNT_S(s, pscl_div)      ((s) * PSCL_CLK(TIMER_DIV, pscl_div))
/// Set timer counter top value  (TOPR = delayInMs * (PSCL_CLK / 1000))
#define TIMER_COUNT_MS(ms, pscl_div)    ((ms) * (PSCL_CLK(TIMER_DIV, pscl_div) / 1000))
/// Set timer counter top value  (TOPR = delayInUs * (PSCL_CLK / 1000000))
#define TIMER_COUNT_US(us, pscl_div)    ((us) * (PSCL_CLK(TIMER_DIV, pscl_div) / 1000000))
// when timer clock working under 1KHz
//#define TIMER_COUNT_MS(ms, pscl_div)    ((ms) * PSCL_CLK(TIMER_DIV, pscl_div) / 1000)
// when timer clock working under 1MHz
//#define TIMER_COUNT_US(us, pscl_div)    ((us) * PSCL_CLK(TIMER_DIV, pscl_div) / 1000000)

/// Timer mode: free running mode, using for normal timer function, PWM(compare counter)
#define FREE_RUNNING_MOD                (0 << TIMER_POS_OMS)
/// Timer mode: input capture timer mode, using for measure period, pulse
#define INCAP_TIMER_MOD                 (1 << TIMER_POS_OMS)
/// Timer mode: input capture event mode, using for measure event number within fix time
#define INCAP_EVENT_MOD                 (2 << TIMER_POS_OMS)
/// Timer mode: input capture counter mode, using for measure time within fix number event
#define INCAP_COUNTER_MOD               (3 << TIMER_POS_OMS)

/// Timer PWM output polarity configuration
//#define TIMER_PWM_POL_CFG               TIMER_MASK_POL
#define TIMER_PWM_POL_CFG               0

/// Timer input capture pin configuration
#define TIMER_INCAP_PIN_CFG             INCAP_PIN0


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/** Timer clock source */
enum TIMER_CSS
{
    CLK_EXT       = (0 << TIMER_POS_CSS),       /*!< Set timer clock source is external clock */
    CLK_ANCMP_OUT = (1 << TIMER_POS_CSS),       /*!< Set timer clock source is analog comparator output */
    CLK_PSCL      = (2 << TIMER_POS_CSS)        /*!< Set timer clock source is timer prescaler */
};

/** Input capture PIN */
enum INCAP_PIN
{
    INCAP_PIN0 = (0 << TIMER_POS_ICPS),         /*!< Set input captrue pin to PIN0 */
    INCAP_PIN1 = (1 << TIMER_POS_ICPS),         /*!< Set input captrue pin to PIN1 */
    INCAP_PIN2 = (2 << TIMER_POS_ICPS),         /*!< Set input captrue pin to PIN2 */
    INCAP_PIN3 = (3 << TIMER_POS_ICPS)          /*!< Set input captrue pin to PIN3 */
};

/** Input capture edge type */
enum INCAP_EDGE
{
    INCAP_EDGE_POS   = (0 << TIMER_POS_ICES),   /*!< Rising edge is set as input capture edge */
    INCAP_EDGE_NEG   = (1 << TIMER_POS_ICES),   /*!< Falling edge is set as input capture edge */
    INCAP_EDGE_BOTH  = (2 << TIMER_POS_ICES)    /*!< Both rising and falling edge are set as input capture edge */
};

/** Input capture source */
enum INCAP_SOURCE
{
    INCAP_SRC_PIN    = 0,                       /*!< Set input capture source is GPIO pin */
    INCAP_SRC_ANCMP  = TIMER_MASK_ICSS          /*!< Set input capture source is analog comparator output */
};

/** PWM output default level */
enum CMP_PWM_POL
{
    CMP_PWM_POL_H = 0,                          /*!< Set timer pwm output default level is high level */
    CMP_PWM_POL_L = TIMER_MASK_POL              /*!< Set timer pwm output default level is low level */
};

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

///TIMER environment parameters
struct timer_env_tag
{
    uint32_t    count;                          /*!< Timer counter value, different working modes have different values */
    void        (*callback)(void);              /*!< The callback of timer interrupt */
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
  
/**
 ****************************************************************************************
 * @brief  Enable or disable timer
 * @param[in]  TIMER           QN_TIMER0,1,2,3
 * @param[in]  able            MASK_ENABLE or MASK_DISABLE
 * @description
 * This function is used to enable or disable the specified Timer.
 *****************************************************************************************
 */
__STATIC_INLINE void timer_enable(QN_TIMER_TypeDef *TIMER, uint32_t able)
{
    uint32_t dev = 0;
    timer_timer_SetCRWithMask(TIMER, TIMER_MASK_TEN, able);

    if(TIMER == QN_TIMER0)
    {
        dev = PM_MASK_TIMER0_ACTIVE_BIT;
    }
    else if(TIMER == QN_TIMER1)
    {
        dev = PM_MASK_TIMER1_ACTIVE_BIT;
    }
    else if(TIMER == QN_TIMER2)
    {
        dev = PM_MASK_TIMER2_ACTIVE_BIT;
    }
    else if(TIMER == QN_TIMER3)
    {
        dev = PM_MASK_TIMER3_ACTIVE_BIT;
    }
        
    if(able)
    {
        dev_prevent_sleep(dev);
    }
    else
    {
        dev_allow_sleep(dev);
    }   
}

/**
 ****************************************************************************************
 * @brief Enable TIMER module clock.
 * @param[in]  TIMER           QN_TIMER0,1,2,3
 * @description
 *  This function is used to enable TIMER module clock
 *****************************************************************************************
 */
__STATIC_INLINE void timer_clock_on(QN_TIMER_TypeDef *TIMER)
{
#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if (TIMER == QN_TIMER0) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER0);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if (TIMER == QN_TIMER1) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER1);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if (TIMER == QN_TIMER2) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER2);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if (TIMER == QN_TIMER3) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER3);
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Disable TIMER module clock
 * @param[in]  TIMER           QN_TIMER0,1,2,3
 * @description
 *  This function is used to disable TIMER module clock
 *****************************************************************************************
 */
__STATIC_INLINE void timer_clock_off(QN_TIMER_TypeDef *TIMER)
{
#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if (TIMER == QN_TIMER0) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER0); 
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if (TIMER == QN_TIMER1) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER1); 
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if (TIMER == QN_TIMER2) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER2); 
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if (TIMER == QN_TIMER3) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER3); 
    }
#endif
}

/**
 ****************************************************************************************
 * @brief   Reset TIMER module
 * @param[in]  TIMER           QN_TIMER0,1,2,3
 * @description
 *  This function is used to reset TIMER module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void timer_reset(QN_TIMER_TypeDef *TIMER)
{
#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if (TIMER == QN_TIMER0) {
        // Reset TIMER0
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_TIMER0_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_TIMER0_RST);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if (TIMER == QN_TIMER1) {
        // Reset TIMER1
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_TIMER1_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_TIMER1_RST);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if (TIMER == QN_TIMER2) {
        // Reset TIMER2
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_TIMER2_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_TIMER2_RST);
    }
#endif
#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if (TIMER == QN_TIMER3) {
        // Reset TIMER3
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_TIMER3_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_TIMER3_RST);
    }
#endif
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/// @cond
#if (CONFIG_ENABLE_DRIVER_TIMER0==TRUE && CONFIG_TIMER0_DEFAULT_IRQHANDLER==TRUE)
extern struct timer_env_tag timer0_env;
void TIMER0_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER1==TRUE && CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE)
extern struct timer_env_tag timer1_env;
void TIMER1_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER2==TRUE && CONFIG_TIMER2_DEFAULT_IRQHANDLER==TRUE)
extern struct timer_env_tag timer2_env;
void TIMER2_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER3==TRUE && CONFIG_TIMER3_DEFAULT_IRQHANDLER==TRUE)
extern struct timer_env_tag timer3_env;
void TIMER3_IRQHandler(void);
#endif
/// @endcond

extern void timer_delay(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t count);
extern void timer_init(QN_TIMER_TypeDef *TIMER, void (*callback)(void));
extern void timer_config(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t count);
extern void timer_pwm_config(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t periodcount, uint32_t pulsecount);
extern void timer_capture_config(QN_TIMER_TypeDef *TIMER, uint32_t cap_mode, uint32_t pscal, uint32_t count, uint32_t event_num);


/// @} TIMER
#endif /* CONFIG_ENABLE_DRIVER_TIMER==TRUE */
#endif /* _TIMER_H_ */
