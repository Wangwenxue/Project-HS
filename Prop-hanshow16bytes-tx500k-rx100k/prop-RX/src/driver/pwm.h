/**
 ****************************************************************************************
 *
 * @file pwm.h
 *
 * @brief Header file for QN9020 PWM.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _PWM_H_
#define _PWM_H_
#include "driver_config.h"
#if (CONFIG_ENABLE_DRIVER_PWM0==TRUE || CONFIG_ENABLE_DRIVER_PWM1==TRUE)

/**
 ****************************************************************************************
 * @defgroup PWM PWM Driver
 * @ingroup DRIVERS
 * @brief PWM driver
 *
 *  QN9020 PWM module provides two channels with programmable period and duty cycle.
 *  The main features of PWM are listed as follow:
 *    - Two 8-bit auto-reload count down counter
 *    - Programmable 10-bit prescaler for both channels
 *    - Predictable PWM initial output state
 *    - Buffered compare register and polarity register to ensure correct PWM output
 *    - Programmable overflow interrupt generation
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example pwm_example.c
 * This is an example of how to use the PWM driver.
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "timer.h"
#include "sleep.h"


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// Set PWM divider
#define PWM_DIV(n)                     ((n) + 1)
/// Set PWM clock
#define PWM_CLK(x, n)                  (TIMER_CLK(x) / (PWM_DIV(n)))
/// Set prescaler
//#define PWM_PSCAL_DIV                  15   // timer divider not bypass
#define PWM_PSCAL_DIV                  63   // timer divider bypass

/// Set period&compare count value  (periodInS * (PWM_CLK))
#define PWM_COUNT_S(s, pscl_div)       ((s) * PWM_CLK(TIMER_DIV, pscl_div))
/// Set period&compare count value  (periodInMs * (PWM_CLK / 1000))
#define PWM_COUNT_MS(ms, pscl_div)     ((ms) * PWM_CLK(TIMER_DIV, pscl_div) / 1000)
/// Set period&compare count value  (periodInUs * (PWM_CLK / 1000000))
#define PWM_COUNT_US(us, pscl_div)     ((us) * PWM_CLK(TIMER_DIV, pscl_div) / 1000000)

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// PWM channel
enum PWM_CH
{
    PWM_CH0 = PWM_MASK_CH0_EN,      /*!< PWM channel 0 */
    PWM_CH1 = PWM_MASK_CH1_EN       /*!< PWM channel 1 */
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  Enable or disable pwm
 *
 * @param[in]  pwmch          PWM_CH0 or PWM_CH1
 * @param[in]  able           MASK_ENABLE or MASK_DISABLE
 * @description
 * This function is used to enable or disable the specified PWM channel
 *****************************************************************************************
 */
__STATIC_INLINE void pwm_enable(enum PWM_CH pwmch, uint32_t able)
{
    uint32_t dev=0;

    pwm_pwm_SetCRWithMask(QN_PWM, pwmch, able);

    if(pwmch == PWM_CH0)
    {
        dev = PM_MASK_PWM0_ACTIVE_BIT;
    }
    else if(pwmch == PWM_CH1)
    {
        dev = PM_MASK_PWM1_ACTIVE_BIT;
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
 * @brief   Enable PWM module clock
 * @description
 *  This function is used to enable PWM module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void pwm_clock_on(void)
{
    // enable PWM module clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_PWM);
}

/**
 ****************************************************************************************
 * @brief   Disable PWM module clock
 * @description
 *  This function is used to disable PWM module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void pwm_clock_off(void)
{
    // disable PWM module clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_PWM);
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_PWM0==TRUE && CONFIG_PWM0_DEFAULT_IRQHANDLER==TRUE)
void PWM0_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_PWM1==TRUE && CONFIG_PWM1_DEFAULT_IRQHANDLER==TRUE)
void PWM1_IRQHandler(void);
#endif

extern void pwm_init(enum PWM_CH pwmch);
extern uint8_t pwm_config(enum PWM_CH pwmch, uint16_t pscal, uint8_t periodcount, uint8_t pulsecount);


/// @} PWM
#endif /* CONFIG_ENABLE_DRIVER_PWM==TRUE */
#endif /* _PWM_H_ */
