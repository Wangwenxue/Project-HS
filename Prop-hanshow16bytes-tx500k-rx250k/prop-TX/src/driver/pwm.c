/**
 ****************************************************************************************
 *
 * @file pwm.c
 *
 * @brief PWM Driver for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  PWM
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "pwm.h"
#if ((CONFIG_ENABLE_DRIVER_PWM0==TRUE || CONFIG_ENABLE_DRIVER_PWM1==TRUE))


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_PWM0==TRUE && CONFIG_PWM0_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief PWM ch0 interrupt handler
 ****************************************************************************************
 */
void PWM0_IRQHandler(void)
{
    uint32_t reg;

    reg = pwm_pwm_GetIntStatus(QN_PWM);
    // ch0
    if (reg & PWM_MASK_CH0_IF) {
        /* clear interrupt flag */
        pwm_pwm_ClrIntStatus(QN_PWM, PWM_MASK_CH0_IF);
    }
}
#endif /* CONFIG_PWM0_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_PWM1==TRUE && CONFIG_PWM1_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief PWM ch1 interrupt handler
 ****************************************************************************************
 */
void PWM1_IRQHandler(void)
{
    uint32_t reg;

    reg = pwm_pwm_GetIntStatus(QN_PWM);
    // ch1
    if (reg & PWM_MASK_CH1_IF) {

        /* clear interrupt flag */
        pwm_pwm_ClrIntStatus(QN_PWM, PWM_MASK_CH1_IF);
    }
}
#endif /* CONFIG_PWM1_DEFAULT_IRQHANDLER==TRUE */


/**
 ****************************************************************************************
 * @brief Initialize the PWM
 * @param[in]    pwmch          PWM_CH0, PWM_CH1
 * @description
 * This function is used to initialize the specified PWM channel. 
 *
 ****************************************************************************************
 */
void pwm_init(enum PWM_CH pwmch)
{
    uint32_t mask = 0, reg = 0;

    pwm_clock_on();

#if CONFIG_ENABLE_DRIVER_PWM0==TRUE    
    if (pwmch == PWM_CH0) {
        mask = PWM_MASK_CH0_POL|PWM_MASK_CH0_IE|PWM_MASK_CH0_EN;
        #if CONFIG_PWM0_ENABLE_INTERRUPT==TRUE
        reg |= PWM_MASK_CH0_IE;  // enable PWM ch0 int
        NVIC_EnableIRQ(PWM0_IRQn);
        #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_PWM1==TRUE
    if (pwmch == PWM_CH1) {
        mask = PWM_MASK_CH1_POL|PWM_MASK_CH1_IE|PWM_MASK_CH1_EN;
        #if CONFIG_PWM1_ENABLE_INTERRUPT==TRUE
        reg |= PWM_MASK_CH1_IE;  // enable PWM ch1 int
        NVIC_EnableIRQ(PWM1_IRQn);
        #endif
    }
#endif

    pwm_pwm_SetCRWithMask(QN_PWM, mask, reg);  // config PWM
}

/**
 ****************************************************************************************
 * @brief Config the PWM
 * @param[in]    pwmch          PWM_CH0, PWM_CH1
 * @param[in]    pscal          PWM prescaler value: 0x0 ~ 0x3FF
 * @param[in]    periodcount    period count: 0x0 ~ 0xFF
 * @param[in]    pulsecount     pulse count: 0x0 ~ 0xFF, pulsecount should less than periodcount
 * @return success(1) or failed(0)
 * @description
 * This function is used to config the specified PWM channel. It contains configuraion of pre-scaler,
 * period, and pulse width.
 *
 * e.g: pwm_config(PWM_CH0, PWM_PSCAL_DIV, PWM_COUNT_US(1000, PWM_PSCAL_DIV), PWM_COUNT_US(500, PWM_PSCAL_DIV));
 *
 ****************************************************************************************
 */
uint8_t pwm_config(enum PWM_CH pwmch, uint16_t pscal, uint8_t periodcount, uint8_t pulsecount)
{
    uint32_t cr_mask = 0, cr_reg = 0;
    uint32_t pscal_mask = 0, pscal_reg = 0;
    uint32_t pcp_mask = 0, pcp_reg = 0;
    uint8_t level = 0;

    if (pscal > 0x3FF) {
        return 0;
    }
    
    if (periodcount == 0) {
        pulsecount = 0;
    }
    else {
        periodcount -= 1;
        if (pulsecount > periodcount) {
            level = 1;
        }
    }

#if CONFIG_ENABLE_DRIVER_PWM0==TRUE
    if (pwmch == PWM_CH0) {
        // set compare output level
        cr_mask = PWM_MASK_CH0_POL;
        if (level == 1) {
            cr_reg = cr_mask;
        }
        else {
            cr_reg = 0;
        }
        
        // set prescaler
        pscal_reg = pscal;
        pscal_mask = PWM_MASK_CH0_PSCL;

        // set period&compare count value
        pcp_reg = (pulsecount << PWM_POS_CH0_CMP) + periodcount;
        pcp_mask = PWM_MASK_CH0_CMP|PWM_MASK_CH0_PERIOD;
    }
#endif

#if CONFIG_ENABLE_DRIVER_PWM1==TRUE
    if (pwmch == PWM_CH1) {
        // set compare output level
        cr_mask = PWM_MASK_CH1_POL;
        if (level == 1) {
            cr_reg = cr_mask;
        }
        else {
            cr_reg = 0;
        }
        
        // set prescaler
        pscal_reg = pscal<<PWM_POS_CH1_PSCL;
        pscal_mask = PWM_MASK_CH1_PSCL;

        // set period&compare count value
        pcp_reg = (pulsecount << PWM_POS_CH1_CMP) + (periodcount << PWM_POS_CH1_PERIOD);
        pcp_mask = PWM_MASK_CH1_CMP|PWM_MASK_CH1_PERIOD;
    }
#endif

    pwm_pwm_SetCRWithMask(QN_PWM, cr_mask, cr_reg);         // config PWM
    pwm_pwm_SetPSCLWithMask(QN_PWM, pscal_mask, pscal_reg); // set PWM prescaler
    pwm_pwm_SetPCPWithMask(QN_PWM, pcp_mask, pcp_reg);      // set PWM period, pulse
    return 1;
}

#endif /* CONFIG_ENABLE_DRIVER_PWM==TRUE */
/// @} PWM
