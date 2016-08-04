/**
 ****************************************************************************************
 *
 * @file timer.c
 *
 * @brief TIMER driver for QN9020.
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
 * @addtogroup  TIMER
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "timer.h"
#if ((CONFIG_ENABLE_DRIVER_TIMER0==TRUE || CONFIG_ENABLE_DRIVER_TIMER1==TRUE \
    || CONFIG_ENABLE_DRIVER_TIMER1==TRUE  || CONFIG_ENABLE_DRIVER_TIMER3==TRUE))

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_TIMER0==TRUE && CONFIG_TIMER0_DEFAULT_IRQHANDLER==TRUE)
///TIMER0 environment variable
struct timer_env_tag timer0_env;
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER1==TRUE && CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE)
///TIMER1 environment variable
struct timer_env_tag timer1_env;
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER2==TRUE && CONFIG_TIMER2_DEFAULT_IRQHANDLER==TRUE)
///TIMER2 environment variable
struct timer_env_tag timer2_env;
#endif
#if (CONFIG_ENABLE_DRIVER_TIMER3==TRUE && CONFIG_TIMER3_DEFAULT_IRQHANDLER==TRUE)
///TIMER3 environment variable
struct timer_env_tag timer3_env;
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Start the timer delay in micro seconds, until elapsed
 *
 * @param[in]    TIMER          QN_TIMER0,1,2,3
 * @param[in]    pscal          timer prescaler value
 * @param[in]    count          counter value
 * @description
 *  This function is used to set a precise delay. HOW TO SET? e.g:
 *  - if TIMER_CLK = 8000000(8MHz), PSCL_DIV = 7, then
 *  - PSCL_CLK = (TIMER_CLK) / (PSCL_DIV + 1) = 1000000Hz(1MHz), so
 *  - delayInUs range: 1us - 4294967295 us  (32bit)
 *  - delayInUs range: 1us - 65535 us       (16bit)
 *
 *  timer_delay(QN_TIMER0, 7, TIMER_COUNT_US(100, 7)); // timer delay 100us
 *
 ****************************************************************************************
 */
void timer_delay(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t count)
{
    uint32_t reg;

    // set the timer top count value
    timer_timer_SetTOPR(TIMER, count);

    reg = CLK_PSCL                              /* set clock source to prescaler clock */
        | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
        | FREE_RUNNING_MOD;                     /* select free running mode */
    timer_timer_SetCR(TIMER, reg);              /* must disable timer first */

    timer_timer_SetCRWithMask(TIMER, TIMER_MASK_TEN, MASK_ENABLE);
    /* wait until delay time has elapsed */
    while (!(timer_timer_GetIntFlag(TIMER) & TIMER_MASK_TOVF));

    // disable timer
    timer_timer_SetCRWithMask(TIMER, TIMER_MASK_TEN, MASK_DISABLE);
    /* clear interrupt flag */
    timer_timer_ClrIntFlag(TIMER, TIMER_MASK_TOVF);
}

#if (CONFIG_ENABLE_DRIVER_TIMER0==TRUE && CONFIG_TIMER0_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief Timer0 interrupt handler
 ****************************************************************************************
 */
void TIMER0_IRQHandler(void)
{
    uint32_t reg;

    reg = timer_timer_GetIntFlag(QN_TIMER0);
    // timer
    if (reg & TIMER_MASK_TOVF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER0, TIMER_MASK_TOVF);

        #if TIMER0_CAP_MODE==INCAP_EVENT_MOD
        timer0_env.count = timer_timer_GetICER(QN_TIMER0); // it's the event number occurred during specified time duration
        #endif
    }
    // compare
    if (reg & TIMER_MASK_OCF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER0, TIMER_MASK_OCF);
    }
    // capture
    if (reg & TIMER_MASK_ICF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER0, TIMER_MASK_ICF);

        #if TIMER0_CAP_MODE==INCAP_TIMER_MOD || TIMER0_CAP_MODE==INCAP_COUNTER_MOD
        timer0_env.count = timer_timer_GetCCR(QN_TIMER0); // it's the timer counter value
        #endif
    }

#if TIMER0_CALLBACK_EN==TRUE
    // Call end of timer overflow
    if(timer0_env.callback != NULL)
    {
        timer0_env.callback();
    }
#endif
}
#endif /* CONFIG_TIMER0_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_TIMER1==TRUE && CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief Timer1 interrupt handler
 ****************************************************************************************
 */
void TIMER1_IRQHandler(void)
{
    uint32_t reg;

    reg = timer_timer_GetIntFlag(QN_TIMER1);
    // timer
    if (reg & TIMER_MASK_TOVF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER1, TIMER_MASK_TOVF);

        #if TIMER1_CAP_MODE==INCAP_EVENT_MOD
        timer1_env.count = timer_timer_GetICER(QN_TIMER1); // it's the event number occurred during specified time duration
        #endif
    }
    // compare
    if (reg & TIMER_MASK_OCF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER1, TIMER_MASK_OCF);
    }
    // capture
    if (reg & TIMER_MASK_ICF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER1, TIMER_MASK_ICF);

        #if TIMER1_CAP_MODE==INCAP_TIMER_MOD || TIMER1_CAP_MODE==INCAP_COUNTER_MOD
        timer1_env.count = timer_timer_GetCCR(QN_TIMER1); // it's the timer counter value
        #endif
    }
    
#if TIMER1_CALLBACK_EN==TRUE
    // Call end of timer overflow
    if(timer1_env.callback != NULL)
    {
        timer1_env.callback();
    }
#endif
}
#endif /* CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_TIMER2==TRUE && CONFIG_TIMER2_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief Timer2 interrupt handler
 ****************************************************************************************
 */
void TIMER2_IRQHandler(void)
{
    uint32_t reg;

    reg = timer_timer_GetIntFlag(QN_TIMER2);
    // timer
    if (reg & TIMER_MASK_TOVF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER2, TIMER_MASK_TOVF);

        #if TIMER2_CAP_MODE==INCAP_EVENT_MOD
        timer2_env.count = timer_timer_GetICER(QN_TIMER2); // it's the event number occurred during specified time duration
        #endif
    }
    // compare
    if (reg & TIMER_MASK_OCF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER2, TIMER_MASK_OCF);
    }
    // capture
    if (reg & TIMER_MASK_ICF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER2, TIMER_MASK_ICF);

        #if TIMER2_CAP_MODE==INCAP_TIMER_MOD || TIMER2_CAP_MODE==INCAP_COUNTER_MOD
        timer2_env.count = timer_timer_GetCCR(QN_TIMER2); // it's the timer counter value
        #endif
    }
    
#if TIMER2_CALLBACK_EN==TRUE
    // Call end of timer overflow
    if(timer2_env.callback != NULL)
    {
        timer2_env.callback();
    }
#endif
}
#endif /* CONFIG_TIMER2_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_TIMER3==TRUE && CONFIG_TIMER3_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief Timer3 interrupt handler
 ****************************************************************************************
 */
void TIMER3_IRQHandler(void)
{
    uint32_t reg;

    reg = timer_timer_GetIntFlag(QN_TIMER3);
    // timer
    if (reg & TIMER_MASK_TOVF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER3, TIMER_MASK_TOVF);

        #if TIMER3_CAP_MODE==INCAP_EVENT_MOD
        timer3_env.count = timer_timer_GetICER(QN_TIMER3); // it's the event number occurred during specified time duration
        #endif
    }
    // compare
    if (reg & TIMER_MASK_OCF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER3, TIMER_MASK_OCF);
    }
    // capture
    if (reg & TIMER_MASK_ICF) {
        /* clear interrupt flag */
        timer_timer_ClrIntFlag(QN_TIMER3, TIMER_MASK_ICF);

        #if TIMER3_CAP_MODE==INCAP_TIMER_MOD || TIMER3_CAP_MODE==INCAP_COUNTER_MOD
        timer3_env.count = timer_timer_GetCCR(QN_TIMER3); // it's the timer counter value
        #endif
    }
    
#if TIMER3_CALLBACK_EN==TRUE
    // Call end of timer overflow
    if(timer3_env.callback != NULL)
    {
        timer3_env.callback();
    }
#endif
}
#endif /* CONFIG_TIMER3_DEFAULT_IRQHANDLER==TRUE */

/**
 ****************************************************************************************
 * @brief Initialize the timer
 * @param[in]    TIMER          QN_TIMER0,1,2,3
 * @param[in]    callback       Call back function name for specified interrupt event
 * @description
 * Initialize the timer module. 
 ****************************************************************************************
 */
void timer_init(QN_TIMER_TypeDef *TIMER, void (*callback)(void))
{
    timer_enable(TIMER, MASK_DISABLE);
    timer_clock_on(TIMER);

#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if ( TIMER == QN_TIMER0 ) {

    #if CONFIG_TIMER0_ENABLE_INTERRUPT==TRUE
    /* Enable the TIMER0 Interrupt */
    NVIC_EnableIRQ(TIMER0_IRQn);
    #endif

    #if ((TIMER0_CALLBACK_EN==TRUE) && (CONFIG_TIMER0_DEFAULT_IRQHANDLER==TRUE))
        timer0_env.count = 0;
        timer0_env.callback = callback;
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if ( TIMER == QN_TIMER1 ) {

    #if CONFIG_TIMER1_ENABLE_INTERRUPT==TRUE
    /* Enable the TIMER1 Interrupt */
    NVIC_EnableIRQ(TIMER1_IRQn);
    #endif

    #if ((TIMER1_CALLBACK_EN==TRUE) && (CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE))
        timer1_env.count = 0;
        timer1_env.callback = callback;
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if ( TIMER == QN_TIMER2 ) {

    #if CONFIG_TIMER2_ENABLE_INTERRUPT==TRUE
    /* Enable the TIMER2 Interrupt */
    NVIC_EnableIRQ(TIMER2_IRQn);
    #endif

    #if ((TIMER2_CALLBACK_EN==TRUE) && (CONFIG_TIMER2_DEFAULT_IRQHANDLER==TRUE))
        timer2_env.count = 0;
        timer2_env.callback = callback;
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if ( TIMER == QN_TIMER3 ) {

    #if CONFIG_TIMER3_ENABLE_INTERRUPT==TRUE
    /* Enable the TIMER3 Interrupt */
    NVIC_EnableIRQ(TIMER3_IRQn);
    #endif

    #if ((TIMER3_CALLBACK_EN==TRUE) && (CONFIG_TIMER3_DEFAULT_IRQHANDLER==TRUE))
        timer3_env.count = 0;
        timer3_env.callback = callback;
    #endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Configure the timer
 * @param[in]    TIMER          QN_TIMER0,1,2,3
 * @param[in]    pscal          timer prescaler value
 * @param[in]    count          counter value
 * @description
 * Configure the timer to work in timer mode, with this function users can easily set Timer pre-scaler,
 * and count number.
 ****************************************************************************************
 */
void timer_config(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t count)
{
    uint32_t ctrl_reg = 0;

    if (count == 0) {
        return;    
    }
    count -= 1;

#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if ( TIMER == QN_TIMER0 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
        #if CONFIG_TIMER0_ENABLE_INTERRUPT==TRUE
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if ( TIMER == QN_TIMER1 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
        #if CONFIG_TIMER1_ENABLE_INTERRUPT==TRUE
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if ( TIMER == QN_TIMER2 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
        #if CONFIG_TIMER2_ENABLE_INTERRUPT==TRUE
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if ( TIMER == QN_TIMER3 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
        #if CONFIG_TIMER3_ENABLE_INTERRUPT==TRUE
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

    timer_timer_SetTOPR(TIMER, count);
    timer_timer_SetCR(TIMER, ctrl_reg);
}

/**
 ****************************************************************************************
 * @brief Configure the timer pwm function
 * @param[in]    TIMER           QN_TIMER0,1,2,3
 * @param[in]    pscal           timer prescaler value
 * @param[in]    periodcount     count value of period
 * @param[in]    pulsecount      count value of pulse
 * @description
 * Configure the timer to work in PWM mode, with this function users can easily set Timer pre-scaler,
 * period, and pulse width.
 ****************************************************************************************
 */
void timer_pwm_config(QN_TIMER_TypeDef *TIMER, uint32_t pscal, uint32_t periodcount, uint32_t pulsecount)
{
    uint32_t ctrl_reg = 0;
    
    if ((periodcount == 0) || (pulsecount == 0)) {
        return;
    }
    periodcount -= 1;
    pulsecount -= 1; 

#if CONFIG_ENABLE_DRIVER_TIMER0==TRUE
    if ( TIMER == QN_TIMER0 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
                 | TIMER_MASK_PWM_OE                     /* PWM output enable */
                 | TIMER_PWM_POL_CFG                     /* set PWM polarity */
        #if CONFIG_TIMER0_ENABLE_INTERRUPT==TRUE && TIMER_PWM_INT_EN==TRUE
                 | TIMER_MASK_OCIE                       /* enable compare int */
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER1==TRUE
    if ( TIMER == QN_TIMER1 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
                 | TIMER_MASK_PWM_OE                     /* PWM output enable */
                 | TIMER_PWM_POL_CFG                     /* set PWM polarity */
        #if CONFIG_TIMER1_ENABLE_INTERRUPT==TRUE && TIMER_PWM_INT_EN==TRUE
                 | TIMER_MASK_OCIE                       /* enable compare int */
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */

    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER2==TRUE
    if ( TIMER == QN_TIMER2 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
                 | TIMER_MASK_PWM_OE                     /* PWM output enable */
                 | TIMER_PWM_POL_CFG                     /* set PWM polarity */
        #if CONFIG_TIMER2_ENABLE_INTERRUPT==TRUE && TIMER_PWM_INT_EN==TRUE
                 | TIMER_MASK_OCIE                       /* enable compare int */
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */

    }
#endif

#if CONFIG_ENABLE_DRIVER_TIMER3==TRUE
    if ( TIMER == QN_TIMER3 ) {
        ctrl_reg = CLK_PSCL                              /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)             /* set prescaler value */
                 | TIMER_MASK_PWM_OE                     /* PWM output enable */
                 | TIMER_PWM_POL_CFG                     /* set PWM polarity */
        #if CONFIG_TIMER3_ENABLE_INTERRUPT==TRUE && TIMER_PWM_INT_EN==TRUE
                 | TIMER_MASK_OCIE                       /* enable compare int */
                 | TIMER_MASK_TOVIE                      /* enable timer overflow int */
        #endif
                 | FREE_RUNNING_MOD;                     /* select free running mode */
    }
#endif

    // set timer counter top value (period)
    timer_timer_SetTOPR(TIMER, periodcount);
    // set compare count value (pulse)
    timer_timer_SetCCR(TIMER, pulsecount);

    timer_timer_SetCR(TIMER, ctrl_reg);
}

/**
 ****************************************************************************************
 * @brief Configure timer capture function
 * @param[in]    TIMER          QN_TIMER0, QN_TIMER1, QN_TIMER2, QN_TIMER3
 * @param[in]    cap_mode       INCAP_TIMER_MOD, INCAP_EVENT_MOD, INCAP_COUNTER_MOD
 * @param[in]    pscal          timer prescaler value
 * @param[in]    count          count value, active in INCAP_EVENT_MOD
 * @param[in]    event_num      active in INCAP_COUNTER_MOD
 * @description
 * Configure the timer to work in capture mode, with this function users can easily set input capture mode,
 * Timer pre-scaler, and count/event number.
 ****************************************************************************************
 */
void timer_capture_config(QN_TIMER_TypeDef *TIMER, uint32_t cap_mode, uint32_t pscal, uint32_t count, uint32_t event_num)
{
    uint32_t ctrl_reg;

    switch (cap_mode) {
    case INCAP_TIMER_MOD:

        // 1. input capture timer mode
        ctrl_reg = CLK_PSCL                             /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)            /* set prescaler value */
                 | TIMER_MASK_ICNCE                     /* enable input capure noise canceller */
                 | TIMER_INCAP_PIN_CFG                  /* set input capure pin to pin0 */
                 | INCAP_SRC_PIN                        /* set input capure source to capure PIN */
                 | INCAP_EDGE_POS                       /* set input capure edge to pos edge */
                 | INCAP_TIMER_MOD                      /* select input capure timer mode */
#if TIMER_CAP_INT_EN==TRUE
                 | TIMER_MASK_ICIE                      /* enable input capure int */
#endif
                 ;
        timer_timer_SetCR(TIMER, ctrl_reg);
        break;
    case INCAP_EVENT_MOD:
        
        if (count == 0) {
            return;    
        }
        // set count value
        timer_timer_SetTOPR(TIMER, count-1);

        // 2. input capture event mode
        ctrl_reg = CLK_PSCL                             /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)            /* set prescaler value */
                 | TIMER_MASK_ICNCE                     /* enable input capure noise canceller */
                 | TIMER_INCAP_PIN_CFG                  /* set input capure pin to pin0 */
                 | INCAP_SRC_PIN                        /* set input capure source to capure PIN */
                 | INCAP_EDGE_POS                       /* set input capure edge to pos edge */
                 | INCAP_EVENT_MOD                      /* select input capure event mode */
#if TIMER_CAP_INT_EN==TRUE
                 | TIMER_MASK_TOVIE                     /* enable timer overflow int */
#endif
                 ;
        timer_timer_SetCR(TIMER, ctrl_reg);
        break;
    case INCAP_COUNTER_MOD:
        // this mode need reset timer at first
        timer_reset(TIMER);
        // set event number
        timer_timer_SetTOPR(TIMER, event_num);

        // 3. input capture counter mode
        ctrl_reg = CLK_PSCL                             /* set clock source to prescaler clock */
                 | (pscal << TIMER_POS_PSCL)            /* set prescaler value */
                 | TIMER_MASK_ICNCE                     /* enable input capure noise canceller */
                 | TIMER_INCAP_PIN_CFG                  /* set input capure pin to pin0 */
                 | INCAP_SRC_PIN                        /* set input capure source to capure PIN */
                 | INCAP_EDGE_POS                       /* set input capure edge to pos edge */
                 | INCAP_COUNTER_MOD                    /* select input capure counter mode */
#if TIMER_CAP_INT_EN==TRUE
                 | TIMER_MASK_ICIE                      /* enable input capure int */
#endif
                 ;
        timer_timer_SetCR(TIMER, ctrl_reg);
        break;
    default:
        break;
    }
}

#endif /* CONFIG_ENABLE_DRIVER_TIMER==TRUE */
/// @} TIMER
