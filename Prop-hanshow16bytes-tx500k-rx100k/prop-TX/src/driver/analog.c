/**
 ****************************************************************************************
 *
 * @file analog.c
 *
 * @brief Analog driver for QN9020.
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
 * @addtogroup  ANALOG
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 *  HowToUse ACMP(Analog Comparator):
 *  
 *  (1) ACMP pin mux configuration
 *      ACMP0 Pin:
 *              P30_ACMP0_P_PIN_CTRL    ACMP0-P External single input pin
 *              P31_ACMP0_N_PIN_CTRL    ACMP0-N Reference single input pin when external reference configured
 *                                      If Internal reference configured, it can be used for other purpose
 *              P23_ACMP0_O_PIN_CTRL    ACMP0 Output pin, The result of P&N comparation
 *              P32_ACMP0_O_PIN_CTRL    ACMP0 Output pin, The result of P&N comparation
 *              
 *      ACMP1 Pin:
 *              P06_ACMP1_P_PIN_CTRL    ACMP1-P External single input pin
 *              P07_ACMP1_N_PIN_CTRL    ACMP1-N Reference single input pin when external reference configured
 *                                      If Internal reference configured, it can be used for other purpose
 *              P05_ACMP1_O_PIN_CTRL    ACMP1 Output pin, The result of P&N comparation
 *              P27_ACMP1_O_PIN_CTRL    ACMP1 Output pin, The result of P&N comparation
 *      syscon_SetPMCR0() and syscon_SetPMCR1() fucntion used to configure.
 *      
 *  (2) Analog pin enable
 *      ACMP0 Pin:  
 *          acmp_pin_enable(ACMP0_PIN_P, MASK_ENABLE);  Must be enabled
 *          acmp_pin_enable(ACMP0_PIN_N, MASK_ENABLE);  Must be enabled if external reference configured
 *      ACMP1 Pin:  
 *          acmp_pin_enable(ACMP1_PIN_P, MASK_ENABLE);  Must be enabled
 *          acmp_pin_enable(ACMP1_PIN_N, MASK_ENABLE);  Must be enabled if external reference configured
 *      
 *  (3) Program reference voltage, hysteresis ,interrupt and acmp enable using acmp_init() function
 *
 *  (4) NVIC interrupt
 *          NVIC_ClearPendingIRQ(ACMPx_IRQn);  x can be 0/1, Clear ACMP interrupt
 *          NVIC_EnableIRQ(ACMPx_IRQn);        x can be 0/1, Enable ACMP interrupt
 *
 *  (5) acmp_enable() function can be used to change interrupt and enable/disable acmp
 *  
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "analog.h"
#if CONFIG_ENABLE_DRIVER_ANALOG==TRUE
#include "nvds.h"

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
#if ACMP_CALLBACK_EN==TRUE
/// Analog comparator environment parameters
/// The callback pointer is user specified, when acmp_init() called. 
/// And it will execute when ACMP interrupt occur.
struct acmp_env_tag
{
    void                (*callback)(void);
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_ACMP0==TRUE && CONFIG_ACMP0_DEFAULT_IRQHANDLER==TRUE)
/// Analog comparator0 environment variable
static struct acmp_env_tag acmp0_env;
#endif

#if (CONFIG_ENABLE_DRIVER_ACMP1==TRUE && CONFIG_ACMP1_DEFAULT_IRQHANDLER==TRUE)
/// Analog comparator1 environment variable
static struct acmp_env_tag acmp1_env;
#endif

#endif

int16_t TEMP_OFFSET = DEFAULT_TEMP_OFFSET;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_ACMP0==TRUE && CONFIG_ACMP0_DEFAULT_IRQHANDLER==TRUE)
 /**
 ****************************************************************************************
 * @brief ACMP0 interrupt handler
 ****************************************************************************************
 */
void ACMP0_IRQHandler(void)
{
#if ACMP_CALLBACK_EN==TRUE
    if (acmp0_env.callback != NULL)
    {
        acmp0_env.callback();
    }
#endif
}
#endif

#if (CONFIG_ENABLE_DRIVER_ACMP1==TRUE && CONFIG_ACMP1_DEFAULT_IRQHANDLER==TRUE)
 /**
 ****************************************************************************************
 * @brief ACMP1 interrupt handler
 ****************************************************************************************
 */
void ACMP1_IRQHandler(void)
{
#if ACMP_CALLBACK_EN==TRUE
    if (acmp1_env.callback != NULL)
    {
        acmp1_env.callback();
    }
#endif
}
#endif

#if (CONFIG_ENABLE_DRIVER_ACMP0==TRUE || CONFIG_ENABLE_DRIVER_ACMP1==TRUE)
/**
 ****************************************************************************************
 * @brief  Initialize and enable ACMP
 * @param[in]    acmpch       ACMP0 or ACMP1
 * @param[in]    acmpref      ACMP voltage: external or internal VDD (ref pin: ACMPx_N)
 *  This parameter can be one of the following value:
 *      @arg    EXT_REF             External reference.
 *      @arg    VDD_x               Where x can be (1..15) to select internal reference voltage
 * @param[in]    acmpint      ACMP interrutp condition: acmp output 1 or 0 generate interrupt
 *  This parameter can be one of the following value:
 *      @arg    ACMPO_0_GEN_INT     When ACMP output is 0, generate interrupt
 *      @arg    ACMPO_1_GEN_INT     When ACMP output is 1, generate interrupt
 * @param[in]    acmphyst     ACMP  Hysteresis status set
 *  This parameter can be one of the following value:
 *      @arg    HYST_DISABLE        Disable Hysteresis
 *      @arg    HYST_ENABLE         Enable  Hysteresis
 * @param[in]    callback     Callback in interrupt handler
 * @description
 *  This function is used to initialize specified analog comparator, and to register callback function.
 *****************************************************************************************
 */
void acmp_init(enum ACMP_CH acmpch, enum ACMP_REF acmpref, enum ACMP_INT_COND acmpint, enum ACMP_HYST_STATUS acmphyst, void (*callback)(void))
{
    uint32_t mask = 0;
    uint32_t reg = 0;

#if CONFIG_ENABLE_DRIVER_ACMP0==TRUE 
    if (acmpch == ACMP0) {
        
#if ACMP_CALLBACK_EN==TRUE
        acmp0_env.callback = callback;
#endif
        mask = SYSCON_MASK_ACMP0_REF
             | SYSCON_MASK_ACMP0_HYST_EN
             | SYSCON_MASK_ACMP0_VALUE
             | SYSCON_MASK_ACMP0_EN;

        // set interrupt condition
        reg = (acmpint << SYSCON_POS_ACMP0_VALUE)
            | (acmpref << SYSCON_POS_ACMP0_REF)
            | (acmphyst<< SYSCON_POS_ACMP0_HYST) // +/- 20mv
            | SYSCON_MASK_ACMP0_EN;
        
        syscon_SetAnalogCRWithMask(QN_SYSCON, mask, reg);
        delay(30);
        
#if CONFIG_ACMP0_ENABLE_INTERRUPT==TRUE
        NVIC_ClearPendingIRQ(ACMP0_IRQn);
        /* Enable the comparator Interrupt */
        NVIC_EnableIRQ(ACMP0_IRQn);
#endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_ACMP1==TRUE 
     if (acmpch == ACMP1) {
         
#if ACMP_CALLBACK_EN==TRUE
        acmp1_env.callback = callback;
#endif
        mask = SYSCON_MASK_ACMP1_REF
             | SYSCON_MASK_ACMP1_HYST_EN
             | SYSCON_MASK_ACMP1_VALUE
             | SYSCON_MASK_ACMP1_EN;

        // set interrupt condition
        reg = (acmpint << SYSCON_POS_ACMP1_VALUE)
            | (acmpref << SYSCON_POS_ACMP1_REF)
            | (acmphyst<< SYSCON_POS_ACMP1_HYST) // +/- 20mv
            | SYSCON_MASK_ACMP1_EN;
        
        syscon_SetAnalogCRWithMask(QN_SYSCON, mask, reg);
        delay(30);
        
#if CONFIG_ACMP1_ENABLE_INTERRUPT==TRUE
        NVIC_ClearPendingIRQ(ACMP1_IRQn);
        /* Enable the comparator Interrupt */
        NVIC_EnableIRQ(ACMP1_IRQn);
#endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief  Enable ACMP with interrupt condition
 * @param[in]    acmpch       ACMP0 or ACMP0
 * @param[in]    acmpint      ACMP interrutp condition: acmp output 1 or 0 generate interrupt
 *  This parameter can be one of the following value:
 *      @arg    ACMPO_0_GEN_INT     When ACMP output is 1, generate interrupt
 *      @arg    ACMPO_1_GEN_INT     When ACMP output is 0, generate interrupt
 * @param[in]    able         MASK_ENABLE or MASK_DISABLE
 * @description
 *  This function is used to enable or disable specified analog comparator with interrupt condition.
 *  If Comparators aren't used during sleep, please set ACMP0 and ACMP1 to disabled for lower sleep
 *  leakage current.
 *****************************************************************************************
 */
void acmp_enable(enum ACMP_CH acmpch, enum ACMP_INT_COND acmpint, uint32_t able)
{
    uint32_t mask = 0;
    uint32_t reg = 0;

#if CONFIG_ENABLE_DRIVER_ACMP0==TRUE 
    if (acmpch == ACMP0) {
        mask = SYSCON_MASK_ACMP0_EN|SYSCON_MASK_ACMP0_VALUE;
        if (able == MASK_DISABLE) {
            mask |= SYSCON_MASK_ACMP0_REF;
        }
        reg =  (SYSCON_MASK_ACMP0_EN & able) | (acmpint << SYSCON_POS_ACMP0_VALUE);
    }
#endif

#if CONFIG_ENABLE_DRIVER_ACMP1==TRUE 
    if (acmpch == ACMP1) {
        mask = SYSCON_MASK_ACMP1_EN|SYSCON_MASK_ACMP1_VALUE;
        if (able == MASK_DISABLE) {
            mask |= SYSCON_MASK_ACMP1_REF;
        }
        reg =  (SYSCON_MASK_ACMP1_EN & able) | (acmpint << SYSCON_POS_ACMP1_VALUE);
    }
#endif
    syscon_SetAnalogCRWithMask(QN_SYSCON, mask, reg);
}
#endif

/**
 ****************************************************************************************
 * @brief  Enable/Disable battery monitor
 * @param[in]    able         MASK_ENABLE or MASK_DISABLE
 * @description
 *  This function is used to enable or disable battery monitor.
 *****************************************************************************************
 */
void battery_monitor_enable(uint32_t able)
{
    syscon_SetAnalogCRWithMask(QN_SYSCON, SYSCON_MASK_BT_EN, able);
}


/**
 ****************************************************************************************
 * @brief  Enable/Disable brown out detector
 * @param[in]    able         MASK_ENABLE or MASK_DISABLE
 * @description
 *  This function is used to enable or disable brown out detector.
 *****************************************************************************************
 */
void brown_out_enable(uint32_t able)
{
    syscon_SetAnalogCRWithMask(QN_SYSCON, SYSCON_MASK_BD_EN, able);
}

/**
 ****************************************************************************************
 * @brief  Enable/Disable temperature sensor
 * @param[in]    able         MASK_ENABLE or MASK_DISABLE
 * @description
 *  This function is used to enable or disable temperature sensor.
 *****************************************************************************************
 */
void temp_sensor_enable(uint32_t able)
{
    syscon_SetAnalogCRWithMask(QN_SYSCON, SYSCON_MASK_TS_EN, able);

    if (able) {
        // read from NVDS
        uint16_t len = 4;
        uint32_t data = 0;
        TEMP_OFFSET = DEFAULT_TEMP_OFFSET;
        if (NVDS_OK == nvds_get(NVDS_TAG_TEMPERATURE_OFFSET, &len, (uint8_t *)&data)) {
            if ((data>0xFD80) && (data<0xFFFF)) { // -640 ~ 0
                TEMP_OFFSET = data;
            }
        }
    }
}

/**
 ****************************************************************************************
 * @brief  Check analog comparator sleep is allowed or not
 * @return  TRUE or FALSE
 * @description
 *  This function is used to check the analog comparator sleep is allowed or not, 
 *  pin ACMPx_O should be configured before this function is called.
 *****************************************************************************************
 */
bool acmp_sleep_allowed(void)
{
    uint32_t ana_reg;
    uint32_t acmp_int_mask;
    uint32_t acmp_out_mask;

    ana_reg = syscon_GetAnalogCR(QN_SYSCON);
    
    if (ana_reg & SYSCON_MASK_ACMP0_EN) {
        acmp_int_mask = (ana_reg & SYSCON_MASK_ACMP0_VALUE) ? 1 : 0;
        // ACMP0_O is pin P3.2 or P2.3 ? 
        acmp_out_mask = (gpio_gpio_GetInputData(QN_GPIO) & GPIO_P32) ? 1 : 0;
        
        if (acmp_int_mask ^ acmp_out_mask) {
            return FALSE;
        }
    }
    
    if (ana_reg & SYSCON_MASK_ACMP1_EN) {
        acmp_int_mask = (ana_reg & SYSCON_MASK_ACMP1_VALUE) ? 1 : 0;
        // ACMP1_O is pin P0.5 or P2.7 ? 
        acmp_out_mask = (gpio_gpio_GetInputData(QN_GPIO) & GPIO_P05) ? 1 : 0;

        if (acmp_int_mask ^ acmp_out_mask) {
            return FALSE;
        }
    }

    return TRUE;
}

#endif /* CONFIG_ENABLE_DRIVER_ANALOG==TRUE */
/// @} ANALOG
