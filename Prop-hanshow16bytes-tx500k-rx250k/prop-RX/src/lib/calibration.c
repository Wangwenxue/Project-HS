/**
 ****************************************************************************************
 *
 * @file calibration.c
 *
 * @brief Calibration driver for QN9020.
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
 * @addtogroup  CALIB
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "calibration.h"
#if CONFIG_ENABLE_DRIVER_CALIB==TRUE && CONFIG_ENABLE_ROM_DRIVER_CALIB==FALSE

#include "timer.h"
#include "rtc.h"
#include "adc.h"
#include "qnrf.h"

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

///CALIBRATION environment parameters
struct calibration_env_tag
{
    // 0 - ref_callback
    // 1 - rc_callback
    // 2 - lo_callback
    // 3 - kvco_callback
    // 4 - pa_callback
    // 5 - r_callback
    // 6 - ros_callback
    // 7 - rco_callback
    void (*calibration_callback[8])(void);
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#if CALIB_CALLBACK_EN==TRUE
static struct calibration_env_tag calibration_env;
#endif

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Calibration interrupt handler
 ****************************************************************************************
 */
#if CONFIG_CALIB_DEFAULT_IRQHANDLER==TRUE
void CALIB_IRQHandler(void)
{
    uint32_t reg;

    reg = cal_cal_GetSR(QN_CALIB);
    if (reg & CALIB_MASK_REF_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_REF_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[0] != NULL)
        {
            calibration_env.calibration_callback[0]();
        }
#endif
    }

    if (reg & CALIB_MASK_RC_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_RC_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[1] != NULL)
        {
            calibration_env.calibration_callback[1]();
        }
#endif
    }

    if (reg & CALIB_MASK_LO_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_LO_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[2] != NULL)
        {
            calibration_env.calibration_callback[2]();
        }
#endif
    }

    if (reg & CALIB_MASK_KVCO_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_KVCO_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[3] != NULL)
        {
            calibration_env.calibration_callback[3]();
        }
#endif
    }

    if (reg & CALIB_MASK_PA_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_PA_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[4] != NULL)
        {
            calibration_env.calibration_callback[4]();
        }
#endif

        // Disable RF set by SW, enable set by HW
//        rf_enable_sw_set_freq(MASK_DISABLE);
//        rf_enable(RF_TX, MASK_DISABLE, MASK_DISABLE, 0x0);
    }

    if (reg & CALIB_MASK_R_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_R_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[5] != NULL)
        {
            calibration_env.calibration_callback[5]();
        }
#endif
    }

    if (reg & CALIB_MASK_ROS_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_ROS_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[6] != NULL)
        {
            calibration_env.calibration_callback[6]();
        }
#endif
    }

    if (reg & CALIB_MASK_RCO_DONE_IF) {
        /* clear interrupt flag */
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_RCO_DONE_IF);

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[7] != NULL)
        {
            calibration_env.calibration_callback[7]();
        }
#endif
    }
}
#endif /* CONFIG_CALIB_DEFAULT_IRQHANDLER==TRUE */

/**
 ****************************************************************************************
 * @brief Intialization calibration
 *****************************************************************************************
 */
void calibration_init(enum CLK_TYPE type)
{
    uint32_t mask, reg;
    uint32_t adc_clk_div = ADC_CLK_1000000;

#if CALIB_CALLBACK_EN==TRUE
    uint8_t i;
    for(i=0; i<8; i++)
    {
        calibration_env.calibration_callback[i] = NULL;
    }
#endif

    // Set AHB clk to 16MHz, CAL module work at 16MHz
    mask = SYSCON_MASK_CLK_MUX
         | SYSCON_MASK_AHB_DIV_BYPASS
         | SYSCON_MASK_AHB_DIVIDER;

    if (type == XTAL_16M) {
        reg = (CLK_XTAL<<SYSCON_POS_CLK_MUX)                // XTAL 16M
            | SYSCON_MASK_AHB_DIV_BYPASS;                   // AHB Divider bypass, set AHB clk to 16M
        adc_clk_div = 0x03;
    }
    else if (type == XTAL_32M){
        reg = (CLK_XTAL<<SYSCON_POS_CLK_MUX)                // XTAL 32M
            | 0 << SYSCON_POS_AHB_DIVIDER;                  // AHB Divider = 0, set AHB clk to 16M
        adc_clk_div = 0x04;
    }
    else {
        reg = ((uint32_t)CLK_INT_32M<<SYSCON_POS_CLK_MUX)   // 32MHz PLL output
            | 0 << SYSCON_POS_AHB_DIVIDER;                  // AHB Divider = 0, set AHB clk to 16M
        adc_clk_div = 0x04;
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, mask, reg);
    
    // power on all related modules
    mask = SYSCON_MASK_DIS_REF_PLL
         | SYSCON_MASK_DIS_LO_VCO
         | SYSCON_MASK_DIS_LO_PLL
         | SYSCON_MASK_DIS_PA
         | SYSCON_MASK_DIS_LNA
         | SYSCON_MASK_DIS_LNA_PKDET
         | SYSCON_MASK_DIS_MIXER
         | SYSCON_MASK_DIS_PPF_PKDET
         | SYSCON_MASK_DIS_PPF
         | SYSCON_MASK_DIS_RX_PKDET
         | SYSCON_MASK_DIS_RX_ADC
         | SYSCON_MASK_DIS_RCO
         | SYSCON_MASK_DIS_SAR_ADC  // power on ADC module, for PA CAL
         | SYSCON_MASK_DIS_SAR_BUF
         ;

    // Switch on REF PLL/LO_VCO/LO PLL/PA/LNA/LNA PKDET/MIXER... power
    syscon_SetPGCR1WithMask(QN_SYSCON, mask, MASK_DISABLE);
    // PLL ready
    while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));

    delay(200);

    // enable ADC module clock
    adc_clock_on();

    // set ADC work clock, should be 1MHz
    mask = SYSCON_MASK_ADC_CLK_SEL
         | SYSCON_MASK_ADC_DIV_BYPASS
         | SYSCON_MASK_ADC_DIV;

    reg = (CLK_HIGH << SYSCON_POS_ADC_CLK_SEL)
        | adc_clk_div;

    syscon_SetADCCRWithMask(QN_SYSCON, mask, reg);


    // config adc for PA
    adc_adc_SetADC0(QN_ADC, 0x9906C200);
    adc_adc_SetADC1(QN_ADC, 0x0005AA00);

    // Enable channel change calibration trigger by hardware
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_CH_CHG_CAL_EN, MASK_DISABLE);

    // Select SAR ADC in KVCO calibration, default using sigma-delta ADC.
    //cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_EN_KCAL_SD, MASK_DISABLE);
    
#if CONFIG_CALIB_ENABLE_INTERRUPT==TRUE
    /* Enable the calibration Interrupt */
    NVIC_EnableIRQ(CALIB_IRQn);

    // enable calibration interrupt
    mask = CALIB_MASK_CAL_IE
         | CALIB_MASK_REF_DONE_IE
         | CALIB_MASK_RC_DONE_IE
         | CALIB_MASK_LO_DONE_IE
         | CALIB_MASK_KVCO_DONE_IE
         | CALIB_MASK_PA_DONE_IE
         | CALIB_MASK_R_DONE_IE
         | CALIB_MASK_ROS_DONE_IE
         | CALIB_MASK_RCO_DONE_IE;
    reg  = CALIB_MASK_CAL_IE
         | CALIB_MASK_REF_DONE_IE
         | CALIB_MASK_RC_DONE_IE
         | CALIB_MASK_LO_DONE_IE
         | CALIB_MASK_KVCO_DONE_IE
         | CALIB_MASK_PA_DONE_IE
         | CALIB_MASK_R_DONE_IE
         | CALIB_MASK_ROS_DONE_IE
         | CALIB_MASK_RCO_DONE_IE;
    cal_cal_SetCRWithMask(QN_CALIB, mask, reg);
#endif
}

#if CALIB_CALLBACK_EN==TRUE
void calibration_cb_register(void (*callback[8])(void))
{
    uint8_t i;
    for(i=0; i<8; i++)
    {
        calibration_env.calibration_callback[i] = callback[i];
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Datapath sequence calibration
 *****************************************************************************************
 */
void seq_calibration(uint32_t pa_cal_en)
{
    // enable RF TX repetitious
    rf_enable(RF_TX, MASK_ENABLE, MASK_ENABLE, 0x1F);
    // set freq to 2448MHz <--> idx21
    rf_set_freq(RF_TX, 21);

    // Enable/Disable pa calibration
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CAL_EN, pa_cal_en);    
    
    // 0->1 Re-cal the calibration sequence
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_SEQ_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_SEQ_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_RC_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_RC_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[1] != NULL)
    {
        calibration_env.calibration_callback[1]();
    }
#endif

    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_LO_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_LO_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[2] != NULL)
    {
        calibration_env.calibration_callback[2]();
    }
#endif

    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_R_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_R_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[5] != NULL)
    {
        calibration_env.calibration_callback[5]();
    }
#endif

    if (pa_cal_en) {
        while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_PA_DONE_IF));
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_PA_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[4] != NULL)
        {
            calibration_env.calibration_callback[4]();
        }
#endif
    }

    // Disable RF set by SW, enable set by HW
//    rf_enable_sw_set_freq(MASK_DISABLE);
//    rf_enable(RF_TX, MASK_DISABLE, MASK_DISABLE, 0x0);

#endif
}

/**
 ****************************************************************************************
 * @brief LO_PLL, LO_KVCO, ROS calibration, re-cal once channal is changed
 *****************************************************************************************
 */
void freq_hop_calibration(uint32_t lo_cal_skip, uint32_t lo_kcal_skip)
{
    uint32_t reg;

    // skip bits
    reg = (lo_cal_skip << CALIB_POS_LO_CAL_SKIP) | (lo_kcal_skip << CALIB_POS_LO_KCAL_SKIP);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_LO_CAL_SKIP | CALIB_MASK_LO_KCAL_SKIP, reg);

    // Enable channel change calibration by software request, default trigger by hardware
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_CH_CHG_CAL_EN, MASK_ENABLE);

    // Select SAR ADC in KVCO calibration, default using sigma-delta ADC.
    //cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_EN_KCAL_SD, 0);

    // 0->1 Re-cal the calibration sequence
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_CH_CHG_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_CH_CHG_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    if (lo_cal_skip == 0) {
        while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_LO_DONE_IF));
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_LO_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[2] != NULL)
        {
            calibration_env.calibration_callback[2]();
        }
#endif
    }

    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_ROS_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_ROS_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[6] != NULL)
    {
        calibration_env.calibration_callback[6]();
    }
#endif

    if (lo_kcal_skip == 0) {
        while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_KVCO_DONE_IF));
        cal_cal_ClrSR(QN_CALIB, CALIB_MASK_KVCO_DONE_IF); // clear int flag

#if CALIB_CALLBACK_EN==TRUE
        if(calibration_env.calibration_callback[3] != NULL)
        {
            calibration_env.calibration_callback[3]();
        }
#endif
    }

    // Enable channel change calibration trigger by hardware
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_CH_CHG_CAL_EN, MASK_DISABLE);
#endif

}

/**
 ****************************************************************************************
 * @brief REF_PLL calibration
 *****************************************************************************************
 */
void ref_pll_calibration(void)
{
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE

    // PLL ready
    while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[0] != NULL)
    {
        calibration_env.calibration_callback[0]();
    }
#endif

#endif
}

/**
 ****************************************************************************************
 * @brief RC calibration
 *****************************************************************************************
 */
void rc_calibration(void)
{
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_RC_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_RC_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_RC_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_RC_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[1] != NULL)
    {
        calibration_env.calibration_callback[1]();
    }
#endif

#endif

}

/**
 ****************************************************************************************
 * @brief LO calibration
 *****************************************************************************************
 */
void lo_calibration(void)
{
    // disable skip bits
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_LO_CAL_SKIP, MASK_DISABLE);

    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_LO_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_LO_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[2] != NULL)
    {
        calibration_env.calibration_callback[2]();
    }
#endif

#endif
}

/**
 ****************************************************************************************
 * @brief LO_KCAL calibration
 *****************************************************************************************
 */
void lo_kcal_calibration(void)
{

    // disable skip bits
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_LO_KCAL_SKIP, MASK_DISABLE);

    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_KCAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_KCAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_KVCO_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_KVCO_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[3] != NULL)
    {
        calibration_env.calibration_callback[3]();
    }
#endif

#endif
}

/**
 ****************************************************************************************
 * @brief PA calibration
 *****************************************************************************************
 */
void pa_calibration(void)
{
    // mask all int
    __set_PRIMASK(1);

    // Enable pa calibration
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CAL_EN, MASK_ENABLE);

    // enable RF TX
    rf_enable(RF_TX, MASK_ENABLE, MASK_ENABLE, 0x1F);

    // 1. set freq to 2448MHz <--> idx21
    rf_set_freq(RF_TX, 21);
    
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition
    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_PA_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_PA_CAL_REQ, MASK_ENABLE);

    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_PA_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_PA_DONE_IF);  // clear int flag

    uint32_t pa_code;
    pa_code = (cal_cal_GetCAL3(QN_CALIB) & CALIB_MASK_PA_CAL) >> CALIB_POS_PA_CAL;
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CODE_TX, pa_code<< CALIB_POS_PA_CODE_TX);

    // 2. config PA_CODE_RX to a fixed value
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CODE_RX, 0);

    // disable pa calibration
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CAL_EN, MASK_DISABLE);
    // power off ADC module
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_SAR_ADC|SYSCON_MASK_DIS_SAR_BUF, MASK_ENABLE);

    syscon_SetAdditionCRWithMask(QN_SYSCON, SYSCON_MASK_PA_CKEN_SEL, MASK_ENABLE);
    
#if CALIB_CALLBACK_EN==TRUE
    if (calibration_env.calibration_callback[4] != NULL)
    {
        calibration_env.calibration_callback[4]();
    }
#endif

    // Disable RF set by SW, enable set by HW
//    rf_enable_sw_set_freq(MASK_DISABLE);
//    rf_enable(RF_TX, MASK_DISABLE, MASK_DISABLE, 0x1F);

    // clear pending int
    NVIC_ClearPendingIRQ(CALIB_IRQn);

    // un-mask all int
    __set_PRIMASK(0);
}

/**
 ****************************************************************************************
 * @brief R calibration
 *****************************************************************************************
 */
void r_calibration(void)
{
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_R_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_R_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_R_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_R_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[5] != NULL)
    {
        calibration_env.calibration_callback[5]();
    }
#endif

#endif
}

/**
 ****************************************************************************************
 * @brief ROS calibration
 *****************************************************************************************
 */
void ros_calibration(void)
{
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_ROS_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL3WithMask(QN_CALIB, CALIB_MASK_ROS_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_ROS_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_ROS_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[6] != NULL)
    {
        calibration_env.calibration_callback[6]();
    }
#endif

#endif
}

/**
 ****************************************************************************************
 * @brief RCO calibration
 *****************************************************************************************
 */
void rco_calibration(void)
{
    // 1: Reset the calibration
    // 0: Re-calibrate at 0->1 transition

    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_RCO_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_RCO_CAL_REQ, MASK_ENABLE);

#if CONFIG_CALIB_ENABLE_INTERRUPT==FALSE
    while (!(cal_cal_GetSR(QN_CALIB) & CALIB_MASK_RCO_DONE_IF));
    cal_cal_ClrSR(QN_CALIB, CALIB_MASK_RCO_DONE_IF);  // clear int flag

#if CALIB_CALLBACK_EN==TRUE
    if(calibration_env.calibration_callback[7] != NULL)
    {
        calibration_env.calibration_callback[7]();
    }
#endif

#endif
}

#endif /* CONFIG_ENABLE_DRIVER_CALIB==TRUE */
/// @} CALIB
