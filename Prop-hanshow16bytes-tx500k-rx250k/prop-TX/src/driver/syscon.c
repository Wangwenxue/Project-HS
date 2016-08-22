/**
 ****************************************************************************************
 *
 * @file syscon.c
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

#include "syscon.h"
#if CONFIG_ENABLE_DRIVER_SYSCON==TRUE
#ifdef BLE_PRJ
#include "lib.h"
#endif
#include "timer.h"
#include "rtc.h"

uint32_t g_SystemClock = __SYSTEM_CLOCK;
uint32_t g_AhbClock = __AHB_CLK;
uint32_t g_ApbClock = __APB_CLK;

/**
 ****************************************************************************************
 * @addtogroup SYSTEM_CONTROLLER
 * @{
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  set system clock source
 * @param[in]    clk_src      System clock source
 * @param[in]    flag         Indicating XTAL is 16MHz or 32MHz, or 32KHz is form XTAL32 or RCO32
 * @return 
 * @description
 *  This function is used to set system clock source.
 *****************************************************************************************
 */ 
void syscon_set_sysclk_src(enum CLK_MUX clk_src, int flag)
{
    uint32_t value;
    switch(clk_src) {
    case CLK_XTAL:
        if ((flag == XTAL_32M) || (flag == XTAL_32MHz)) {
            // external clock is 32MHz
            value = MASK_ENABLE;
            g_SystemClock = XTAL_32MHz;
        }
        else {
            value = MASK_DISABLE;
            g_SystemClock = XTAL_16MHz;
        }
        syscon_SetLO1WithMask(QN_SYSCON, SYSCON_MASK_XDIV, value);
        
        // external 16M/32M clk ready
        while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY));
        // High frequency XTAL
        value = CLK_XTAL<<SYSCON_POS_CLK_MUX;
        break;
        
    case CLK_INT_20M:
        g_SystemClock = SYS_INT_20M;
        value = CLK_INT_20M<<SYSCON_POS_CLK_MUX;
        break;
        
    case CLK_INT_32M:
        g_SystemClock = SYS_PLL_32M;
        value = (uint32_t)CLK_INT_32M<<SYSCON_POS_CLK_MUX;
        break;
        
    case CLK_LOW_32K:
        clk32k_enable(flag);
        // 32K low speed clock
        g_SystemClock = SYS_LOW_32K;
        value = (uint32_t)CLK_LOW_32K<<SYSCON_POS_CLK_MUX;
        break;       
    default:
        break;
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, value);        
}

/**
 ****************************************************************************************
 * @brief  Set AHB clock
 * @param[in]    clk        AHB clock frequency
 * @return 
 * @description
 *  This function is used to set AHB clock.
 *****************************************************************************************
 */ 
void syscon_set_ahb_clk(int clk)
{
    int div = AHB_CLK_DIV(clk);
    uint32_t mask = 0, value = 0;

    if (div >= 0) {
        g_AhbClock = clk;
        mask = SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_AHB_DIVIDER;
        value = div<<SYSCON_POS_AHB_DIVIDER;
    }
    else {
        // bypass
        g_AhbClock = g_SystemClock;
        mask = SYSCON_MASK_AHB_DIV_BYPASS;
        value = MASK_ENABLE;
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, mask, value);
}

/**
 ****************************************************************************************
 * @brief  Get AHB clock
 * @description
 *  This function is used to get AHB clock.
 *****************************************************************************************
 */ 
void syscon_get_ahb_clk(void)
{
    uint32_t ahb_div;
    ahb_div = syscon_GetCMDCR(QN_SYSCON);
    if (ahb_div & SYSCON_MASK_AHB_DIV_BYPASS) {
        g_AhbClock = g_SystemClock;
    }
    else {
        ahb_div = (ahb_div & SYSCON_MASK_AHB_DIVIDER) >> SYSCON_POS_AHB_DIVIDER;
        g_AhbClock = g_SystemClock / (2*(ahb_div+1));
    }
}

/**
 ****************************************************************************************
 * @brief  Set APB clock
 * @param[in]    clk        APB clock frequency
 * @return 
 * @description
 *  This function is used to set APB clock.
 *****************************************************************************************
 */ 
void syscon_set_apb_clk(int clk)
{
    int div = APB_CLK_DIV(clk);
    uint32_t mask = 0, value = 0;

    if (div >= 0) {
        g_ApbClock = clk;
        mask = SYSCON_MASK_APB_DIV_BYPASS|SYSCON_MASK_APB_DIVIDER;
        value = div<<SYSCON_POS_APB_DIVIDER;
    }
    else {
        // bypass
        g_ApbClock = g_AhbClock;
        mask = SYSCON_MASK_APB_DIV_BYPASS;
        value = MASK_ENABLE;
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, mask, value);
}

/**
 ****************************************************************************************
 * @brief  Get APB clock
 * @description
 *  This function is used to get APB clock.
 *****************************************************************************************
 */ 
void syscon_get_apb_clk(void)
{
    uint32_t apb_div;
    apb_div = syscon_GetCMDCR(QN_SYSCON);
    if (apb_div & SYSCON_MASK_APB_DIV_BYPASS) {
        g_ApbClock = g_AhbClock;
    }
    else {
        apb_div = (apb_div & SYSCON_MASK_APB_DIVIDER) >> SYSCON_POS_APB_DIVIDER;
        g_ApbClock = g_AhbClock / (2*(apb_div+1));
    }
}

/**
 ****************************************************************************************
 * @brief  Set TIMER clock
 * @param[in]    clk        TIMER clock frequency
 * @return 
 * @description
 *  This function is used to set TIMER clock.
 *****************************************************************************************
 */
void syscon_set_timer_clk(int clk)
{
    int div = TIMER_CLK_DIV(clk);
    uint32_t mask = 0, value = 0;
    
    if (div >= 0) {
        mask = SYSCON_MASK_TIMER_DIV_BYPASS|SYSCON_MASK_TIMER_DIVIDER;
        value = div<<SYSCON_POS_TIMER_DIVIDER;
    }
    else {
        // bypass
        mask = SYSCON_MASK_TIMER_DIV_BYPASS;
        value = MASK_ENABLE;
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, mask, value);
}

/**
 ****************************************************************************************
 * @brief  Set USART clock
 * @param[in]    usart      QN_UART0 / QN_UART1 / QN_SPI0 / QN_SPI1
 * @param[in]    clk        USART clock frequency
 * @return 
 * @description
 *  This function is used to set USART clock.
 *****************************************************************************************
 */ 
void syscon_set_usart_clk(uint32_t usart, int clk)
{
    int div = USARTx_CLK_DIV(clk);
    uint32_t mask = 0, value = 0;
    
    if ((usart == (uint32_t)QN_UART0) || (usart == (uint32_t)QN_SPI0)) {
        if (div >= 0) {
            mask = SYSCON_MASK_USART0_DIV_BYPASS|SYSCON_MASK_USART0_DIVIDER;
            value = div<<SYSCON_POS_USART0_DIVIDER;
        }
        else {
            // bypass
            mask = SYSCON_MASK_USART0_DIV_BYPASS;
            value = MASK_ENABLE;
        }
    }
    else if ((usart == (uint32_t)QN_UART1) || (usart == (uint32_t)QN_SPI1)) {
        if (div >= 0) {
            mask = SYSCON_MASK_USART1_DIV_BYPASS|SYSCON_MASK_USART1_DIVIDER;
            value = div<<SYSCON_POS_USART1_DIVIDER;
        }
        else {
            // bypass
            mask = SYSCON_MASK_USART1_DIV_BYPASS;
            value = MASK_ENABLE;
        }
    }
    syscon_SetCMDCRWithMask(QN_SYSCON, mask, value);
}

/**
 ****************************************************************************************
 * @brief  Set BLE clock
 * @param[in]    clk        BLE clock frequency: only support 8M, 16M
 * @return 
 * @description
 *  This function is used to set BLE clock.
 *****************************************************************************************
 */ 
void syscon_set_ble_clk(int clk)
{
    int div = BLE_CLK_DIV(clk);
    uint32_t mask, value;

    if (div >= 0) {
        mask = SYSCON_MASK_BLE_DIV_BYPASS|SYSCON_MASK_BLE_DIVIDER;
        value = div<<SYSCON_POS_BLE_DIVIDER;
    }
    else {
        // bypass
        mask = SYSCON_MASK_BLE_DIV_BYPASS;
        value = SYSCON_MASK_BLE_DIV_BYPASS;
    }

    if (clk == CLK_16M) {
        value |= SYSCON_MASK_BLE_FRQ_SEL;
    }

    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_BLE_FRQ_SEL|mask, value);
}

/**
 ****************************************************************************************
 * @brief  Get reset source
 * @return     enum RESET_CAUSE
 * @description
 *  This function is used to get system reset cause.
 *****************************************************************************************
 */
enum RESET_CAUSE syscon_get_reset_cause(void)
{
    enum RESET_CAUSE ret;
    uint32_t reg;

    reg = syscon_GetRCS(QN_SYSCON) & SYSCON_MASK_RST_CAUSE;
    if (reg & POWER_ON_RST) {
        ret = POWER_ON_RST;
    }
    else if (reg & BROWN_OUT_RST) {
        ret = BROWN_OUT_RST;
    }
    else if (reg & EXT_PIN_RST) {
        ret = EXT_PIN_RST;
    }
    else if (reg & WDT_RST) {
        ret = WDT_RST;
    }
    else if (reg & LOCK_UP_RST) {
        ret = LOCK_UP_RST;
    }
    else if (reg & REBOOT_RST) {
        ret = REBOOT_RST;
    }
    else if (reg & CPU_SYS_RST) {
        ret = CPU_SYS_RST;
    }
    else if (reg & CPU_SOFT_RST) {
        ret = CPU_SOFT_RST;
    }
    else {
        ret = NONE_RST;
    }

    // clear reset cause
    syscon_SetRCS(QN_SYSCON, SYSCON_MASK_RST_CAUSE_CLR);

    return ret;
}

/**
 ****************************************************************************************
 * @brief  Enable or disable transceiver
 * @param[in]    able       MASK_ENABLE or MASK_DISABLE
 * @return
 * @description
 *  This function is used to enable or disable transceiver, contains BLE clock setting and
 *  REF PLL power setting.
 *****************************************************************************************
 */
void syscon_enable_transceiver(uint32_t able)
{
    uint32_t value;
    if (able == MASK_DISABLE) {
        // disable BLE clock
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_BLE_AHB);
        // Switch off REF PLL power
        value = MASK_ENABLE;
    }
    else {
        // enable BLE clock
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_BLE_AHB);
        // Switch on REF PLL power
        value = MASK_DISABLE;
    }
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_REF_PLL, value);
}

#if CLOCK_32K_CORRECTION_EN==TRUE
/**
 ****************************************************************************************
 * @brief Initialize 32K clock correction
 * @return
 * @description
 *  This function is used to initialize 32K clock correction
 *****************************************************************************************
 */
void clock_32k_correction_init(void)
{
    syscon_set_ahb_clk(__AHB_CLK);
    // Bypass Tiemr Divider
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_TIMER_DIV_BYPASS, MASK_ENABLE);
}

/**
 ****************************************************************************************
 * @brief enable 32k clock correction
 * @param[in]    callback       Callback function pointer, which is called in IRQHandler
 * @return
 * @description
 *  This function is used to enable correction of 32K clock
 *****************************************************************************************
 */
void clock_32k_correction_enable(void (*callback)(void))
{
    uint32_t reg;
    
    // Enable Timer1 clock
    timer_init(QN_TIMER1, callback);
    
    // reset Timer1
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_TIMER1_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_TIMER1_RST);

    // set counter event top number fix to 16
    timer_timer_SetTOPR(QN_TIMER1, 16);
    reg = CLK_PSCL                             /* set clock source to prescaler clock */
        | (0 << TIMER_POS_PSCL)                /* set prescaler to zero */
        | TIMER_MASK_ICNCE                     /* enable input capture noise canceller */
        | INCAP_PIN1                           /* set input capture pin to pin1(32k) */
        | INCAP_SRC_PIN                        /* set input capture source to capture PIN */
        | INCAP_EDGE_POS                       /* set input capture edge to pos edge */
        | INCAP_COUNTER_MOD                    /* select input capture counter mode */
#if CONFIG_TIMER1_ENABLE_INTERRUPT==TRUE        
        | TIMER_MASK_ICIE                      /* enable input capure int */
#endif
        | TIMER_MASK_TEN;                      /* enable timer */
    timer_timer_SetCR(QN_TIMER1, reg);

#if ((TIMER1_CALLBACK_EN==TRUE) && (CONFIG_TIMER1_DEFAULT_IRQHANDLER==TRUE))
    // TBC in Timer1 ISR
#else
    while (!(timer_timer_GetIntFlag(QN_TIMER1) & TIMER_MASK_ICF));
    // clear int flag
    timer_timer_ClrIntFlag(QN_TIMER1, TIMER_MASK_ICF);
    // read out counter value
    timer1_env.count = timer_timer_GetCCR(QN_TIMER1);
    
    if (callback != NULL) {
        callback();
    }
#endif
}

/**
 ****************************************************************************************
 * @brief callback function of 32k clock correction
 * @return
 * @description
 *  This function will be called after 32k clock correction is finish
 *****************************************************************************************
 */
void clock_32k_correction_cb(void)
{
    uint32_t dir = 0;
    uint32_t ppm;
    uint32_t tmp;

    // remove warning
    dir = dir;

    // Formula: ppm = (0x100000ull * 32000(Hz) * count) / (refclk_freq(Hz) * ncycle(32K cycle number));
    // In this function, refclk_freq is set to 8MHz, ncycle is set to 16
    //ppm = (0x2000000ull * timer1_env.count) / (8*16 * 1000);
    //ppm = (0x8000 * timer1_env.count) / 125;
#if (__AHB_CLK == CLK_8M)
    tmp = (0x8000 * timer1_env.count);
#elif (__AHB_CLK == CLK_16M) 
    tmp = (0x4000 * timer1_env.count);
#else 
    tmp = (0x2000 * timer1_env.count);
#endif
    // error: 1.3e-9
    ppm = (tmp>>7) + (tmp>>13) + (tmp>>14) + (tmp>>18) + (tmp>>21) + (tmp>>24) + (tmp>>25) + (tmp>>26);
    
    if (ppm > 0x100000) {
        ppm = ppm - 0x100000;
        dir = 0;
    }
    else {
        ppm = 0x100000 - ppm;
        dir = 1;
    }
    if (ppm > 0xFFFF) {
        ppm = 0xFFFF;
    }
    // disable timer
    timer_enable(QN_TIMER1, MASK_DISABLE);
    // Disable Timer1 clock
    timer_clock_off(QN_TIMER1);

#if (CONFIG_ENABLE_DRIVER_RTC == TRUE)
    // write to rtc calibration register
    rtc_calibration(dir, ppm);
#endif

    // TODO: add user code here
#ifdef BLE_PRJ
#if (QN_32K_RCO)
    int32_t real_ppm = (dir)? -ppm : ppm;
    set_32k_ppm(real_ppm);
    dev_allow_sleep(PM_MASK_TIMER1_ACTIVE_BIT);
#endif
#endif
}

/**
 ****************************************************************************************
 * @brief RCO calibration callback function
 *****************************************************************************************
 */
void rco_calibration_cb(void)
{
    clock_32k_correction_enable(clock_32k_correction_cb);
}
#endif

#endif
/// @} SYSTEM_CONTROLLER
