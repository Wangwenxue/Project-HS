/**
 ****************************************************************************************
 *
 * @file sleep.c
 *
 * @brief Sleep driver for QN9020.
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
 * @addtogroup  SLEEP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "sleep.h"
#if CONFIG_ENABLE_DRIVER_SLEEP==TRUE
#include "syscon.h"
#include "intc.h"
#include "lib.h"
#ifdef BLE_PRJ
#include "usr_design.h"
#include "uart.h"
#include "spi.h"
#include "button.h"
#endif
#if ACMP_WAKEUP_EN == TRUE
#include "analog.h"
#endif

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
struct sleep_env_tag sleep_env = {QN_DEEP_SLEEP, 0, true, 0, 0};
volatile uint32_t PGCR1_restore;
volatile uint8_t low_power_mode_en = 0;
volatile uint32_t ahb_clock_flag = 0;

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */
#ifdef BLE_PRJ
/**
 ****************************************************************************************
 * @brief   Check application whether to enter sleep mode
 * @return  sleep allowed status
 ****************************************************************************************
 */
int usr_sleep(void)
{
    int32_t rt;

    rt = sleep_get_pm();

    // If the BLE timer queue is not NULL or BLE event is exist, prevent entering into DEEPSLEEP mode
    if(rt == PM_DEEP_SLEEP && 
       (!ke_timer_empty() || !ble_evt_empty()))
    {
        rt = PM_SLEEP;
    }

    // Check Device status
    if((rt >= PM_SLEEP)
       && dev_get_bf())
    {
        // If any devices are still working, the chip cann't enter into SLEEP/DEEPSLEEP mode.
        rt = PM_IDLE;
    }

    if ((rt >= PM_SLEEP) && (!gpio_sleep_allowed()))
    {
        return PM_ACTIVE;
    }

#if ACMP_WAKEUP_EN == TRUE
    if ((rt >= PM_SLEEP) && (!acmp_sleep_allowed()))
    {
        return PM_ACTIVE;
    }
#endif
    
    
#if QN_DBG_PRINT
    int uart_tx_st = uart_check_tx_free(QN_DEBUG_UART);
    
    if((rt >= PM_SLEEP) && (uart_tx_st == UART_TX_BUF_BUSY))
    {
        rt = PM_IDLE;
    }
    else if(uart_tx_st == UART_LAST_BYTE_ONGOING)
    {
        return PM_ACTIVE;    // If CLOCK OFF & POWER DOWN is disabled, return immediately
    }
#endif

#if QN_EACI
    if ((rt >= PM_SLEEP) &&
        (  (eaci_env.tx_state!=EACI_STATE_TX_IDLE)              // Check EACI UART TX status
        || (eaci_env.rx_state!=EACI_STATE_RX_START)) )          // Check EACI UART RX status
    {
        rt = PM_IDLE;
    }

    int tx_st = 0;
    #if (defined(CFG_HCI_UART))
    tx_st = uart_check_tx_free(QN_HCI_PORT);
    if ((rt >= PM_SLEEP) && (tx_st == UART_TX_BUF_BUSY))
    {
        rt = PM_IDLE;
    }
    else if (tx_st == UART_LAST_BYTE_ONGOING)
    {
        return PM_ACTIVE;    // If CLOCK OFF & POWER DOWN is disabled, return immediately
    }
    #elif (defined(CFG_HCI_SPI))
    tx_st = spi_check_tx_free(QN_HCI_PORT);
    if ((rt >= PM_SLEEP) && (tx_st == SPI_TX_BUF_BUSY))
    {
        rt = PM_IDLE;
    }
    else if (tx_st == SPI_LAST_BYTE_ONGOING)
    {
        return PM_ACTIVE;    // If CLOCK OFF & POWER DOWN is disabled, return immediately
    }
    #endif

#endif
    
#if !(QN_32K_RCO)
    // wait for 32k xtal ready
    if(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_XTAL32_RDY)
    {
        // disable schmitt trigger in 32.768KHz buffer
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_DISABLE);
        // Set 32.768KHz xtal to normal current
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 16);
    }
    else if(rt > PM_ACTIVE)
    {
        rt = PM_ACTIVE;
    }
#endif

    return rt;
}
#endif

/**
 ****************************************************************************************
 * @brief  Init sleep power down modules
 * @description
 *  This function is used to init MCU sleep mode.
 *****************************************************************************************
 */
void sleep_init(void)
{
    // --------------------------------------------
    // sleep
    // --------------------------------------------
    
    //23 : PD_XTAL32
    //10 : PD_RCO
    // 7 : PD_MEM7
    // 6 : PD_MEM6
    // 5 : PD_MEM5
    // 4 : PD_MEM4
    // 3 : PD_MEM3
    // 2 : PD_MEM2
    // 1 : PD_MEM1
    // 0 : PL_VREG_D
    sleep_env.retention_modules |= QN_MEM_RETENTION;

    // power down all module in sleep except retention modules
    syscon_SetPGCR0WithMask(QN_SYSCON, 0xF7FFFCFE, (0xFFFFFC00 | QN_MEM_UNRETENTION));

    // power down all unretention memory all the time.
    // if you want to use the unretention memory in the active mode, remove the following snippet.
    syscon_SetPGCR1WithMask(QN_SYSCON, 
                            (SYSCON_MASK_DIS_MEM1
                            | SYSCON_MASK_DIS_MEM2
                            | SYSCON_MASK_DIS_MEM3
                            | SYSCON_MASK_DIS_MEM4
                            | SYSCON_MASK_DIS_MEM5
                            | SYSCON_MASK_DIS_MEM6
                            | SYSCON_MASK_DIS_MEM7),
                            QN_MEM_UNRETENTION);
}

/**
 ****************************************************************************************
 * @brief  Enable sleep mode
 * @param[in]    mode           sleep mode
 * @param[in]    iconfig        wakeup interrupt config
 * @param[in]    callback       callback after wakeup
 * @description
 *  This function is used to set MCU into sleep mode, before enter sleep, wakeup source should be set.
 *****************************************************************************************
 */
void enter_sleep(enum SLEEP_MODE mode, uint32_t iconfig, void (*callback)(void))
{
    if (mode == SLEEP_CPU_CLK_OFF) {
        // --------------------------------------------
        // cpu clock disable
        // --------------------------------------------

        // Ensure we use deep SLEEP - SLEEPDEEP should be set
        // SCR[2] = SLEEPDEEP
        SCB->SCR |= (1UL << 2);
        // set pd state to deep gating
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_PMUENABLE, MASK_DISABLE);

        GLOBAL_INT_DISABLE();
#if SLEEP_CONFIG_EN == TRUE
        NVIC->ISER[0] = iconfig;
#endif
        // Wait For Interrupt
        __WFI();  // Enter sleep mode

        // Wakeup when interrupt is triggered
        GLOBAL_INT_RESTORE();

        // TODO
    }
    else if (mode == SLEEP_NORMAL) {
        
#if QN_32K_LOW_POWER_MODE_EN==TRUE

        // Ensure we use SLEEP
        SCB->SCR &= ~(1UL << 2);

        enter_low_power_mode(0);

#else
        // --------------------------------------------
        // sleep 
        // --------------------------------------------

#ifdef BLE_PRJ
        // Save configuration before power down
        save_ble_setting();
#endif
       
        // switch to internal 20MHz
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_INT_20M<<SYSCON_POS_CLK_MUX);
        
        // power down all module in sleep except 32K and retention memory
        syscon_SetPGCR0WithMask(QN_SYSCON, sleep_env.retention_modules|0x00000001, 0x00000001);
        
#if (defined(QN_EXT_FLASH))
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
        sf_ctrl_SetCRWithMask(QN_SF_CTRL, SF_CTRL_MASK_BOOT_DONE, MASK_ENABLE);
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
        syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_BOND_EN, MASK_DISABLE);
#endif

        // Ensure we use deep SLEEP - SLEEPDEEP should be set
        // SCR[2] = SLEEPDEEP
        SCB->SCR |= (1UL << 2);
        // set pd state to sleep
#if !QN_PMU_VOLTAGE
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_DVDD12_PMU_SET|SYSCON_MASK_PMUENABLE, MASK_ENABLE);
#else
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_PMUENABLE, MASK_ENABLE);
#endif

        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A|SYSCON_MASK_VREG12_D|SYSCON_MASK_DVDD12_SW_EN, 
                                              (0x0 << SYSCON_POS_VREG12_A)|(0x0 << SYSCON_POS_VREG12_D));
                                              
        // Reduce HCLK frequency
        ahb_clock_flag = syscon_GetCMDCR(QN_SYSCON) | 0x1;
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_AHB_DIVIDER, 
                                              (31<<SYSCON_POS_AHB_DIVIDER));
#endif // QN_32K_LOW_POWER_MODE_EN==TRUE

#if SLEEP_CONFIG_EN == TRUE
        NVIC->ICPR[0] = 0x00000020;  // clear OSC_EN pending flag
        NVIC->ISER[0] = iconfig;
#endif
        // Wait For Interrupt
        __WFI();  // Enter sleep mode
        // Wakeup when sleep timer, comparator or gpio is triggered

        // Disable interrupt in the wakeup procedure.
        NVIC->ICER[0] = iconfig;
        
#if (defined(QN_EXT_FLASH))
        syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_BOND_EN, MASK_ENABLE);
#endif

#if QN_32K_LOW_POWER_MODE_EN==TRUE
#ifdef BLE_PRJ
        restore_from_low_power_mode(NULL);
#else
        restore_from_low_power_mode(callback);
#endif
#else

        // 1.2V
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A|SYSCON_MASK_VREG12_D|SYSCON_MASK_DVDD12_SW_EN, 
                                              (0x1 << SYSCON_POS_VREG12_A)|(0x0 << SYSCON_POS_VREG12_D)|SYSCON_MASK_DVDD12_SW_EN);

        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_DVDD12_PMU_SET, MASK_DISABLE);

#if SLEEP_CALLBACK_EN == TRUE
        if (callback != NULL) {
            callback();
        }
#endif

        // 16MHz/32MHz XTAL is ready
        while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY))
        {
            // XTAL shall be ready before BLE wakeup
            if(check_ble_wakeup())
            {
                // In this case XTAL wakeup duration is larger than setting.
                // The parameter 'Oscillator wake-up time' in the NVDS should be revised.
#if (QN_DBG_INFO)
                set_dbg_info(QN_DBG_INFO_XTAL_WAKEUP_DURATION);
#endif
            }
        }
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_XTAL<<SYSCON_POS_CLK_MUX);

#endif // QN_32K_LOW_POWER_MODE_EN==TRUE

#if (defined(BLE_PRJ))
        sleep_post_process();
#endif
    }
#if (QN_DEEP_SLEEP_EN)
    else if (mode == SLEEP_DEEP) {
        // --------------------------------------------
        // deep sleep
        // --------------------------------------------

#ifdef BLE_PRJ
        // Save configuration before power down
        save_ble_setting();
#endif
     
        // switch to internal 20MHz
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_INT_20M<<SYSCON_POS_CLK_MUX);

        sleep_env.deep_sleep = true;
        // power down all module in deep sleep except retention memory
        syscon_SetPGCR0WithMask(QN_SYSCON, 0xF7FFFCFF, 0xFFFFFC01|~sleep_env.retention_modules);

#if (defined(QN_EXT_FLASH))
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
        sf_ctrl_SetCRWithMask(QN_SF_CTRL, SF_CTRL_MASK_BOOT_DONE, MASK_ENABLE);
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
        syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_BOND_EN, MASK_DISABLE);
#endif

        // Ensure we use deep SLEEP - SLEEPDEEP should be set
        // SCR[2] = SLEEPDEEP
        SCB->SCR |= (1UL << 2);
        // set pd state to sleep
#if !QN_PMU_VOLTAGE
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_DVDD12_PMU_SET|SYSCON_MASK_PMUENABLE, MASK_ENABLE);
#else
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_PMUENABLE, MASK_ENABLE);
#endif

        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A|SYSCON_MASK_VREG12_D|SYSCON_MASK_DVDD12_SW_EN, 
                                              (0x0 << SYSCON_POS_VREG12_A)|(0x0 << SYSCON_POS_VREG12_D));

        // Reduce HCLK frequency
        ahb_clock_flag = syscon_GetCMDCR(QN_SYSCON) | 0x1;
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_AHB_DIVIDER, 
                                              (31<<SYSCON_POS_AHB_DIVIDER));

#if SLEEP_CONFIG_EN == TRUE
        NVIC->ICPR[0] = 0x00000020;  // clear OSC_EN pending flag
        NVIC->ISER[0] = iconfig;
#endif
        // Wait For Interrupt
        __WFI();  // Enter sleep mode
        // Wakeup when sleep timer, comparator or gpio is triggered

        // Disable interrupt in the wakeup procedure.
        NVIC->ICER[0] = iconfig;
        
#if (defined(QN_EXT_FLASH))
        syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_BOND_EN, MASK_ENABLE);
#endif

        // 1.2V
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A|SYSCON_MASK_VREG12_D|SYSCON_MASK_DVDD12_SW_EN, 
                                              (0x1 << SYSCON_POS_VREG12_A)|(0x0 << SYSCON_POS_VREG12_D)|SYSCON_MASK_DVDD12_SW_EN);

        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_DVDD12_PMU_SET, MASK_DISABLE);

#if SLEEP_CALLBACK_EN == TRUE
        if (callback != NULL) {
            callback();
        }
#endif

        // 16MHz/32MHz XTAL is ready
        while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY))
        {
            // XTAL shall be ready before BLE wakeup
            if(check_ble_wakeup())
            {
                // In this case XTAL wakeup duration is larger than setting.
                // The parameter 'Oscillator wake-up time' in the NVDS should be revised.
#if (QN_DBG_INFO)
                set_dbg_info(QN_DBG_INFO_XTAL_WAKEUP_DURATION);
#endif
            }
        }
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_XTAL<<SYSCON_POS_CLK_MUX);

#if (defined(BLE_PRJ))
        sleep_post_process();
#endif
    }
#endif // QN_DEEP_SLEEP_EN

}

#if GPIO_WAKEUP_EN == TRUE
/**
 ****************************************************************************************
 * @brief  Set GPIO wakeup
 * @param[in]    pin         wakeup pin: P0 and P1
 * @param[in]    type        Wakeup type: high, low, change
 * @description
 *  This function is used to set MCU wakeup by gpio pin.
 *****************************************************************************************
 */
void wakeup_by_gpio(enum gpio_pin pin, enum gpio_wakeup_type type)
{
    if (sleep_env.wakeup_by_sleeptimer == 0) {
        // Disable sleep timer wakeup
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_OSC_WAKEUP_EN, MASK_DISABLE);
    }
    
    // configure gpio wakeup pin
    gpio_wakeup_config(pin, type);
    gpio_enable_interrupt(pin);

    // Ensure gpio interrupt is not pending before the test
    NVIC_ClearPendingIRQ(GPIO_IRQn);
    // Enable Interrupts
    NVIC_EnableIRQ(GPIO_IRQn);
}
#endif

#if ACMP_WAKEUP_EN == TRUE
/**
 ****************************************************************************************
 * @brief  Set analog comparator wakeup
 * @param[in]    acmpch         enum ACMP_CH
 * @param[in]    callback       Callback function pointer, which is called in IRQHandler.
 * @description
 *  This function is used to set MCU wakeup by analog comparator.
 *****************************************************************************************
 */
void wakeup_by_analog_comparator(enum ACMP_CH acmpch, void (*callback)(void))
{
    if (sleep_env.wakeup_by_sleeptimer == 0) {
        // Disable sleep timer wakeup
        syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_OSC_WAKEUP_EN, MASK_DISABLE);
    }
    
    if (acmpch == ACMP0) {
        acmp_pin_enable(ACMP0_PIN_P, MASK_ENABLE);
    }
    else {
        acmp_pin_enable(ACMP1_PIN_P, MASK_ENABLE);
    }
    acmp_init(acmpch, VDD_8, ACMPO_0_GEN_INT, HYST_ENABLE, callback);
}
#endif

#if SLEEP_TIMER_WAKEUP_EN == TRUE
/**
 ****************************************************************************************
 * @brief  Set sleep timer wakeup
 * @param[in]    clk_src      32KHz clock source
 * @description
 *  This function is used to set MCU wakeup by sleep timer.
 *****************************************************************************************
 */
void wakeup_by_sleep_timer(int clk_src)
{
    uint32_t dis_type, en_type;
    uint32_t pd_type;
    
    // Enable sleep timer wakeup 
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_OSC_WAKEUP_EN, MASK_ENABLE);
    sleep_env.wakeup_by_sleeptimer = 1;

    if (clk_src == RCO_32K) {
        // Enable 32k RCO
        dis_type = SYSCON_MASK_DIS_RCO;
        en_type = MASK_ENABLE;
        pd_type = SYSCON_MASK_PD_XTAL32;
        sleep_env.retention_modules |= SYSCON_MASK_DIS_RCO;
    }
    else {
        // Enable 32k XTAL
        dis_type = SYSCON_MASK_DIS_XTAL32;
        en_type = MASK_DISABLE;
        pd_type = SYSCON_MASK_PD_RCO;
        sleep_env.retention_modules |= SYSCON_MASK_DIS_XTAL32;
    }
    syscon_SetPGCR1WithMask(QN_SYSCON, dis_type, MASK_DISABLE);
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, en_type);
    syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_PD_XTAL32|SYSCON_MASK_PD_RCO, pd_type);
}
#endif

#if SLEEP_CALLBACK_EN == TRUE
/**
 ****************************************************************************************
 * @brief  Sleep wakeup callback function
 * @description
 *  This function will be called before clock switching to XTAL in sleep mode.
 *****************************************************************************************
 */
void sleep_cb(void)
{
#if __XTAL == XTAL_32MHz
    // external clock is 32MHz
    syscon_SetLO1WithMask(QN_SYSCON, SYSCON_MASK_XDIV, MASK_ENABLE);
#endif

#ifdef BLE_PRJ    
    // Restore configuration
    restore_ble_setting();

    // Restore peripheral used in the application
    usr_sleep_restore();
#endif    
    
    // clear pending interrupt, except sleep mode wakeup source: analog comparator0/1, gpio, osc_en
    NVIC->ICPR[0] = 0xffffffd8;
}
#endif

#ifdef BLE_PRJ
#if (QN_DEEP_SLEEP_EN && !QN_32K_RCO)
/**
 ****************************************************************************************
 * @brief  Wakeup 32k XTAL
 * @description
 *  This function will be called after waking up from deep sleep mode.
 *****************************************************************************************
 */
void wakeup_32k_xtal_switch_clk(void)
{
    //if(sleep_env.deep_sleep)
    {
        // prevent ble sleep
        enable_ble_sleep(false);

        // ensable schmitt trigger in 32.768KHz buffer
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_ENABLE);
        // Set 32.768KHz xtal higher current, that can let the 32k xtal stable fastly (decrease stable time).
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 32);
        
        // switch to 32k RCO
        syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RCO, MASK_DISABLE);
#if (QN_PMU_VOLTAGE)
        delay(10);
#else
        delay(300);
#endif
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, MASK_ENABLE);
    }
}

/**
 ****************************************************************************************
 * @brief  Wakeup 32k XTAL
 * @description
 *  This function will be called after waking up from deep sleep mode.
 *****************************************************************************************
 */
void wakeup_32k_xtal_start_timer(void)
{
    //if(sleep_env.deep_sleep)
    {
        // start 32k XTAL wakeup timer
        ke_timer_set(APP_SYS_32K_XTAL_WAKEUP_TIMER, TASK_APP, 50);
    }
}
#endif
#endif

/**
 ****************************************************************************************
 * @brief  Enter low power mode
 * @param[in]    en       enabled peripheral at low power mode
 * @description
 *  This function is used to set MCU entering into low power mode.
 *****************************************************************************************
 */
void enter_low_power_mode(uint32_t en)
{
    uint32_t reg_pgcr1;
    uint32_t mask;

    PGCR1_restore = syscon_GetPGCR1(QN_SYSCON);
    reg_pgcr1 = PGCR1_restore;

    // bypass AHB divider, prepare for clock switch to 32k
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_APB_DIV_BYPASS, MASK_ENABLE);
 
    syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_PL_VREG_D, MASK_DISABLE);
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_PMUENABLE, MASK_DISABLE);
    
    // power off all not needed modules
    mask = SYSCON_MASK_DIS_OSC
         | SYSCON_MASK_DIS_BG
         | SYSCON_MASK_DIS_V2I
         | SYSCON_MASK_DIS_BUCK
         | SYSCON_MASK_DIS_VREG_A
         | SYSCON_MASK_DIS_VREG_D
         | SYSCON_MASK_DIS_XTAL
#if QN_32K_RCO == TRUE
         | SYSCON_MASK_DIS_XTAL32
#endif
         | SYSCON_MASK_DIS_REF_PLL
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
         | SYSCON_MASK_DIS_SAR_ADC
#if (QN_32K_RCO == FALSE)
         | SYSCON_MASK_DIS_RCO
#endif
         | SYSCON_MASK_DIS_MEM7
         | SYSCON_MASK_DIS_MEM6
         | SYSCON_MASK_DIS_MEM5
         | SYSCON_MASK_DIS_MEM4
         | SYSCON_MASK_DIS_MEM3
         | SYSCON_MASK_DIS_MEM2
         | SYSCON_MASK_DIS_MEM1
         | SYSCON_MASK_DIS_SAR_BUF
         ;

    reg_pgcr1 |= (mask & (~QN_MEM_RETENTION));

    // set 32k low power flag
    low_power_mode_en = 1;

    // set system clock to 32K
    syscon_set_sysclk_src(CLK_LOW_32K, __32K_TYPE);

    // disable power of unused block in 32k mode
    syscon_SetPGCR1(QN_SYSCON, reg_pgcr1);

#if 0
    // gating all not needed modules
    mask = SYSCON_MASK_GATING_TIMER3
         | SYSCON_MASK_GATING_TIMER2
#if QN_32K_RCO == FALSE
         | SYSCON_MASK_GATING_TIMER1
#endif
         | SYSCON_MASK_GATING_TIMER0
         | SYSCON_MASK_GATING_UART1
         | SYSCON_MASK_GATING_UART0
         | SYSCON_MASK_GATING_SPI1
         | SYSCON_MASK_GATING_SPI0
         //| SYSCON_MASK_GATING_32K_CLK
         | SYSCON_MASK_GATING_SPI_AHB
         //| SYSCON_MASK_GATING_GPIO
         | SYSCON_MASK_GATING_ADC
         | SYSCON_MASK_GATING_DMA
         //| SYSCON_MASK_GATING_BLE_AHB
         | SYSCON_MASK_GATING_PWM
         ;
    syscon_SetCRSS(QN_SYSCON, mask&(~en));
#endif
}

/**
 ****************************************************************************************
 * @brief  Restore from low power mode
 * @param[in]    callback       callback before XTAL clock ready
 * @description
 *  This function is used to set MCU restoring from low power mode, switch system clock to XTAL.
 *****************************************************************************************
 */
void restore_from_low_power_mode(void (*callback)(void))
{
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_PD_STATE|SYSCON_MASK_DVDD12_PMU_SET, MASK_DISABLE);
    if (callback != NULL) {
        callback();
    }
    
    // 16MHz/32MHz XTAL is ready
    while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY))
    {
        // XTAL shall be ready before BLE wakeup
        if (check_ble_wakeup())
        {
            // In this case XTAL wakeup duration is larger than setting.
            // The parameter 'Oscillator wake-up time' in the NVDS should be revised.
#if (QN_DBG_INFO)
            set_dbg_info(QN_DBG_INFO_XTAL_WAKEUP_DURATION);
#endif
        }
    }
    //syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_XTAL<<SYSCON_POS_CLK_MUX);
    syscon_set_sysclk_src(CLK_XTAL, __XTAL);
    syscon_set_ahb_clk(__AHB_CLK);
    syscon_set_apb_clk(__APB_CLK);
}

#ifdef BLE_PRJ
/**
 ****************************************************************************************
 * @brief OSC interrupt handler, BLE wakeup source
 ***************************************************************************************
 */
typedef void (*p_rwble_prevent_sleep_set)(uint16_t prv_slp_bit);
#define rwble_prevent_sleep_set ((p_rwble_prevent_sleep_set)(_rwble_prevent_sleep_set))
extern void exit_low_power_mode(void);
void OSC_EN_IRQHandler(void)
{
#if QN_32K_LOW_POWER_MODE_EN==TRUE
    exit_low_power_mode();
#else
    if (ahb_clock_flag & 0x01) {
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_AHB_DIVIDER, ahb_clock_flag);
        ahb_clock_flag &= ~0x01;
    }
#endif

    NVIC->ICER[0] = 0x00000020;
    rwble_prevent_sleep_set(0x0001);
}
#endif

#endif /* CONFIG_ENABLE_DRIVER_SLEEP==TRUE */
/// @} SLEEP
