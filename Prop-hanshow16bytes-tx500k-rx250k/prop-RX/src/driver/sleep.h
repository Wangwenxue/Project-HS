/**
 ****************************************************************************************
 *
 * @file sleep.h
 *
 * @brief Header file of sleep for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _SLEEP_H_
#define _SLEEP_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_SLEEP==TRUE
#include "syscon.h"

/**
 ****************************************************************************************
 * @defgroup SLEEP Sleep Driver
 * @ingroup DRIVERS
 * @brief Sleep driver
 *
 * In QN9020, three sleep modes are defined according to cortex-M0 low power modes. 
 * 
 * - CPU clock gating mode: 
 *  Cortex-M0 can be clock gated, 
 *  NVIC remains sensitive to interrupts, 
 *  all NVIC interrupt sources can wake up Cortex-M0.
 *  
 * - CPU deep clock gating mode:
 *  Cortex-M0 and NVIC can be clock gated, 
 *  WIC remains sensitive  to selected interrupts, 
 *  all WIC interrupt sources can wake up Cortex-M0, 
 *  Cortex-M0 can be put into state retention.
 *
 * - CPU sleep mode: 
 *  Power down Cotex-M0 processor, 
 *  all clocks can be powered down, 32Khz clock is an option(if using sleep timer wakeup), 
 *  WIC signals wake-up to PMU, 
 *  all WIC interrupt sources can wake up Cortex-M0, 
 *  Cortex-M0 can be put into state retention.
 *
 * @{
 *
 ****************************************************************************************
 */

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */



/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "syscon.h"
#if GPIO_WAKEUP_EN == TRUE
#include "gpio.h"
#endif
#if ACMP_WAKEUP_EN == TRUE
#include "analog.h"
#endif
    
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// Device active bit field
#define PM_MASK_ADC_ACTIVE_BIT          (0x00000001)
#define PM_MASK_DMA_ACTIVE_BIT          (0x00000002)
#define PM_MASK_UART0_TX_ACTIVE_BIT     (0x00000004)
#define PM_MASK_UART1_TX_ACTIVE_BIT     (0x00000008)
#define PM_MASK_UART0_RX_ACTIVE_BIT     (0x00000010)
#define PM_MASK_UART1_RX_ACTIVE_BIT     (0x00000020)
#define PM_MASK_SPI0_TX_ACTIVE_BIT      (0x00000040)
#define PM_MASK_SPI1_TX_ACTIVE_BIT      (0x00000080)
#define PM_MASK_SPI0_RX_ACTIVE_BIT      (0x00000100)
#define PM_MASK_SPI1_RX_ACTIVE_BIT      (0x00000200)
#define PM_MASK_I2C_ACTIVE_BIT          (0x00000400)
#define PM_MASK_TIMER0_ACTIVE_BIT       (0x00000800)
#define PM_MASK_TIMER1_ACTIVE_BIT       (0x00001000)
#define PM_MASK_TIMER2_ACTIVE_BIT       (0x00002000)
#define PM_MASK_TIMER3_ACTIVE_BIT       (0x00004000)
#define PM_MASK_PWM0_ACTIVE_BIT         (0x00008000)
#define PM_MASK_PWM1_ACTIVE_BIT         (0x00010000)
#define PM_POS_ADC_ACTIVE_BIT           0
#define PM_POS_DMA_ACTIVE_BIT           1
#define PM_POS_UART0_TX_ACTIVE_BIT      2
#define PM_POS_UART1_TX_ACTIVE_BIT      3
#define PM_POS_UART0_RX_ACTIVE_BIT      4
#define PM_POS_UART1_RX_ACTIVE_BIT      5
#define PM_POS_SPI0_TX_ACTIVE_BIT       6
#define PM_POS_SPI1_TX_ACTIVE_BIT       7
#define PM_POS_SPI0_RX_ACTIVE_BIT       8
#define PM_POS_SPI1_RX_ACTIVE_BIT       9
#define PM_POS_I2C_ACTIVE_BIT           10
#define PM_POS_TIMER0_ACTIVE_BIT        11
#define PM_POS_TIMER1_ACTIVE_BIT        12
#define PM_POS_TIMER2_ACTIVE_BIT        13
#define PM_POS_TIMER3_ACTIVE_BIT        14
#define PM_POS_PWM0_ACTIVE_BIT          15
#define PM_POS_PWM1_ACTIVE_BIT          16

/// Wakeup by all of the system interrupt source
#define WAKEUP_BY_ALL_IRQ_SOURCE   ( WAKEUP_BY_GPIO          \
                                   | WAKEUP_BY_ACMP0         \
                                   | WAKEUP_BY_ACMP1         \
                                   | WAKEUP_BY_BLE           \
                                   | WAKEUP_BY_RTC_CAP       \
                                   | WAKEUP_BY_RTC           \
                                   | WAKEUP_BY_ADC           \
                                   | WAKEUP_BY_DMA           \
                                   | WAKEUP_BY_UART0_TX      \
                                   | WAKEUP_BY_UART0_RX      \
                                   | WAKEUP_BY_SPI0_TX       \
                                   | WAKEUP_BY_SPI0_RX       \
                                   | WAKEUP_BY_UART1_TX      \
                                   | WAKEUP_BY_UART1_RX      \
                                   | WAKEUP_BY_SPI1_TX       \
                                   | WAKEUP_BY_SPI1_RX       \
                                   | WAKEUP_BY_I2C           \
                                   | WAKEUP_BY_TIMER0        \
                                   | WAKEUP_BY_TIMER1        \
                                   | WAKEUP_BY_TIMER2        \
                                   | WAKEUP_BY_TIMER3        \
                                   | WAKEUP_BY_WDT           \
                                   | WAKEUP_BY_PWM0          \
                                   | WAKEUP_BY_PWM1          \
                                   | WAKEUP_BY_TUNER_SETTING)

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// power mode
enum POWER_MODE
{
    PM_ACTIVE,          /*!< CO_PD_DISALLOWED, disallow cpu clock off & cpu power down */
    PM_IDLE,            /*!< CPU_CLK_OFF_ALLOW */
    PM_SLEEP,           /*!< CPU_POWER_DOWN_ALLOW */
    PM_DEEP_SLEEP       /*!< CPU_DEEP_SLEEP_ALLOW */
};

/// QN9020 sleep mode
enum SLEEP_MODE
{
    SLEEP_CPU_CLK_OFF = 0,      /*!< Disable CPU clock */
    SLEEP_NORMAL,               /*!< Sleep */
    SLEEP_DEEP                  /*!< Deep Sleep */
};

/// QN9020 wakeup source
enum WAKEUP_SOURCE
{                        
    WAKEUP_BY_GPIO           = (1 << GPIO_IRQn),           /*!< Wakeup by GPIO          */ 
    WAKEUP_BY_ACMP0          = (1 << ACMP0_IRQn),          /*!< Wakeup by ACMP0         */ 
    WAKEUP_BY_ACMP1          = (1 << ACMP1_IRQn),          /*!< Wakeup by ACMP1         */ 
    WAKEUP_BY_BLE            = (1 << BLE_IRQn),            /*!< Wakeup by BLE           */ 
    WAKEUP_BY_RTC_CAP        = (1 << RTC_CAP_IRQn),        /*!< Wakeup by RTC_CAP       */ 
    WAKEUP_BY_OSC_EN         = (1 << OSC_EN_IRQn),         /*!< Wakeup by OSC_EN        */ 
    WAKEUP_BY_RTC            = (1 << RTC_IRQn),            /*!< Wakeup by RTC           */ 
    WAKEUP_BY_ADC            = (1 << ADC_IRQn),            /*!< Wakeup by ADC           */ 
    WAKEUP_BY_DMA            = (1 << DMA_IRQn),            /*!< Wakeup by DMA           */ 
    WAKEUP_BY_UART0_TX       = (1 << UART0_TX_IRQn),       /*!< Wakeup by UART0_TX      */ 
    WAKEUP_BY_UART0_RX       = (1 << UART0_RX_IRQn),       /*!< Wakeup by UART0_RX      */ 
    WAKEUP_BY_SPI0_TX        = (1 << SPI0_TX_IRQn),        /*!< Wakeup by SPI0_TX       */ 
    WAKEUP_BY_SPI0_RX        = (1 << SPI0_RX_IRQn),        /*!< Wakeup by SPI0_RX       */ 
    WAKEUP_BY_UART1_TX       = (1 << UART1_TX_IRQn),       /*!< Wakeup by UART1_TX      */ 
    WAKEUP_BY_UART1_RX       = (1 << UART1_RX_IRQn),       /*!< Wakeup by UART1_RX      */ 
    WAKEUP_BY_SPI1_TX        = (1 << SPI1_TX_IRQn),        /*!< Wakeup by SPI1_TX       */ 
    WAKEUP_BY_SPI1_RX        = (1 << SPI1_RX_IRQn),        /*!< Wakeup by SPI1_RX       */ 
    WAKEUP_BY_I2C            = (1 << I2C_IRQn),            /*!< Wakeup by I2C           */ 
    WAKEUP_BY_TIMER0         = (1 << TIMER0_IRQn),         /*!< Wakeup by TIMER0        */ 
    WAKEUP_BY_TIMER1         = (1 << TIMER1_IRQn),         /*!< Wakeup by TIMER1        */ 
    WAKEUP_BY_TIMER2         = (1 << TIMER2_IRQn),         /*!< Wakeup by TIMER2        */ 
    WAKEUP_BY_TIMER3         = (1 << TIMER3_IRQn),         /*!< Wakeup by TIMER3        */ 
    WAKEUP_BY_WDT            = (1 << WDT_IRQn),            /*!< Wakeup by WDT           */ 
    WAKEUP_BY_PWM0           = (1 << PWM0_IRQn),           /*!< Wakeup by PWM0          */ 
    WAKEUP_BY_PWM1           = (1 << PWM1_IRQn),           /*!< Wakeup by PWM1          */ 
    WAKEUP_BY_CALIB          = (1 << CALIB_IRQn),          /*!< Wakeup by CALIB         */ 
    WAKEUP_BY_TUNER_RX       = (1 << TUNER_RX_IRQn),       /*!< Wakeup by TUNER_RX      */ 
    WAKEUP_BY_TUNER_TX       = (1 << TUNER_TX_IRQn),       /*!< Wakeup by TUNER_TX      */ 
    WAKEUP_BY_TUNER_SETTING  = 0,                          /*!< Wakeup by TUNER_SETTING */ 
};

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
 
/// SLEEP environment parameters
struct sleep_env_tag
{
    uint8_t     sleep_allow;
    uint32_t    dev_active_bf;
    bool        deep_sleep;

    int         retention_modules;
    int         wakeup_by_sleeptimer;
};

extern struct sleep_env_tag sleep_env;

extern volatile uint32_t PGCR1_restore;
extern volatile uint8_t low_power_mode_en;
extern volatile uint32_t ahb_clock_flag;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief   Set user program's power mode.
 * @param[in]   pm  active/clock off/sleep/deep sleep
 *                  PM_ACTIVE
 *                  PM_IDLE,
 *                  PM_SLEEP,
 *                  PM_DEEP_SLEEP
 ****************************************************************************************
 */
__STATIC_INLINE void sleep_set_pm(uint8_t pm)
{
    sleep_env.sleep_allow = pm;
}

/**
 ****************************************************************************************
 * @brief   Get user program's power mode
 * @return  sleep allowed status
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t sleep_get_pm(void)
{
    return sleep_env.sleep_allow;
}

/**
 ****************************************************************************************
 * @brief   Device prevent sleep
 * @param[in]   dev_bf  bit field of active device
 ****************************************************************************************
 */
__STATIC_INLINE void dev_prevent_sleep(uint32_t dev_bf)
{
    sleep_env.dev_active_bf |= dev_bf;
}

/**
 ****************************************************************************************
 * @brief   User device allow sleep
 * @param[in]   dev_bf  bit field of active device
 ****************************************************************************************
 */
__STATIC_INLINE void dev_allow_sleep(uint32_t dev_bf)
{
    sleep_env.dev_active_bf &= (~dev_bf);
}

/**
 ****************************************************************************************
 * @brief   Get device bit field
 * @return  device actived bits
 ****************************************************************************************
 */
__STATIC_INLINE uint32_t dev_get_bf(void)
{
    return sleep_env.dev_active_bf;
}

/**
 ****************************************************************************************
 * @brief  Exit low power mode
 * @description
 *  This function is used to set MCU exiting from low power mode, switch system clock to internal 20MHz.
 *****************************************************************************************
 */
__STATIC_INLINE void exit_low_power_mode(void)
{
    if (low_power_mode_en) {
        // restore PGCR1 reigster
        syscon_SetPGCR1(QN_SYSCON, PGCR1_restore);
        // switch to internal 20MHz
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_INT_20M<<SYSCON_POS_CLK_MUX);
        low_power_mode_en = 0;
    }
}


extern int usr_sleep(void);
extern void sleep_init(void);
extern void enter_sleep(enum SLEEP_MODE mode, uint32_t iconfig, void (*callback)(void));
#if GPIO_WAKEUP_EN == TRUE
extern void wakeup_by_gpio(enum gpio_pin pin, enum gpio_wakeup_type type);
#endif
#if ACMP_WAKEUP_EN == TRUE
extern void wakeup_by_analog_comparator(enum ACMP_CH acmpch, void (*callback)(void));
#endif
#if SLEEP_TIMER_WAKEUP_EN == TRUE
extern void wakeup_by_sleep_timer(int clk_src);
#endif
#if SLEEP_CALLBACK_EN == TRUE
extern void sleep_cb(void);
#endif
#if (QN_DEEP_SLEEP_EN && !QN_32K_RCO)
extern void wakeup_32k_xtal_switch_clk(void);
extern void wakeup_32k_xtal_start_timer(void);
#endif

extern void enter_low_power_mode(uint32_t en);
extern void restore_from_low_power_mode(void (*callback)(void));




#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

/// @} SLEEP
#endif /* CONFIG_ENABLE_DRIVER_SLEEP==TRUE */
#endif /* _SLEEP_H_ */
