/**
 ****************************************************************************************
 *
 * @file lib.h
 *
 * @brief QN9020 library API header file.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
 
#ifndef LIB_H_
#define LIB_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "driver_config.h"
#include "fw_func_addr.h"

/*
 * DEFINITIONS
 ****************************************************************************************
 */
#define QN_DBG_INFO_REG                     0x1000FFFC

#define QN_DBG_INFO_XTAL_WAKEUP_DURATION    0x00000001
#define QN_DBG_INFO_BLE_HEAP_FULL           0x00000002

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */
/// Work mode
enum WORK_MODE
{
    SOC_MODE,   // Wireless SoC
    NP_MODE,    // Network processor
    HCI_MODE    // Controller only
};

/// Power mode
enum PW_MODE
{
    NORMAL_MODE,        // Low power
    HIGH_PERFORMANCE    // High Power
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable 32k low power mode
 ***************************************************************************************
 */
extern void enable_32k_mode(void);

/**
 ****************************************************************************************
 * @brief DC-DC Enable
 * @param[in]   enable true - enable dc-dc; false - disable
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern void dc_dc_enable(bool enable);
#elif defined(QN_9020_B4)
typedef void (*p_dc_dc_enable)(bool enable);
#define dc_dc_enable ((p_dc_dc_enable)(_dc_dc_enable))
#endif

/**
 ****************************************************************************************
 * @brief Set 32k xtal ppm.
 * @param[in]     ppm
 ***************************************************************************************
 */
typedef void (*p_set_32k_ppm)(int32_t ppm);
#define set_32k_ppm ((p_set_32k_ppm)(_set_32k_ppm))

/**
 ****************************************************************************************
 * @brief Set 32k xtal frequency.
 * @param[in]     freq  frequency (Hz)
 ***************************************************************************************
 */
typedef void (*p_set_32k_freq)(int32_t ppm);
#define set_32k_freq ((p_set_32k_freq)(_set_32k_freq))

/**
 ****************************************************************************************
 * @brief Initilaize BLE hardware platform
 * @param[in]   pw_mode             NORMAL_MODE - low power; HIGH_PERFORMANCE - high power
 * @param[in]   xtal                which crystal is used - 16MHz or 32MHz
 * @param[in]   clk_32k             which 32k is used - 0:xtal, 1:rco
 * @param[in]   nvds_tmp_buf        pointer of nvds temp buffer
 * @param[in]   nvds_tmp_buf_len    length of nvds temp buffer
 ****************************************************************************************
 */
extern void plf_init(enum PW_MODE pw_mode, uint32_t xtal, uint8_t clk_32k, uint8_t* nvds_tmp_buf, uint32_t nvds_tmp_buf_len);


/**
 ****************************************************************************************
 * @brief Allow or prevent from the BLE hardware going to sleep
 * @param[in]   enable  ture - allow; false - prevent
 ****************************************************************************************
 */
typedef void (*p_enable_ble_sleep)(bool enable);
#define enable_ble_sleep ((p_enable_ble_sleep)(_enable_ble_sleep))

/**
 ****************************************************************************************
 * @brief Set maximum sleep duration of BLE sleep timer. If the BLE stack works, it will
 *        revises the sleep duration based on BLE stack's requirement, otherwise this
 *        sleep duration will be the setting value.
 * @param[in]   duration    unit 625us, maximum is 209715199(36hours16mins)
 ****************************************************************************************
 */
typedef bool (*p_set_max_sleep_duration)(uint32_t duration);
#define set_max_sleep_duration ((p_set_max_sleep_duration)(_set_max_sleep_duration))

/**
 ****************************************************************************************
 * @brief Set BLE program latency. The software should program BLE hardware before 
 *        the BLE hardware handle BLE event. This value specifices this duration.
 * @param[in]   latency    0 < latency <= 8, unit 625us, default value is 4
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern void set_ble_program_latency(uint8_t latency);
#elif defined(QN_9020_B4)
typedef void (*p_set_ble_program_latency)(uint8_t latency);
#define set_ble_program_latency ((p_set_ble_program_latency)(_set_ble_program_latency))
#endif

/**
 ***************************************************************
 * @brief Check the sleep status of ble hardware.
 ***************************************************************
*/
#if defined(QN_9020_B2)
extern bool ble_hw_sleep(void);
#elif defined(QN_9020_B4)
typedef bool (*p_ble_hw_sleep)(void);
#define ble_hw_sleep ((p_ble_hw_sleep)(_ble_hw_sleep))
#endif

/**
 ***************************************************************
 * @brief Check whether to do BLE external wakeup.
 ***************************************************************
*/
#if defined(QN_9020_B2)
extern bool ble_ext_wakeup_allow(void);
#elif defined(QN_9020_B4)
typedef bool (*p_ble_ext_wakeup_allow)(void);
#define ble_ext_wakeup_allow ((p_ble_ext_wakeup_allow)(_ble_ext_wakeup_allow))
#endif

/**
 ***************************************************************
 * @brief Wakeup BLE hardware by software.
 ***************************************************************
*/
typedef void (*p_sw_wakeup_ble_hw)(void);
#define sw_wakeup_ble_hw ((p_sw_wakeup_ble_hw)(_sw_wakeup_ble_hw))

/**
 ****************************************************************************************
 * @brief Register sleep callback.
 *        enter_cb is invoked before BLE entering sleep mode. If the return of
 *        callback function is FALSE, the BLE hardware will not enter into sleep mode.
 *        exit_cb provides user a way to do sth after ble hardware wakeup.
 * @param[in]     enter_cb   Callback function 
 * @param[in]     exit_cb    Callback function
 ***************************************************************************************
 */
#if defined(QN_9020_B2)
extern void reg_ble_sleep_cb(bool (*enter_cb)(void), void (*exit_cb)(void));
#elif defined(QN_9020_B4)
typedef void (*p_reg_ble_sleep_cb)(bool (*enter_cb)(void), void (*exit_cb)(void));
#define reg_ble_sleep_cb ((p_reg_ble_sleep_cb)(_reg_ble_sleep_cb))
#endif

/**
 ****************************************************************************************
 * @brief Save configuration which will lose in sleep mode
 ****************************************************************************************
 */
typedef void (*p_save_ble_setting)(void);
#define save_ble_setting ((p_save_ble_setting)(_save_ble_setting))

/**
 ****************************************************************************************
 * @brief Restore configuration which is saved before entering sleep mode
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern void restore_ble_setting(void);
#elif defined (QN_9020_B4)
typedef void (*p_restore_ble_setting)(void);
#define restore_ble_setting ((p_restore_ble_setting)(_restore_ble_setting))
#endif

/**
 ****************************************************************************************
 * @brief Sleep post process
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern void sleep_post_process(void);
#elif defined(QN_9020_B4)
typedef void (*p_sleep_post_process)(void);
#define sleep_post_process ((p_sleep_post_process)(_sleep_post_process))
#endif


/**
 ****************************************************************************************
 * @brief Check that BLE has already waked-up
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern uint32_t check_ble_wakeup(void);
#elif defined(QN_9020_B4)
typedef uint32_t (*p_check_ble_wakeup)(void);
#define check_ble_wakeup ((p_check_ble_wakeup)(_check_ble_wakeup))
#endif



/**
 ****************************************************************************************
 * @brief set debug bit, the debug information is saved at 0x1000fffc.
 * @param[in]  dbg_info_bit     Debug information
 ****************************************************************************************
 */
#if defined(QN_9020_B2)
extern void set_dbg_info(uint32_t dbg_info_bit);
#elif defined(QN_9020_B4)
typedef void (*p_set_dbg_info)(uint32_t dbg_info_bit);
#define set_dbg_info ((p_set_dbg_info)(_set_dbg_info))
#endif


#endif

