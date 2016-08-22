/**
 ****************************************************************************************
 *
 * @file lib.c
 *
 * @brief Patch, BLE hardware initialization, BLE work mode.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "gpio.h"
#include "uart.h"

#include "qnrf.h"
#include "calibration.h"
#include "lib.h"
#include "system.h"

#include "nvds.h"

#include "sleep.h"
#include "serialflash.h"

typedef void (*p_powerup_calibration)(enum CLK_TYPE cry_type);
#define powerup_calibration ((p_powerup_calibration)(_powerup_calibration))

/*
 * DEFINITIONS
 ****************************************************************************************
 */
 
#define APP_FLAG_32K_MODE            0x1

#define REG_BLE_RD(addr)             __rd_reg(addr)
#define REG_BLE_WR(addr, value)      __wr_reg(addr, value)


/*
 * STRUCTURES
 ****************************************************************************************
 */
struct qn_config_api
{
    // ACI / HCI
    uint8_t     aci_flag;
    uint8_t     hci_flag;
    int         aci_layer;
    void (*hci_tx_done)(void);
    QN_UART_TypeDef *uart;
    void (*uart_read)(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*rx_callback)(void));
    void (*uart_write)(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*tx_callback)(void));
    void (*uart_flow_on)(QN_UART_TypeDef *UART);
    bool (*uart_flow_off)(QN_UART_TypeDef *UART);    
    bool (*hci_enter_sleep)(void);
    void (*hci_exit_sleep)(void);

    // BLE
    uint8_t     ble_con_max;
    uint16_t    prefetch_time;
    uint8_t     lld_evt_prog_latency;
    uint8_t     lld_sleep_advance;
    uint8_t     lld_evt_default_rx_win_size;
    uint8_t     lld_evt_rx_win_default_offset;
    uint8_t     lld_adv_delay_enable;
    uint8_t     lld_prog_check;
    void (*ble_isr_callback)(uint32_t *irq_st);
    void (*timer_callback)(uint16_t msg_id, uint16_t task_id);
    uint8_t     cx_scan_cnx;
    volatile bool ext_ctrl;
    volatile uint8_t patch_ctrl;

    // BLE heap
    uint8_t     *heap_addr;
    uint32_t    heap_size;

    // BLE sleep
    uint32_t    max_sleep_duration_periodic_wakeup;
    uint32_t    max_sleep_duration_external_wakeup;

    bool (*enter_sleep_cb)(void);
    void (*exit_sleep_cb)(void);

    uint32_t ble_retention_regs[69];
    uint32_t cal_retention_regs[4];
    uint32_t sys_retention_regs[10];

    volatile bool sleep_flag;
    int32_t mode;

    // Reset callback
    void (*plf_reset_cb)(void);
    
    // Flash power on delay
    uint16_t flash_on_delay;
};
#define config_api    ((struct qn_config_api*)(_config_api))


struct nvds_env_tag
{
    /// Function to read the device Address being in the NVDS memory space
    void  (*read)(uint32_t const                    address,
                  uint32_t const                    length,
                  uint8_t* const                    buf);
    /// Function to write the device Address being in the NVDS memory space
    void (*write)(uint32_t const                    address,
                  uint32_t const                    length,
                  uint8_t* const                    buf);
    /// Function to erase the entire NVDS memory space
    void (*erase)(uint32_t const                    address,
                  uint32_t const                    length);
    
    void (*flash_on)(void);
    void (*flash_off)(void);

    /// NVDS base pointer
    uint8_t *nvds_space;

    /// Total size of the NVDS area
    uint32_t   total_size;

    /// Flash ID
    uint8_t flash_id;

    // WJ : temp buffer
    /// temp buffer pointer
    uint8_t *nvds_temp_buf;

    /// temp buffer length
    uint32_t nvds_temp_buf_len;
};
#define nvds_env    ((struct nvds_env_tag*)(_nvds_env))

/*
 * LOCAL VARIABLES
 ****************************************************************************************
 */

static volatile uint8_t app_flags = 0;

/*
 * LOCAL XXX
 ****************************************************************************************
 */


/*
 * FUNCTIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable 32k low power mode
 ***************************************************************************************
 */
void enable_32k_mode(void)
{
    app_flags |= APP_FLAG_32K_MODE;
}

/**
 ****************************************************************************************
 * @brief BLE interrupt service routine
 *
 * This function is the interrupt service handler of BLE stack.
 *
 ****************************************************************************************
 */

/*
 ****************************************************************************************
 * @brief Initilaize QN9020 platform
 ****************************************************************************************
 */

typedef void (*p_qn_config_init)(struct qn_config_api *api);
#define qn_config_init ((p_qn_config_init)(_qn_config_init))

static void plf_reset_cb(void)
{
    set_dbg_info(QN_DBG_INFO_BLE_HEAP_FULL);
}

typedef void (*p_qn_plf_init)(enum PW_MODE pw_mode, uint32_t xtal, uint8_t clk_32k, uint8_t* nvds_tmp_buf, uint32_t nvds_tmp_buf_len);
#define qn_plf_init ((p_qn_plf_init)(_qn_plf_init))

typedef void (*p_flash_on)(void);
typedef void (*p_flash_off)(void);

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
void plf_init(enum PW_MODE pw_mode, uint32_t xtal, uint8_t clk_32k, uint8_t* nvds_tmp_buf, uint32_t nvds_tmp_buf_len)
{
    uint32_t reg_val = syscon_GetAdditionCR(QN_SYSCON);
    
    qn_plf_init(pw_mode, xtal, 1, nvds_tmp_buf, nvds_tmp_buf_len);

    if(nvds_env->flash_on == 0)
    {
        nvds_env->flash_on = (p_flash_on)(_flash_on);
        nvds_env->flash_off = (p_flash_on)(_flash_off);
    }
    nvds_env->flash_off();
    
    syscon_SetAdditionCRWithMask(QN_SYSCON, SYSCON_MASK_XADD_C, reg_val);
    
    if(clk_32k == 0)
    {
        // ensable schmitt trigger in 32.768KHz buffer
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32SMT_EN, MASK_ENABLE);
        // Set 32.768KHz xtal higher current, that can let the 32k xtal stable fastly (decrease stable time).
        syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_X32ICTRL, 32);
    }
    
    config_api->plf_reset_cb = plf_reset_cb;

    //set random seed
    srand((unsigned)QN_TIMER0->CNT);
}

