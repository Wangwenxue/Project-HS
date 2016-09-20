#ifndef _ATE_SYS_H_
#define _ATE_SYS_H_

//#include "type.h"
#include "gpio.h"
#include "syscon.h"

#define DEBUG

#define CAL_DAT_LEN     4
#define SYS_DAT_LEN     9

//#define QN_PMU_VOLTAGE  1

///////////////////////////////////////////////////////////////
// struct
typedef struct
{
    uint32_t cal_data[CAL_DAT_LEN];
    uint32_t sys_data[SYS_DAT_LEN];
} RET_DATA_TYPE;

///////////////////////////////////////////////////////////////
// macro define
#define PK_LEN          2048

#define PREAMBLE_NUM    8
#define AA_NUM          5
#define CRC_NUM         3

#define PAGAIN_NUM      5

#define RES_NUM         15

#define DELAY           50//000

#define TRX_CHANNEL     10
#define MF_CHANNEL      21

/* Address Define */
#define UNRETENTION_ADDR        0x40005008
#define RETENTION_ADDR          0x4000600C
#define ALWAYSON_ADDR           0x40000008

#define UNRETENTION_DFT         0x0000FFFF
#define RETENTION_DFT           0x00
#define ALWAYSON_DFT            0x09

#define UNRETENTION_VAL         0x00005555
#define RETENTION_VAL           0x55555555

#define ALWAYSON_VAL            0x0A
#define ALWAYSON_MSK            0x0F

#define MBIST_SRAM_BASE_ADDR    0x10000000              // SRAM_BASE_ADDR, changed since ARM breakpoint issue
#define MBIST_SRAM_SIZE         0x00010000              // 64K
#define MBIST_SRAM_SEC_SIZE     0x2000                  // 8K

#define MBIST_SRAM_V0           0x55                    // 01010101b
#define MBIST_SRAM_V1           0xAA                    // 10101010b

#define MBIST_CODE_SIZE         0x2200                  //

#define MBIST_SRAM_TST_START_ADDR       (MBIST_SRAM_BASE_ADDR+MBIST_CODE_SIZE)
#define MBIST_SRAM_TST_END_ADDR         (MBIST_SRAM_BASE_ADDR+MBIST_SRAM_SIZE)

#define TEST_FAIL               1
#define TEST_OK                 0

/* Config GPIO Output */
#define ATE_GPIO_OK             GPIO_LOW
#define ATE_GPIO_FAILED         GPIO_HIGH

// golden
#define ATE_GOLDEN_NOTE_PIN     GPIO_P17

// dut output
#define ATE_GPIO_END_PIN        GPIO_P30
#define ATE_GPIO_RESULT_PIN     GPIO_P31

#define ATE_PRNT_PIN            (GPIO_P00 | GPIO_P01 | GPIO_P02 | GPIO_P03 | GPIO_P04 | GPIO_P05 | GPIO_P23 | GPIO_P24)

// dut input
#define ATE_DUT_RST_PIN         GPIO_P11
#define ATE_ENTER_SLP_PIN       GPIO_P13
#define ATE_DUT_NOTE_PIN        GPIO_P10
#define ATE_WKUP_PIN            GPIO_P12

#define HW32_REG(ADDRESS)       (*((volatile unsigned long  *)(ADDRESS)))

///////////////////////////////////////////////////////////////
// sleep configuration
/// Memory block
//#define QN_MEM_RETENTION        (MEM_BLOCK1 | MEM_BLOCK2 | MEM_BLOCK3 | MEM_BLOCK4 | MEM_BLOCK5 | MEM_BLOCK6 | MEM_BLOCK7)
//#define QN_MEM_UNRETENTION      (~(QN_MEM_RETENTION) & 0xfe)

//#define RET_START_ADDR          MBIST_SRAM_TST_START_ADDR
//#define RET_END_ADDR            0x10008000
//
//#define UNRET_START_ADDR        RET_END_ADDR
//#define UNRET_END_ADDR          MBIST_SRAM_TST_END_ADDR


///////////////////////////////////////////////////////////////
// function declare
void ate_SystemIOCfg (void);
void SystemCoreClockUpdate(void);
void ate_rst_gpio(void);
void ate_gpio_init(void);
void ate_gpio_output(bool r);
void test_wait(void);
void powerup_calibration(enum CLK_TYPE cry_type);
void ate_sys_init(uint32_t xtal);
void gpio_prnt(uint8_t data);
void rx_alwayson(void);
void tx_alwayson(void);
void led_flash(enum gpio_pin pin);

void save_cal_data(void);
void save_sys_data(void);
void load_cal_data(void);
void load_sys_data(void);

void slp_test(void);
void sleep_recover(void);

void dc_dc_enable(bool enable);

#endif

