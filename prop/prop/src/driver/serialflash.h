/**
 ****************************************************************************************
 *
 * @file serialflash.h
 *
 * @brief Header file of serial flash driver for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.1 $
 *
 ****************************************************************************************
 */
#ifndef _SERIAL_FLASH_H_
#define _SERIAL_FLASH_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_SERIAL_FLASH==TRUE
#include "syscon.h"

/**
 ****************************************************************************************
 * @defgroup SERIAL_FLASH Serial Flash Driver
 * @ingroup DRIVERS
 * @brief Serial Flash Driver
 *
 *  QN9020 contains a Serial Flash Controller, which has mainly 2 functions: access external
 *  serial flash (erase/read/write) and boot from external serial flash (copy code from external
 *  serial flash to internal RAM and then to execute). The main features are listed as follow:
 *    - Access serial flash by SPI master port
 *    - Support SPI mode 0, up to 16MHz clock output
 *    - Flash command configurable
 *    - Two 8-bit shift registers for data transmit and receive
 *    - 6 bit pre-scaler
 *    - Boot error detection
 *    - Support code encryption & decryption
 *
 * @{
 ****************************************************************************************
 */

/* \example serialflash_example.c
 * This is an example of how to use the Serial Flash driver.
 */

#ifdef __cplusplus
#if __cplusplus
extern "C"{
#endif
#endif /* __cplusplus */


/// Set serial Flash clock divider
#define FLASH_CLK_DIV(x)        (g_AhbClock/(2*(x)) - 1)
/// Read serial flash status command
#define RD_FLASH_ST_CMD         (g_flash_cmd[RDSR_CMD] << 8)
/* MX25 series command hex application definition */
//ID commands
#define FLASH_CMD_RDID          0x9F    /*!< RDID (Read Identification) */
#define FLASH_CMD_RES           0xAB    /*!< RES (Read Electronic ID) */
#define FLASH_CMD_REMS          0x90    /*!< REMS (Read Electronic & Device ID) */

//Register commands
#define FLASH_CMD_WRSR          0x01    /*!< WRSR (Write Status Register) */
#define FLASH_CMD_RDSR          0x05    /*!< RDSR (Read Status Register) */

//READ commands
#define FLASH_CMD_READ          0x03    /*!< READ (1 x I/O) */
#define FLASH_CMD_FASTREAD      0x0B    /*!< FAST READ (Fast read data) */
#define FLASH_CMD_DREAD         0x3B    /*!< DREAD (1In/2 Out fast read) */

//Program commands
#define FLASH_CMD_WREN          0x06    /*!< WREN (Write Enable) */
#define FLASH_CMD_WRDI          0x04    /*!< WRDI (Write Disable) */
#define FLASH_CMD_PP            0x02    /*!< PP (page program) */

//Erase commands
#define FLASH_CMD_SE            0x20    /*!< SE (Sector Erase) */
#define FLASH_CMD_BE            0xD8    /*!< BE (Block Erase) */
#define FLASH_CMD_CE            0x60    /*!< CE (Chip Erase) hex application: 60 or C7 */

//Mode setting commands
#define FLASH_CMD_DP            0xB9    /*!< DP (Deep Power Down) */
#define FLASH_CMD_RDP           0xAB    /*!< RDP (Release form Deep Power Down) */

//Flash setup duration after power on, unit is us.
#define FLASH_SETUP_DUR_RD      20
#define FLASH_SETUP_DUR_WR      200
#define FLASH_SETUP_DELAY_RD    (__AHB_CLK / 1000000 * FLASH_SETUP_DUR_RD / 9)
#define FLASH_SETUP_DELAY_WR    (__AHB_CLK / 1000000 * FLASH_SETUP_DUR_WR / 9)


/// Serial flash command index
enum FLASH_CMD
{
    RDSR_CMD = 0,         /*!< read status register */
    WREN_CMD = 1,         /*!< flash write enable */
    SE_CMD   = 2,         /*!< sector erase flash */
    BE_CMD   = 3,         /*!< block erase flash */
    CE_CMD   = 4,         /*!< erase whole flash */
    DPD_CMD  = 5,         /*!< deep power down */
    RDPD_CMD = 6,         /*!< release deep power down */
    RESERVED  = 7,        /*!< RESERVED */
    MAX_FLASH_CMD_NUM     /*!< it must 4 integer times */
};

/// Serial flash power type
enum POWER_TYPE
{
    FLASH_POWER_DOWN = 0,   /*!< Set Serial flash power down */
    FLASH_POWER_ON          /*!< Set Serial flash power on */
};

/// Serial flash accessing type after power on
enum ACCESS_TYPE
{
    FLASH_RD,
    FLASH_WR
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
  
/**
 ****************************************************************************************
 * @brief   Enable Serial Flash module clock
 * @description
 *  This function is used to enable Serial Flash module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void flash_clock_on(void)
{
    // enable Serial Flash module clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
}

/**
 ****************************************************************************************
 * @brief   Disable Serial Flash module clock
 * @description
 *  This function is used to disable Serial Flash module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void flash_clock_off(void)
{
    // disable Serial Flash module clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_SPI_AHB);
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

extern uint8_t g_flash_cmd[];

extern void set_flash_clock(uint32_t clock_div);
extern void read_flash(uint32_t addr, uint32_t *pBuf, uint32_t nbyte);
extern void write_flash(uint32_t addr,const uint32_t *pBuf,uint32_t nbyte);
extern void sector_erase_flash(uint32_t addr, uint32_t n);
extern void block_erase_flash(uint32_t addr, uint32_t block_size, uint32_t n);
extern uint32_t read_flash_id(void);

// new function
extern bool is_flash_present(void);
extern void power_on_flash(enum ACCESS_TYPE type);
extern void power_off_flash(void);


#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */


/// @} SERIAL_FLASH
#endif /* CONFIG_ENABLE_DRIVER_SERIAL_FLASH==TRUE */
#endif /* _SERIAL_FLASH_H_ */
