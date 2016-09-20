/**
 ****************************************************************************************
 *
 * @file serialflash.c
 *
 * @brief Serial Flash driver for QN9020 (MX25L512_U2, W25X40_U4, S25FL008A_U3).
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.1 $
 *
 ****************************************************************************************
 */

/**
****************************************************************************************
* @addtogroup  SERIAL_FLASH
* @{
****************************************************************************************
*/

/*
* INCLUDE FILES
****************************************************************************************
*/
#include "serialflash.h"
#include "syscon.h"

#if CONFIG_ENABLE_DRIVER_SERIAL_FLASH==TRUE
/// Serial flash command list
uint8_t g_flash_cmd[MAX_FLASH_CMD_NUM]=
{
    0x05,       /*!< Read Status Register */
    0x06,       /*!< Write Enable */
    0x20,       /*!< Sector Erase */
    0x52,       /*!< Block Erase */
    0x60,       /*!< Chip Erase */
    0xB9,       /*!< Deep Power Down */
    0xAB,       /*!< Release form Deep Power Down */
    0x01,       /*!< reserved, the value is not 0x00 and 0xFF */
};


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
****************************************************************************************
* @brief  Check whether flash is busy
* @return flash write operateion status
* @description
* Check flash status register's WIP before program, erase or write status register
*****************************************************************************************
*/
static bool is_flash_busy(void)
{
    uint32_t status;

    // write flash cmd
    sf_ctrl_SetCRWithMask(QN_SF_CTRL, SF_CTRL_MASK_RD_STAT_CMD, RD_FLASH_ST_CMD);

    // read the flash status
    status = sf_ctrl_GetFlashSR(QN_SF_CTRL);
    
    // WIP=1 is in write operation, WIP=0 is not in write operation.
    return ((status & 0x01) == 0x1);
}

/**
****************************************************************************************
* @brief  Enable Flash write operation
*****************************************************************************************
*/
static void flash_write_enable(void)
{
    // wait for flash free
    while(is_flash_busy());

    // write flash cmd
    sf_ctrl_SetCMD(QN_SF_CTRL, g_flash_cmd[WREN_CMD]);
}

/*
* EXPORTED FUNCTION DEFINITIONS
****************************************************************************************
*/

/**
****************************************************************************************
* @brief Set Serial Flash controller clock
* @param[in]  clock_div        FLASH_CLK_DIV(clock), clock units is Hz
* @description
*  This functin is used to set serial flash controller work clock.
*****************************************************************************************
*/
void set_flash_clock(uint32_t clock_div)
{
    sf_ctrl_SetCRWithMask(QN_SF_CTRL, SF_CTRL_MASK_CLK_DIV, clock_div);
}

/**
****************************************************************************************
* @brief  Read out flash ID
* @return Flash ID
* @description
*  This function is used to read serial flash ID, which consists of 3 or 4 bytes depending on difference vendor.
*****************************************************************************************
*/
uint32_t read_flash_id(void)
{
    return sf_ctrl_GetFlashID(QN_SF_CTRL);
}

/**
****************************************************************************************
* @brief  Erase n sectors of flash
* @param[in]  addr        A23-A0 specified a valid 24bit address of a sector
* @param[in]  n           number of sector
* @description
*  This function is used to erase serial flash sector.
*****************************************************************************************
*/
void sector_erase_flash(uint32_t addr, uint32_t n)
{
    while (n--)
    {
        flash_write_enable(); //set the Write Enable Latch bit
        sf_ctrl_SetCmdAddr(QN_SF_CTRL, __REV((addr & 0xffffff) | (g_flash_cmd[SE_CMD]<<24)));
        addr += 4*1024;            //all flash sector erase command erasing size is 4K
    }
}

/**
****************************************************************************************
* @brief  Erase n blocks of flash
* @param[in]  addr        A23-A0 specified a valid 24bit address of a block.
* @param[in]  block_size  flash a block content size
* @param[in]  n           requirement erasing number blocks
* @description
*  This function is used to erase serial flash block.
*****************************************************************************************
*/
void block_erase_flash(uint32_t addr, uint32_t block_size, uint32_t n)
{
    while (n--)
    {
        flash_write_enable(); //set the Write Enable Latch bit
        sf_ctrl_SetCmdAddr(QN_SF_CTRL, __REV(addr | (g_flash_cmd[BE_CMD]<<24)));
        addr += block_size;
    }
}

/**
****************************************************************************************
* @brief  Read data form flash
* @param[in]  addr        flash address(3 bytes)
* @param[in]  pBuf        pointer to read data buffer address
* @param[in]  nByte       read size, it must <= 256 and must be 4 integer times
* @description
*  This function is used to read data from serial flash.
*
* @note
*  1. The parameter "addr" note:
*       - When the address range is from 0x00 to 0x1000 (NVDS area), the address must be 4
*         integer times.
*       - When the address range is greater than or equal to 0x1000 (Code area), the
*         address must be 256 integer times. (Encryption request)
*  2. The parameter "nByte" note:
*       - When the address range is from 0x00 to 0x1000 (NVDS area), the size must be 4
*         integer times and less than or equal to 256.
*       - When the address range is greater than or equal to 0x1000 (Code area), the
*         size must be 256 bytes integer times. (Encryption request)
*****************************************************************************************
*/
void read_flash(uint32_t addr, uint32_t *pBuf, uint32_t nByte)
{
    addr += QN_FLASH_BASE;   //get the data register address of flash control register
    while(is_flash_busy());  //wait the flash is free.
    sf_ctrl_SetDataLen(QN_SF_CTRL, nByte);
    nByte >>= 2;               //nByte must is 4 integer times
    while (nByte--)
    {
        *pBuf++ = *(uint32_t *)addr;
        addr += 4;
    }
}

/**
****************************************************************************************
* @brief      Write data to flash
* @param[in]  addr        flash address(3 bytes)
* @param[in]  pBuf        pointer to write data address
* @param[in]  nByte       write size, it must <= 256 and must be 4 integer times
* @description
*  This function is used to write data to serial flash.
*
* @note
*  1. The parameter "addr" note:
*       - When the address range is from 0x00 to 0x1000 (NVDS area), the address must be 4
*         integer times.
*       - When the address range is greater than or equal to 0x1000 (Code area), the
*         address must be 256 integer times. (Encryption request)
*  2. The parameter "nByte" note:
*       - When the address range is from 0x00 to 0x1000 (NVDS area), the size must be 4
*         integer times and less than or equal to 256.
*       - When the address range is greater than or equal to 0x1000 (Code area), the
*         size must be 256 bytes. (Encryption request)
*****************************************************************************************
*/
void write_flash(uint32_t addr, const uint32_t *pBuf, uint32_t nByte)
{
    addr += QN_FLASH_BASE;     //get the data register address of flash control register
    sf_ctrl_SetDataLen(QN_SF_CTRL, nByte);
    flash_write_enable();      //set the Write Enable Latch bit
    nByte >>= 2;               //nByte must is 4 integer time.
    while (nByte--)
    {
        *(uint32_t *)addr = *pBuf++;
        addr += 4;
    }
}

/**
****************************************************************************************
* @brief check whether flash is present
* @return ture or false
*****************************************************************************************
*/
bool is_flash_present(void)
{
    uint32_t id = read_flash_id() & 0xFFFFFF;

    if(id > 0 && id < 0xFFFFFF)
        return true;
    else
        return false;
}

/**
****************************************************************************************
* @brief Power on serial flash.
* @param[in]  type        flash accessing type(read or write).
* @description
*   There is a setup duration from power on to accessing flash correctly. Generally
*   the duration from power on to allowance of reading is shorter than the duration from
*   power on to allowance of writing. The parameter allows the developer select this 
*   duration depends on application scenarios.
*   This duration is different for different flash. Please reference the specification
*   of selected flash and configure macro FLASH_SETUP_DUR_RD and FLASH_SETUP_DUR_WR in 
*   the serialflash.h.
*****************************************************************************************
*/
void power_on_flash(enum ACCESS_TYPE type)
{
    uint32_t setup_dur;

#if defined(QN_EXT_FLASH)    
    // switch to flash controller
    syscon_SetPMCR1WithMask(QN_SYSCON, SYSCON_MASK_FLASH_CTRL_PIN, MASK_DISABLE);
#endif
    
    // power on
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_FLASH_VCC_EN, MASK_ENABLE);

    // select flash setup duration
    if(type == FLASH_RD)
        setup_dur = FLASH_SETUP_DELAY_RD;
    else
        setup_dur = FLASH_SETUP_DELAY_WR;
    
    // wait for setup_dur(us) then enable flash clock
    delay(setup_dur);

    // flash clock on
    flash_clock_on();

    // wait for flash is ready
    while(!is_flash_present());
}

/**
****************************************************************************************
* @brief Power off serial flash.
*****************************************************************************************
*/
void power_off_flash(void)
{
    // wait for the flash is free when flash is entered power-down
    while (is_flash_busy());

    // power off
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_FLASH_VCC_EN, MASK_DISABLE);

    // flash clock off
    flash_clock_off();

#if defined(QN_EXT_FLASH)
    // switch to GPIO, prevent leakage 
    syscon_SetPMCR1WithMask(QN_SYSCON, SYSCON_MASK_FLASH_CTRL_PIN, MASK_ENABLE);
#endif
}

#endif /* CONFIG_ENABLE_DRIVER_SERIAL_FLASH==TRUE */
/// @} SERIAL_FLASH
