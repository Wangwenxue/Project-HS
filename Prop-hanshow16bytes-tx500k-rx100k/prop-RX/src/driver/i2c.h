/**
 ****************************************************************************************
 *
 * @file i2c.h
 *
 * @brief Header file of I2C for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.1 $
 *
 ****************************************************************************************
 */
#ifndef _I2C_H_
#define _I2C_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_I2C==TRUE
#include "syscon.h"

/**
 ****************************************************************************************
 * @defgroup I2C I2C Driver
 * @ingroup DRIVERS
 * @brief I2C driver
 *
 *  I2C is a bi-directional serial bus with two wires that provides a simple and efficient
 *  method of data exchange between devices. The I2C standard is a true multi-master bus
 *  including collision detection and arbitration that prevents data corruption if two or
 *  more masters attempt to control the bus simultaneously.
 *
 *  For QN9020, I2C device could act as master or slave and I2C driver can help user to
 *  use I2C functions easily. The main features of I2C are listed as follow:
 *    - Both I2C master and slave control are supported
 *    - Master baud rate is configurable.
 *    - Supports up to 400Kbps baud rate.
 *    - Master & slave support both 8 bit and 10 bit address mode.
 *    - Master supports SCL synchronization, and bus arbitration.
 *    - Slave supports SCL stretching.
 *    - 8 bit shift register for transform.
 *
 * @{
 *
 ****************************************************************************************
 */


/* \example  i2c_example.c
 * This is an example of how to use the I2C driver.
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/// Define QN9020 I2C slave address
#define QN9020_I2C_ADDR                 0x1A

/// Define I2C ratio algorithm: I2C CLK(x) should less than or equal to __APB_CLK/20
#define I2C_SCL_RATIO(x)                (((__APB_CLK/(20 * (x))) - 1) << I2C_POS_SCL_RATIO)
//#define I2C_SCL_RATIO(x)                (((g_ApbClock/(20 * (x))) - 1) << I2C_POS_SCL_RATIO)

/// Set QN9020 I2C slave address
#define I2C_SLAVE_ADDR(x)               ((x) << I2C_POS_SLAVE_ADDR)

/// Mask of all I2C interrupt
#define I2C_MASK_ALL_INT                0x0000003F   /* 5 - 0 */
/// Define I2C timeout time
#define I2C_MAX_TIMEOUT                 0x0000FFFF

/// Define I2C master mode
#define I2C_MASTER                      0
/// Define I2C slave mode
#define I2C_SLAVE                       1


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */


/// I2C bus state
enum I2C_BUS_STATE
{
    I2C_BUS_FREE     = 0,       /*!< I2C bus free */
    I2C_BUS_BUSY     = 1        /*!< I2C bus busy */
};

/// I2C operate status
enum I2C_OP_FSM
{
    I2C_OP_IDLE      = 0,       /*!< I2C idle */
    I2C_OP_WRDATA    = 1,       /*!< I2C write data */
    I2C_OP_SETADDR   = 2,       /*!< I2C set address */
    I2C_OP_RDDATA    = 3,       /*!< I2C read data */
    I2C_OP_ABORT     = 4,       /*!< I2C abort */
    I2C_OP_FINISH    = 5        /*!< I2C operate finish */
};


/// I2C error code
enum I2C_ERR_CODE
{
    I2C_NO_ERROR     = 0,       /*!< I2C no error */
    I2C_CONFLICT     = 1,       /*!< I2C conflict */
    I2C_NO_ACK       = 2,       /*!< I2C no ack */
    I2C_TIMEOUT      = 3        /*!< I2C timeout */
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
  
/**
 ****************************************************************************************
 * @brief   Reset I2C module
 * @description
 *  This function is used to reset I2C module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void i2c_reset(void)
{
    // Reset I2C module
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_I2C_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_I2C_RST);
}


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if I2C_MODE == I2C_MASTER

#if CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE
void I2C_IRQHandler( void );
#endif
extern void i2c_init(uint32_t speed, uint8_t *buffer, uint16_t size);
extern void I2C_BYTE_WRITE(uint8_t saddr, uint8_t reg_addr, uint8_t reg_data);
extern uint8_t I2C_BYTE_READ(uint8_t saddr, uint8_t reg_addr);
extern void I2C_BYTE_WRITE2(uint8_t saddr, uint16_t reg_addr, uint8_t reg_data);
extern uint8_t I2C_BYTE_READ2(uint8_t saddr, uint16_t reg_addr);
extern void I2C_nBYTE_WRITE(uint8_t saddr, uint8_t reg_addr, uint8_t *buffer, uint16_t len);
extern void I2C_nBYTE_READ(uint8_t saddr, uint8_t reg_addr, uint8_t *buffer, uint16_t len);
extern void I2C_nBYTE_WRITE2(uint8_t saddr, uint16_t reg_addr, uint8_t *buffer, uint16_t len);
extern void I2C_nBYTE_READ2(uint8_t saddr, uint16_t reg_addr, uint8_t *buffer, uint16_t len);

#else // I2C_MODE == I2C_SLAVE

#if CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE
void I2C_IRQHandler( void );
#endif
extern void i2c_init(uint8_t *buffer, uint16_t size);
#endif /* I2C_MODE == I2C_MASTER */

/// @} I2C
#endif /* CONFIG_ENABLE_DRIVER_I2C==TRUE */
#endif /* end _I2C_H_ */
