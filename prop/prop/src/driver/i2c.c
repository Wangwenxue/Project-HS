/**
 ****************************************************************************************
 *
 * @file i2c.c
 *
 * @brief I2C driver for QN9020.
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
 * @addtogroup  I2C
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "i2c.h"
#if CONFIG_ENABLE_DRIVER_I2C==TRUE

#if I2C_MODE == I2C_MASTER
/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

///Structure defining I2C environment parameters
struct i2c_env_tag
{
    int16_t             i2cIndex;
    int16_t             i2cTxCount;
    int16_t             i2cRxCount;
    uint16_t            i2cBufferSize;
    uint8_t             *i2cBuffer;
    enum I2C_OP_FSM     i2cOpFsm;
#if I2C_CALLBACK_EN==TRUE
    void                (*callback)(void);
#endif
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///I2C environment variable
static volatile struct i2c_env_tag i2c_env;



#if CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE
/**
 ****************************************************************************************
 * @brief I2C interrupt handler, deal with master mode only.
 ****************************************************************************************
 */
void I2C_IRQHandler(void)
{
    uint32_t status;
    uint32_t reg = 0;

    status = i2c_i2c_GetIntStatus(QN_I2C);
    if (status & I2C_MASK_AL_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_AL_INT);
    }

    if (status & I2C_MASK_RX_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_RX_INT);

        // store read result
        i2c_env.i2cBuffer[i2c_env.i2cIndex++] = i2c_i2c_GetRXD(QN_I2C);
        i2c_env.i2cRxCount--;
        if (i2c_env.i2cRxCount > 1) {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_ACK_SEND;         // ACK
        }
        else if (i2c_env.i2cRxCount == 1) {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_NACK_SEND;        // NACK
        }
        else if (i2c_env.i2cRxCount == 0) {  // data rx finish
            reg = I2C_MASK_STOP;             // STOP
            i2c_env.i2cOpFsm = I2C_OP_FINISH;
        }
        i2c_i2c_SetTXD(QN_I2C, reg);

    }

    if (status & I2C_MASK_TX_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_TX_INT);

        // check ack type
        if (i2c_i2c_GetSR(QN_I2C) & I2C_MASK_ACK_RECEIVED) { // NO ACK
            i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);           // STOP
            i2c_env.i2cOpFsm = I2C_OP_ABORT;
        }
        else { // ACK

            if (i2c_env.i2cOpFsm == I2C_OP_RDDATA) {  // enable data read
                if (i2c_env.i2cRxCount > 1) {
                    reg = I2C_MASK_RD_EN
                        | I2C_MASK_ACK_SEND;          // ACK
                }
                else if (i2c_env.i2cRxCount == 1) {
                    reg = I2C_MASK_RD_EN
                        | I2C_MASK_NACK_SEND;         // NACK
                }
                i2c_i2c_SetTXD(QN_I2C, reg);
            }
            else {
                if (i2c_env.i2cIndex < i2c_env.i2cTxCount) {
                    reg = I2C_MASK_WR_EN
                        | i2c_env.i2cBuffer[i2c_env.i2cIndex++];    // write address buffer
                    i2c_i2c_SetTXD(QN_I2C, reg);
                }
                else { // data tx finish, check need stop or not
                    if (i2c_env.i2cOpFsm == I2C_OP_WRDATA) {
                        i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);      // STOP
                        i2c_env.i2cOpFsm = I2C_OP_FINISH;
                    }
                    else if (i2c_env.i2cOpFsm == I2C_OP_SETADDR) {
                        //i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);
                        i2c_env.i2cOpFsm = I2C_OP_RDDATA;
                    }
                }
            }
        }
    }

}
#endif /* CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE */

#if CONFIG_I2C_ENABLE_INTERRUPT==FALSE
/**
 ****************************************************************************************
 * @brief  i2c polling, deal with master mode only
 * @description
 *  This function is used repeatedly in i2c_read() or i2c_write() in order to
 *  complete a transfer.
 *****************************************************************************************
 */
void i2c_polling(void)
{
    uint32_t status;
    uint32_t reg;

    status = i2c_i2c_GetIntStatus(QN_I2C);
    if (status & I2C_MASK_AL_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_AL_INT);
    }

    if (status & I2C_MASK_RX_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_RX_INT);

        i2c_env.i2cBuffer[i2c_env.i2cIndex++] = i2c_i2c_GetRXD(QN_I2C);
        i2c_env.i2cRxCount--;
        if (i2c_env.i2cRxCount > 1) {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_ACK_SEND;         // ACK
        }
        else if (i2c_env.i2cRxCount == 1) {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_NACK_SEND;        // NACK
        }
        else if (i2c_env.i2cRxCount == 0) {  // data rx finish
            reg = I2C_MASK_STOP;             // STOP
            i2c_env.i2cOpFsm = I2C_OP_FINISH;
        }
        i2c_i2c_SetTXD(QN_I2C, reg);
    }

    if (status & I2C_MASK_TX_INT) {
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_TX_INT);

        // check ack type
        if (i2c_i2c_GetSR(QN_I2C) & I2C_MASK_ACK_RECEIVED) { // NO ACK
            i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);           // STOP
            i2c_env.i2cOpFsm = I2C_OP_ABORT;
        }
        else { // ACK

            if (i2c_env.i2cOpFsm == I2C_OP_RDDATA) {  // enable data read
                if (i2c_env.i2cRxCount > 1) {
                    reg = I2C_MASK_RD_EN
                        | I2C_MASK_ACK_SEND;          // ACK
                }
                else if (i2c_env.i2cRxCount == 1) {
                    reg = I2C_MASK_RD_EN
                        | I2C_MASK_NACK_SEND;         // NACK
                }
                i2c_i2c_SetTXD(QN_I2C, reg);
            }
            else {
                if (i2c_env.i2cIndex < i2c_env.i2cTxCount) {
                    reg = I2C_MASK_WR_EN
                        | i2c_env.i2cBuffer[i2c_env.i2cIndex++];    // write address buffer
                    i2c_i2c_SetTXD(QN_I2C, reg);
                }
                else { // data tx finish, check need stop or not
                    if (i2c_env.i2cOpFsm == I2C_OP_WRDATA) {
                        i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);      // STOP
                        i2c_env.i2cOpFsm = I2C_OP_FINISH;
                    }
                    else if (i2c_env.i2cOpFsm == I2C_OP_SETADDR) {
                        //i2c_i2c_SetTXD(QN_I2C, I2C_MASK_STOP);
                        i2c_env.i2cOpFsm = I2C_OP_RDDATA;
                    }
                }
            }
        }
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Initialize the I2C controller
 * @param[in]    speed      SCL 1K: I2C_SCL_RATIO(1000)
 * @param[in]    buffer     i2c buffer  (point to a gobal memory)
 * @param[in]    size       i2c buffer len, = address size + data size
 * @description
 *  This function is used to initialize I2C in master mode. SCL speed is up to 400KHz. The function is also
 *  used to enable I2c interrupt, and enable NVIC I2C IRQ.
 *****************************************************************************************
 */
void i2c_init(uint32_t speed, uint8_t *buffer, uint16_t size)
{
    uint32_t reg;

    //Configure I2C environment
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 0;
    i2c_env.i2cRxCount = 0;
    i2c_env.i2cBufferSize = size;
    i2c_env.i2cBuffer = buffer;

#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_reset();

#if CONFIG_I2C_ENABLE_INTERRUPT==TRUE

    /* Enable the I2C Interrupt */
    NVIC_EnableIRQ(I2C_IRQn);

    /*
        * Mask all master interrupt in I2C component
        */
    reg = speed                             // I2C scl speed
        | I2C_MASK_MASTR_EN                 // master
        | I2C_MASK_AL_INT_EN                // Enable arbitration loss interrupt
        | I2C_MASK_RX_INT_EN                // Enable RX interrupt
        | I2C_MASK_TX_INT_EN;               // Enable TX interrupt

#else

    reg = speed                             // I2C scl speed
        | I2C_MASK_MASTR_EN;                // master

#endif /* CONFIG_I2C_ENABLE_INTERRUPT==TRUE */

    i2c_i2c_SetCR(QN_I2C, reg);
}

/**
 ****************************************************************************************
 * @brief Check I2C bus is busy or free
 * @return Busy or free
 *****************************************************************************************
 */
enum I2C_BUS_STATE i2c_bus_check( void )
{
    uint32_t timeout = 0;

    do {
        timeout++;
        if ( timeout > I2C_MAX_TIMEOUT ) {
            return I2C_BUS_BUSY;
        }
    } while ( i2c_i2c_GetSR(QN_I2C) & I2C_MASK_BUSY );

    return I2C_BUS_FREE;
}


/**
 ****************************************************************************************
 * @brief Start a data reception.
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @return Error code
 * @description
 * This function is used to complete an I2C read transaction from start to stop. All the intermittent steps
 * are handled in the interrupt handler while the interrupt is enabled.
 * Before this function is called, the read length, write length, I2C master buffer,
 * and I2C state need to be filled. Please refer to I2C_BYTE_READ().
 * As soon as the end of the data transfer is detected, the callback function is called.
 *****************************************************************************************
 */
static enum I2C_ERR_CODE i2c_read(uint8_t saddr)
{
    uint32_t reg;
    uint32_t timeout = 0;

    if (i2c_bus_check() == I2C_BUS_BUSY) {
        return I2C_CONFLICT;
    }

    if (i2c_env.i2cTxCount) {
        i2c_env.i2cOpFsm = I2C_OP_SETADDR;
        // start write slave address with write bit
        reg = I2C_MASK_WR_EN
            | I2C_MASK_START
            | ((saddr << 1) & 0xFE);
        i2c_i2c_SetTXD(QN_I2C, reg);


        do {
            timeout++;
            if (timeout > I2C_MAX_TIMEOUT) {
                return I2C_TIMEOUT;
            }
#if CONFIG_I2C_ENABLE_INTERRUPT==FALSE
            i2c_polling();
#endif

            if (i2c_env.i2cOpFsm == I2C_OP_ABORT) {
                return I2C_NO_ACK;
            }

        } while (i2c_env.i2cOpFsm != I2C_OP_RDDATA);
    }
    else {
        // does not need write address, directly read data from device
        i2c_env.i2cOpFsm = I2C_OP_RDDATA;
    }
    
    // start write slave address with read bit
    reg = I2C_MASK_WR_EN
        | I2C_MASK_START
        | ((saddr << 1) | 0x01);
    i2c_i2c_SetTXD(QN_I2C, reg);

    timeout = 0;
    do {
        timeout++;
        if (timeout > I2C_MAX_TIMEOUT) {
            return I2C_TIMEOUT;
        }
#if CONFIG_I2C_ENABLE_INTERRUPT==FALSE
        i2c_polling();
#endif

        if (i2c_env.i2cOpFsm == I2C_OP_ABORT) {
            return I2C_NO_ACK;
        }

    } while (i2c_env.i2cOpFsm != I2C_OP_FINISH);

#if I2C_CALLBACK_EN==TRUE
    // Call end of reception callback
    if (i2c_env.callback != NULL)
    {
        i2c_env.callback();
    }
#endif

    return I2C_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief Start a data transmission.
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @return Error code
 * @description
 * This function is used to complete an I2C write transaction from start to stop. All the intermittent steps
 * are handled in the interrupt handler while the interrupt is enabled.
 * Before this function is called, the read length, write length, I2C master buffer,
 * and I2C state need to be filled. Please refer to I2C_BYTE_WRITE().
 * As soon as the end of the data transfer is detected, the callback function is called.
 *****************************************************************************************
 */
static enum I2C_ERR_CODE i2c_write(uint8_t saddr)
{
    uint32_t reg;
    uint32_t timeout = 0;

    if (i2c_bus_check() == I2C_BUS_BUSY) {
        return I2C_CONFLICT;
    }
    else {
        i2c_env.i2cOpFsm = I2C_OP_WRDATA;
    }

    // start write slave address with write bit
    reg = I2C_MASK_WR_EN
        | I2C_MASK_START
        | ((saddr << 1) & 0xFE);
    i2c_i2c_SetTXD(QN_I2C, reg);

    do {
        timeout++;
        if (timeout > I2C_MAX_TIMEOUT) {
            return I2C_TIMEOUT;
        }
#if CONFIG_I2C_ENABLE_INTERRUPT==FALSE
        i2c_polling();
#endif

        if (i2c_env.i2cOpFsm == I2C_OP_ABORT) {
            return I2C_NO_ACK;
        }

    } while (i2c_env.i2cOpFsm != I2C_OP_FINISH);

#if I2C_CALLBACK_EN==TRUE
    // Call end of transmission callback
    if (i2c_env.callback != NULL)
    {
        i2c_env.callback();
    }
#endif

    return I2C_NO_ERROR;
}

/**
 ****************************************************************************************
 * @brief Read a byte data form i2c device
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @return     reg_data      read from i2c bus
 * @description
 * Read a byte data from slave device, the data address is 8 bits. If I2C device not need 
 * to specifiy a data address, the input param reg_addr should be set to 0, and i2c_env.i2cTxCount 
 * also should be set to 0
 *****************************************************************************************
 */
uint8_t I2C_BYTE_READ(uint8_t saddr, uint8_t reg_addr)
{
    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 1;
    i2c_env.i2cRxCount = 1;
    i2c_env.i2cBuffer[0] = reg_addr;
    i2c_env.i2cBuffer[1] = 0;   // clear result buffer
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_read(saddr);
    return i2c_env.i2cBuffer[i2c_env.i2cTxCount];
}

/**
 ****************************************************************************************
 * @brief Read a byte data form i2c device
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @return     reg_data      read from i2c bus
 * @description
 * Read a byte data from slave device, the data address is 16 bits
 *
 *****************************************************************************************
 */
uint8_t I2C_BYTE_READ2(uint8_t saddr, uint16_t reg_addr)
{
    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 2;
    i2c_env.i2cRxCount = 1;
    i2c_env.i2cBuffer[0] = (reg_addr >> 8) & 0xFF;
    i2c_env.i2cBuffer[1] = reg_addr & 0xFF;
    i2c_env.i2cBuffer[2] = 0;    // clear result buffer
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_read(saddr);
    return i2c_env.i2cBuffer[i2c_env.i2cTxCount];
}

/**
 ****************************************************************************************
 * @brief Read n byte data form i2c device
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  buffer        Pointer to read data buffer
 * @param[in]  len           read data length
 * @description
 * Read n byte data from slave device, read start address is 8 bits and the data will be
 * stored in buffer, n is the specified length
 *****************************************************************************************
 */
void I2C_nBYTE_READ(uint8_t saddr, uint8_t reg_addr, uint8_t *buffer, uint16_t len)
{
    uint32_t i;

    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 1;
    i2c_env.i2cRxCount = len;
    i2c_env.i2cBuffer[0] = reg_addr;
    for (i = 0; i < len; i++) {
        i2c_env.i2cBuffer[i2c_env.i2cTxCount + i] = 0;  // clear result buffer
    }
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_read(saddr);
    for (i = 0; i < len; i++) {
        buffer[i] = i2c_env.i2cBuffer[i2c_env.i2cTxCount + i];
    }
}

/**
 ****************************************************************************************
 * @brief Read n byte data form i2c device
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  buffer        Pointer to read data buffer
 * @param[in]  len           read data length
 * @description
 * Read n byte data from slave device, read start address is 16 bits and the data will be
 * stored in buffer, n is the specified length
 *****************************************************************************************
 */
void I2C_nBYTE_READ2(uint8_t saddr, uint16_t reg_addr, uint8_t *buffer, uint16_t len)
{
    uint32_t i;

    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 2;
    i2c_env.i2cRxCount = len;
    i2c_env.i2cBuffer[0] = (reg_addr >> 8) & 0xFF;
    i2c_env.i2cBuffer[1] = reg_addr & 0xFF;
    for (i = 0; i < len; i++) {
        i2c_env.i2cBuffer[i2c_env.i2cTxCount + i] = 0;  // clear result buffer
    }
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_read(saddr);
    for (i = 0; i < len; i++) {
        buffer[i] = i2c_env.i2cBuffer[i2c_env.i2cTxCount + i];
    }
}

/**
 ****************************************************************************************
 * @brief Write a byte data to i2c device
 * *
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  reg_data      byte data
 * @description
 *  Write a byte data to a 8 bits address of slave device
 *****************************************************************************************
 */
void I2C_BYTE_WRITE(uint8_t saddr, uint8_t reg_addr, uint8_t reg_data)
{
    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 2;
    i2c_env.i2cRxCount = 0;
    i2c_env.i2cBuffer[0] = reg_addr;
    i2c_env.i2cBuffer[1] = reg_data;
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_write(saddr);
}

/**
 ****************************************************************************************
 * @brief Write a byte data to i2c device
 * *
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  reg_data      byte data
 * @description
 *  Write a byte data to a 16 bits address of slave device
 *****************************************************************************************
 */
void I2C_BYTE_WRITE2(uint8_t saddr, uint16_t reg_addr, uint8_t reg_data)
{
    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 3;
    i2c_env.i2cRxCount = 0;
    i2c_env.i2cBuffer[0] = (reg_addr >> 8) & 0xFF;
    i2c_env.i2cBuffer[1] = reg_addr & 0xFF;
    i2c_env.i2cBuffer[2] = reg_data;
#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_write(saddr);
}

/**
 ****************************************************************************************
 * @brief Write n byte data to i2c device
 * *
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  buffer        pointer to write data
 * @param[in]  len           write data length
 * @description
 *  Write n byte data to slave device. The write starting address is 8 bits. The data is from
 *  the buffer and n is a specified length
 *****************************************************************************************
 */
void I2C_nBYTE_WRITE(uint8_t saddr, uint8_t reg_addr, uint8_t *buffer, uint16_t len)
{
    uint32_t i;

    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 1+len;
    i2c_env.i2cRxCount = 0;
    i2c_env.i2cBuffer[0] = reg_addr;
    for (i = 0; i < len; i++) {
        i2c_env.i2cBuffer[1 + i] = buffer[i];
    }

#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_write(saddr);
}

/**
 ****************************************************************************************
 * @brief Write n byte data to i2c device
 * *
 * @param[in]  saddr         slave device address(7bits, without R/W bit)
 * @param[in]  reg_addr      device register address
 * @param[in]  buffer        pointer to write data
 * @param[in]  len           write data length
 * @description
 *  Write n byte data to slave device. The write starting address is 16 bits. The data is from
 *  the buffer and n is a specified length
 *****************************************************************************************
 */
void I2C_nBYTE_WRITE2(uint8_t saddr, uint16_t reg_addr, uint8_t *buffer, uint16_t len)
{
    uint32_t i;

    //Store environment parameters
    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cIndex = 0;
    i2c_env.i2cTxCount = 2+len;
    i2c_env.i2cRxCount = 0;
    i2c_env.i2cBuffer[0] = (reg_addr >> 8) & 0xFF;
    i2c_env.i2cBuffer[1] = reg_addr & 0xFF;
    for (i = 0; i < len; i++) {
        i2c_env.i2cBuffer[2 + i] = buffer[i];
    }

#if I2C_CALLBACK_EN==TRUE
    i2c_env.callback = NULL;
#endif

    i2c_write(saddr);
}

#else // I2C_SLAVE

#define I2C_MASK_SLV_NACK_SEND                  0x00000000      /* 20 */
#define I2C_MASK_SLV_ACK_SEND                   0x00100000      /* 20 */

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */

///Structure defining I2C environment parameters
struct i2c_env_tag
{
    int16_t             i2cIndex;
    uint16_t            i2cBufferSize;
    uint8_t             *i2cBuffer;
    enum I2C_OP_FSM     i2cOpFsm;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
///I2C environment variable
static struct i2c_env_tag i2c_env;

/**
 ****************************************************************************************
 * @brief Initializes the I2C controller
 * @param[in]    buffer     i2c buffer (point to a gobal memory)
 * @param[in]    size       i2c buffer len, = address size + data size
 * @description
 *  The function is used to initialize I2C in slave mode, this function is also
 *  used to enable I2c interrupt, and enable NVIC I2C IRQ.
 *****************************************************************************************
 */
void i2c_init(uint8_t *buffer, uint16_t size)
{
    uint32_t reg;

    i2c_env.i2cOpFsm = I2C_OP_IDLE;
    i2c_env.i2cBufferSize = size;
    i2c_env.i2cBuffer = buffer;

    i2c_reset();

#if CONFIG_I2C_ENABLE_INTERRUPT==TRUE
    /* Enable the I2C Interrupt */
    NVIC_EnableIRQ(I2C_IRQn);
#endif

    /*
        * Mask all slave interrupt in I2C component
        */
    reg = I2C_SLAVE_ADDR(QN9020_I2C_ADDR)   // slave address, slave mode active
        | I2C_MASK_SLAVE_EN                 // slave
        | I2C_MASK_STP_INT_EN               // Enable abnormal stop interrupt
        | I2C_MASK_SAM_INT_EN               // Enable slave address match interrupt, slave mode active
        | I2C_MASK_GC_INT_EN                // Enable general call interrupt, slave mode active
        | I2C_MASK_RX_INT_EN                // Enable RX interrupt
        | I2C_MASK_TX_INT_EN;               // Enable TX interrupt
    i2c_i2c_SetCR(QN_I2C, reg);

    reg = I2C_MASK_RD_EN
        | I2C_MASK_SLV_ACK_SEND;            //ACK
    i2c_i2c_SetTXD(QN_I2C, reg);

}

#if CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE
/**
 ****************************************************************************************
 * @brief I2C interrupt handler, deal with slave mode only.
 ****************************************************************************************
 */
void I2C_IRQHandler(void)
{
    uint32_t status;
    uint32_t reg;

    status = i2c_i2c_GetIntStatus(QN_I2C);
    if (status & I2C_MASK_STP_INT) {

        reg = I2C_MASK_RD_EN
            | I2C_MASK_SLV_ACK_SEND;     // ACK
        i2c_i2c_SetTXD(QN_I2C, reg);
        i2c_env.i2cOpFsm = I2C_OP_IDLE;

        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_STP_INT);
    }

    if (status & I2C_MASK_GC_INT) {

        reg = I2C_MASK_RD_EN
            | I2C_MASK_SLV_ACK_SEND;     // ACK
        i2c_i2c_SetTXD(QN_I2C, reg);
        i2c_env.i2cOpFsm = I2C_OP_IDLE;

        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_GC_INT);
    }

    if (status & I2C_MASK_SAM_INT) { // slave adress is match

        reg = i2c_i2c_GetRXD(QN_I2C);
        if (reg & 0x01) { // master read, slave write

            reg = I2C_MASK_WR_EN
                | I2C_MASK_SLV_ACK_SEND     // ACK
                | i2c_env.i2cBuffer[i2c_env.i2cIndex++];
            i2c_env.i2cOpFsm = I2C_OP_WRDATA;
        }
        else { // mast write, slave read

            reg = I2C_MASK_RD_EN
                | I2C_MASK_SLV_ACK_SEND;     // ACK
            i2c_env.i2cOpFsm = I2C_OP_SETADDR;
        }
        i2c_i2c_SetTXD(QN_I2C, reg);
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_SAM_INT|I2C_MASK_RX_INT);
        status = 0;
    }

    if (status & I2C_MASK_RX_INT) {
        if (i2c_env.i2cOpFsm == I2C_OP_SETADDR) {
            i2c_env.i2cIndex = i2c_i2c_GetRXD(QN_I2C);  // The 1st byte is the index.
            i2c_env.i2cOpFsm = I2C_OP_RDDATA;
        }
        else if (i2c_env.i2cOpFsm == I2C_OP_RDDATA) {
            i2c_env.i2cBuffer[i2c_env.i2cIndex++] = i2c_i2c_GetRXD(QN_I2C);
        }

        if (i2c_env.i2cIndex >= i2c_env.i2cBufferSize) {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_SLV_NACK_SEND;     // NACK
        }
        else {
            reg = I2C_MASK_RD_EN
                | I2C_MASK_SLV_ACK_SEND;     // ACK
        }
        i2c_i2c_SetTXD(QN_I2C, reg);
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_RX_INT);
    }

    if (status & I2C_MASK_TX_INT) {
        if (!(i2c_i2c_GetSR(QN_I2C) & I2C_MASK_ACK_RECEIVED)) { // ACK == 0,  go on

            if (i2c_env.i2cIndex >= i2c_env.i2cBufferSize) {  // user can modify here
                i2c_env.i2cIndex = 0;
            }

            reg = I2C_MASK_WR_EN
                | I2C_MASK_SLV_ACK_SEND     // ACK
                | i2c_env.i2cBuffer[i2c_env.i2cIndex++];
        }
        else { // NO ACK, SLAVE back to RD
            reg = I2C_MASK_RD_EN
                | I2C_MASK_SLV_ACK_SEND;     // ACK
        }

        i2c_i2c_SetTXD(QN_I2C, reg);
        i2c_i2c_ClrIntStatus(QN_I2C, I2C_MASK_TX_INT);
    }
}
#endif  /* CONFIG_I2C_DEFAULT_IRQHANDLER==TRUE */
#endif /* I2C_MODE == I2C_SLAVE */


#endif /* CONFIG_ENABLE_DRIVER_I2C==TRUE */
/// @} I2C
