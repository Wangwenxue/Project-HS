/**
 ****************************************************************************************
 *
 * @file spi.h
 *
 * @brief Header file of SPI for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.1 $
 *
 ****************************************************************************************
 */
#ifndef _SPI_H_
#define _SPI_H_
#include "driver_config.h"
#if (CONFIG_ENABLE_DRIVER_SPI0==TRUE || CONFIG_ENABLE_DRIVER_SPI1==TRUE)
#include "syscon.h"
#include "sleep.h"

/**
 ****************************************************************************************
 * @defgroup SPI SPI Driver
 * @ingroup DRIVERS
 * @brief SPI driver
 *
 *  The Serial Peripheral Interface (SPI) is a synchronous serial data communication protocol
 *  which operates in full duplex mode. Devices communicate in master/slave mode with 4-wire
 *  bi-direction interface. QN9020 contains 2 sets of SPI controller performing a serial-to-parallel
 *  conversion on data received from a peripheral device, and a parallel-to-serial conversion on
 *  data transmitted to a peripheral device. Each SPI set can drive up to 2 external peripherals.
 *  It also can be driven as the slave device when the slave mode is enabled.
 *
 *  Each controller can generate an individual interrupt signal when data transfer buffer is empty or
 *  receive buffer is full. Writing a divisor into DIVIDER register can program the frequency of serial 
 *  clock output when it is as the master.
 *
 *  The main features of SPI are listed as follow:
 *   - 2 sets of SPI controller
 *   - Support master/slave mode operation
 *   - 2 slave/device select lines in the master mode
 *   - Variable output serial clock frequency in master mode
 *   - SPI mode 0/1/2/3 configurable
 *   - 8 bit/ 32 bit data width configurable
 *   - MSB or LSB first data transfer
 *   - 4 bytes TX & RX synchronies FIFO
 *   - Both TX & RX DMA request
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example spi_example.c
 * This is an example of how to use the SPI driver.
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */


/*
 * DEFINES
 ****************************************************************************************
 */
/// SPI SS configure
#define SPI_SSx_CFG                     SPI_MASK_MSTR_SS0
#define SPI_BITORDER_CFG                SPI_MSB_FIRST

/// Dummy byte
//#define SPI_DUMMY_DATA                  (0x0)
#define SPI_DUMMY_DATA                  (0xFFFFFFFF)

/// SPI bit rate algorithm, x should less than (__USART_CLK/4)
#define SPI_BITRATE(x)                  ((__USART_CLK/(2*(x)) - 1) << SPI_POS_CLK_DIV_MASTER)
// SPI bit rate algorithm (usart divider bypass)
//#define SPI_BITRATE(x)                  ((g_AhbClock/(2*(x)) - 1) << SPI_POS_CLK_DIV_MASTER)
// SPI bit rate algorithm (usart divider not bypass)
//#define SPI_BITRATE(div,x)              ((USARTx_CLK(div)/(2*(x)) - 1) << SPI_POS_CLK_DIV_MASTER)

/// SPI0 mode confiure: TRUE(3-wire: CS, CLK, DAT(I/O)), FALSE(4-wire: CS, CLK, DIN(I), DAT(O))
#define SPI0_MOD_3WIRE_EN               FALSE
/// SPI1 mode confiure: TRUE(3-wire: CS, CLK, DAT(I/O)), FALSE(4-wire: CS, CLK, DIN(I), DAT(O))
#define SPI1_MOD_3WIRE_EN               FALSE


#if SPI_DMA_EN==TRUE
// SPI TX DMA enable, DMA only have one channel
#define SPI_TX_DMA_EN                   TRUE
// SPI RX DMA enable, only support in SLAVE mode, DMA only have one channel
#define SPI_RX_DMA_EN                   FALSE
#else
// SPI TX DMA enable, DMA only have one channel
#define SPI_TX_DMA_EN                   FALSE
// SPI RX DMA enable, only support in SLAVE mode, DMA only have one channel
#define SPI_RX_DMA_EN                   FALSE
#endif

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */


///SPI SCK polarity
enum SPI_POLARITY
{
    SPI_CPOL_0 = 0,         /*!< Set SPI SCK to low level at IDLE */
    SPI_CPOL_1 = 1          /*!< Set SPI SCK to high level at IDLE */
};

///SPI SCK phase
enum SPI_PHASE
{
    SPI_CPHA_0 = 0,                     /*!< Set SCK phase, 1ST EDGE */
    SPI_CPHA_1 = (1 << SPI_POS_PHASE)   /*!< Set SCK phase, 2ND EDGE */
};

///SPI module mode: master or slave
enum SPI_MODE
{
    SPI_MASTER_MOD = (0 << SPI_POS_MODE),   /*!< Set SPI mode to master mode */
    SPI_SLAVE_MOD = (1 << SPI_POS_MODE)     /*!< Set SPI mode to slave mode*/
};

///SPI Bit ordering
enum SPI_BIT_ORDERING
{
    SPI_LSB_FIRST = 0,                          /*!< Send LSB first */
    SPI_MSB_FIRST = (1 << SPI_POS_BIT_ORDERING) /*!< Send MSB first */
};

///SPI buffer width
enum SPI_BUFFER_WIDTH
{
    SPI_8BIT = 0,                               /*!< Set SPI buffer width is 8 bits */
    SPI_32BIT = (1 << SPI_POS_BUFFER_WIDTH)     /*!< Set SPI buffer width is 32 bits */
};

///SPI byte endian
enum SPI_BYTE_ENDIAN
{
    SPI_BIG_ENDIAN = 0,                             /*!< Set SPI as big endian mode */
    SPI_LITTLE_ENDIAN = (1 << SPI_POS_BYTE_ENDIAN)  /*!< Set SPI as little endian mode */
};

/// SPI TX status
enum SPI_TX_STATE
{
    SPI_TX_BUF_BUSY,           /*!< SPI TX busy */
    SPI_LAST_BYTE_ONGOING,     /*!< SPI last byte ongoing */
    SPI_TX_FREE                /*!< SPI Tx free */
};

/// SPI interrupt type
enum SPI_INT_TYPE
{
    SPI_TX_INT = SPI_MASK_TX_FIFO_NFUL_IE,       /*!< SPI TX interrupt */
    SPI_RX_INT = SPI_MASK_RX_FIFO_NEMT_IE        /*!< SPI RX interrupt */
};



/*
 * STRUCT DEFINITIONS
 *****************************************************************************************
 */

/// SPI channel parameters, holding data used for synchronous R/W data transactions
struct spi_txrxchannel
{
    int32_t  size;                      /*!< Transmission size */
    uint8_t  *bufptr;                   /*!< Data buffer of TX or RX */
    #if SPI_CALLBACK_EN==TRUE
    void     (*callback)(void);         /*!< Callback at the end of transmission */
    #endif
};

///Structure defining SPI environment parameters
struct spi_env_tag
{
    enum SPI_MODE mode;                 /*!< SPI work mode */
    enum SPI_BUFFER_WIDTH width;        /*!< SPI transmission width */
    struct spi_txrxchannel tx;          /*!< Instance of TX */
    struct spi_txrxchannel rx;          /*!< Instance of RX */
};

#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
///SPI0 environment variable
extern volatile struct spi_env_tag spi0_env;
#endif
#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
///SPI1 environment variable
extern volatile struct spi_env_tag spi1_env;
#endif

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Enable SPI module clock.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @description
 *  This function is used to enable SPI module clock
 *****************************************************************************************
 */
__STATIC_INLINE void spi_clock_on(QN_SPI_TypeDef *SPI)
{
#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if (SPI == QN_SPI0) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI0);
    }
#endif

#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if (SPI == QN_SPI1) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_SPI1);
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Disable SPI module clock
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @description
 *  This function is used to disable SPI module clock
 *****************************************************************************************
 */
__STATIC_INLINE void spi_clock_off(QN_SPI_TypeDef *SPI)
{

    // Wait until the TX buffer is empty
    while (!( spi_spi_GetSR(SPI) & SPI_MASK_TX_FIFO_EMPT )); 

    // Wait until the Busy bit is cleared
    while ( spi_spi_GetSR(SPI) & SPI_MASK_BUSY );

#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if (SPI == QN_SPI0) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_SPI0); 
    }
#endif

#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if (SPI == QN_SPI1) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_SPI1); 
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Send data to SPI.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @param[in]       spi_env       Environment variable of specified SPI port
 * @description
 *  Send a byte/word data to SPI
 *****************************************************************************************
 */
__STATIC_INLINE void spi_tx_data(QN_SPI_TypeDef * SPI, struct spi_env_tag *spi_env)
{
    __set_PRIMASK(1);
    union {
        uint32_t u32reg;
        uint8_t  u8array[4];
    } tx_dat;

    if (spi_env->width == SPI_32BIT) {  // little endian
        for (int i = 0; i < 4; i++) {
            tx_dat.u8array[i] = *spi_env->tx.bufptr++;
        }
        spi_env->tx.size -= 4;
    }
    else {
        tx_dat.u32reg = *spi_env->tx.bufptr++;
        spi_env->tx.size--;
    }
    spi_spi_SetTXD(SPI, tx_dat.u32reg);
    __set_PRIMASK(0);
}

/**
 ****************************************************************************************
 * @brief Get data form SPI.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @param[in]       spi_env       Environment variable of specified SPI port
 * @description
 *  Receive a byte/word data from SPI.
 *****************************************************************************************
 */
__STATIC_INLINE void spi_rx_data(QN_SPI_TypeDef * SPI, struct spi_env_tag *spi_env)
{
    __set_PRIMASK(1);
    union {
        uint32_t u32reg;
        uint8_t  u8array[4];
    } rx_dat;
    
    rx_dat.u32reg = spi_spi_GetRXD(SPI);
    if (spi_env->width == SPI_32BIT) {
        for (int i = 0; i < 4; i++) {  // little endian
            *spi_env->rx.bufptr++ = rx_dat.u8array[i];
        }
        spi_env->rx.size -= 4;
    }
    else {
        *spi_env->rx.bufptr++ = rx_dat.u8array[0];
        spi_env->rx.size--;
    }
    __set_PRIMASK(0);
}

/**
 ****************************************************************************************
 * @brief Enable/Disable SPI interrupt.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @param[in]       type          Interrupt type: refer to enum SPI_INT_TYPE
 * @param[in]       able          MASK_ENABLE or MASK_DISABLE
 * @description
 *  Enable or disable specified SPI interrupt
 *****************************************************************************************
 */
__STATIC_INLINE void spi_int_enable(QN_SPI_TypeDef *SPI, uint32_t type, uint32_t able)
{
    uint32_t dev = 0;
    uint32_t mask = type;
    struct spi_env_tag *spi_env;

    if (SPI == QN_SPI0) {
#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
        
        spi_env = (struct spi_env_tag *)&spi0_env;

        #if SPI_RX_ACTIVE_BIT_EN==TRUE
        if (type & SPI_RX_INT) {
            dev |= PM_MASK_SPI0_RX_ACTIVE_BIT;
        }
        #endif

        if (type & SPI_TX_INT) {
            dev |= PM_MASK_SPI0_TX_ACTIVE_BIT;
        }
#endif
    }
    else {
#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
        
        spi_env = (struct spi_env_tag *)&spi1_env;

        #if SPI_RX_ACTIVE_BIT_EN==TRUE
        if (type & SPI_RX_INT) {
            dev |= PM_MASK_SPI1_RX_ACTIVE_BIT;
        }
        #endif

        if (type & SPI_TX_INT) {
            dev |= PM_MASK_SPI1_TX_ACTIVE_BIT;
        }
#endif
    }

    if (able) {
        dev_prevent_sleep(dev);
        
        if (!(spi_spi_GetCR0(SPI) & SPI_MASK_TX_FIFO_NFUL_IE)) { // if TX interrupt not enabled
            if ((spi_env->tx.size > 0) && (spi_spi_GetSR(SPI) & SPI_MASK_TX_FIFO_NFUL_IF)) {
                spi_tx_data(SPI, spi_env);
            }
            else if ((spi_env->mode == SPI_MASTER_MOD) && (spi_env->rx.size > 0)) {
                spi_spi_SetTXD(SPI, SPI_DUMMY_DATA);
            }
        }
    }
    else {
        dev_allow_sleep(dev);
    }

    spi_spi_SetCR0WithMask(SPI, mask, able);
}

#if (CONFIG_ENABLE_DRIVER_UART0==FALSE && CONFIG_ENABLE_DRIVER_UART1==FALSE)
/**
 ****************************************************************************************
 * @brief   Reset USART module (UART&SPI)
 * @param[in]       usart          QN_UART0 / QN_UART1 / QN_SPI0 / QN_SPI1
 * @description
 *  This function is used to reset USART module (include UART and SPI)
 *
 *****************************************************************************************
 */
__STATIC_INLINE void usart_reset(uint32_t usart)
{
#if (CONFIG_ENABLE_DRIVER_UART0==TRUE || CONFIG_ENABLE_DRIVER_SPI0==TRUE)
    if ((usart == (uint32_t)QN_UART0) || (usart == (uint32_t)QN_SPI0)) {
        // Reset UART0 and SPI0 module
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_USART0_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_USART0_RST);
    }
#endif
#if (CONFIG_ENABLE_DRIVER_UART1==TRUE || CONFIG_ENABLE_DRIVER_SPI1==TRUE)
    if ((usart == (uint32_t)QN_UART1) || (usart == (uint32_t)QN_SPI1)) {
        // Reset UART1 and SPI1 module
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_USART1_RST);
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_USART1_RST);
    }
#endif
}
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_SPI0==TRUE && CONFIG_SPI0_DEFAULT_IRQHANDLER==TRUE)
void SPI0_IRQHandler(void);
#endif

#if (CONFIG_ENABLE_DRIVER_SPI1==TRUE && CONFIG_SPI1_DEFAULT_IRQHANDLER==TRUE)
void SPI1_IRQHandler(void);
#endif

extern void spi_init(QN_SPI_TypeDef * SPI, uint32_t bitrate, enum SPI_BUFFER_WIDTH width, enum SPI_MODE mode);
extern void spi_read(QN_SPI_TypeDef *SPI, uint8_t *bufptr, int32_t size, void (*rx_callback)(void));
extern void spi_write(QN_SPI_TypeDef *SPI,  uint8_t *bufptr, int32_t size, void (*tx_callback)(void));
extern int spi_check_tx_free(QN_SPI_TypeDef *SPI);


/// @} SPI
#endif /* CONFIG_ENABLE_DRIVER_SPI==TRUE */
#endif /* _SPI_H_ */
