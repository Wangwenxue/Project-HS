/**
 ****************************************************************************************
 *
 * @file uart.h
 *
 * @brief Header file of UART for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _UART_H_
#define _UART_H_
#include "driver_config.h"
#if (CONFIG_ENABLE_DRIVER_UART0==TRUE || CONFIG_ENABLE_DRIVER_UART1==TRUE)
#include "syscon.h"
#include "sleep.h"
#ifdef BLE_PRJ
#include "app_config.h"
#endif

/**
 ****************************************************************************************
 * @defgroup UART UART Driver
 * @ingroup DRIVERS
 * @brief UART driver
 *
 *  The Universal Asynchronous Receiver/Transmitter (UART) performs a serial-to-parallel
 *  conversion on data characters received from the peripheral, and a parallel-to-serial
 *  conversion on data characters received from the CPU. QN9020 UART is an AMBA slave module
 *  that connects to the Advanced Peripheral Bus (APB), and the features are listed as fellow:
 *    - Compliance to the AMBA specification (Rev 2.0, APB4)
 *    - Configurable full-duplex or half-duplex data transmission
 *    - Configurable hardware flow control with nRTS and nCTS option
 *    - Receive and transmit data buffer are supported (only one depth)
 *    - Configurable over-sampling rate (8 or 16)
 *    - Programmable baud rate generator, baud rates up to 2MHz if 16MHz UART clock is adopted
 *    - Full programmable serial interface characteristics:
 *      - Data width support 8bit
 *      - Odd, even or no-parity bit generation and detection
 *      - 1 or 2 stop bit generation
 *      - Configurable LSB- or MSB-first transfer
 *    - Parity¡¢overrun and framing error detection
 *    - Transmit and receive interrupts
 *    - Support for Direct Memory Access(DMA)
 *    - Line-break generation and detection
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example uart_example.c
 * This is an example of how to use the UART driver.
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/// USARTx_CLK = AHB_CLK / (2*(USARTx_DIVIDER + 1))
#define USARTx_CLK(div)                  __USART_CLK
//#define USARTx_CLK(div)                  g_AhbClock                      // default bypass usart divider
//#define USARTx_CLK(div)                  (g_AhbClock / (2*((div) + 1)))    // not bypass usart divider

#if UART_DMA_EN==TRUE
// UART TX DMA enable, DMA only have one channel
#define UART_TX_DMA_EN                  FALSE
// UART RX DMA enable, DMA only have one channel
#define UART_RX_DMA_EN                  TRUE
#else
// UART TX DMA enable, DMA only have one channel
#define UART_TX_DMA_EN                  FALSE
// UART RX DMA enable, DMA only have one channel
#define UART_RX_DMA_EN                  FALSE
#endif

/*
 * ENUMERATION DEFINITIONS
 ****************************************************************************************
 */


/// UART oversample type
enum UART_OVERSAMPLE_TYPE
{
    UART_OVS8 = 0,                  /*!< Set oversampling is 8 */
    UART_OVS16 = UART_MASK_OVS      /*!< Set oversampling is 16 */
};

/// UART buadrate
enum UART_BAUDRATE
{
#if UART_BAUDRATE_TABLE_EN == TRUE
#if __USART_CLK == 32000000UL
    UART_4800     = 0,     /*!< Set baud rate to 4800 when UART clock is 32MHz */
    UART_9600     = 1,     /*!< Set baud rate to 9600 when UART clock is 32MHz */
    UART_19200    = 2,     /*!< Set baud rate to 19200 when UART clock is 32MHz */
    UART_38400    = 3,     /*!< Set baud rate to 38400 when UART clock is 32MHz */
    UART_57600    = 4,     /*!< Set baud rate to 57600 when UART clock is 32MHz */
    UART_76800    = 5,     /*!< Set baud rate to 76800 when UART clock is 32MHz */
    UART_115200   = 6,     /*!< Set baud rate to 115200 when UART clock is 32MHz */
    UART_153600   = 7,     /*!< Set baud rate to 153600 when UART clock is 32MHz */
    UART_230400   = 8,     /*!< Set baud rate to 230400 when UART clock is 32MHz */
    UART_256000   = 9,     /*!< Set baud rate to 256000 when UART clock is 32MHz */
    UART_307200   = 10,    /*!< Set baud rate to 307200 when UART clock is 32MHz */
    UART_460800   = 11,    /*!< Set baud rate to 460800 when UART clock is 32MHz */
    UART_512000   = 12,    /*!< Set baud rate to 512000 when UART clock is 32MHz */
    UART_921600   = 13,    /*!< Set baud rate to 921600 when UART clock is 32MHz */
    UART_1382400  = 14,    /*!< Set baud rate to 1382400 when UART clock is 32MHz */
    UART_1843200  = 15,    /*!< Set baud rate to 1843200 when UART clock is 32MHz */
    UART_2000000  = 16,    /*!< Set baud rate to 2000000 when UART clock is 32MHz */
#elif __USART_CLK == 16000000UL
    UART_2400     = 0,     /*!< Set baud rate to 2400 when UART clock is 16MHz */
    UART_4800     = 1,     /*!< Set baud rate to 4800 when UART clock is 16MHz */
    UART_9600     = 2,     /*!< Set baud rate to 9600 when UART clock is 16MHz */
    UART_19200    = 3,     /*!< Set baud rate to 19200 when UART clock is 16MHz */
    UART_28800    = 4,     /*!< Set baud rate to 28800 when UART clock is 16MHz */
    UART_38400    = 5,     /*!< Set baud rate to 38400 when UART clock is 16MHz */
    UART_57600    = 6,     /*!< Set baud rate to 57600 when UART clock is 16MHz */
    UART_76800    = 7,     /*!< Set baud rate to 76800 when UART clock is 16MHz */
    UART_115200   = 8,     /*!< Set baud rate to 115200 when UART clock is 16MHz */
    UART_128000   = 9,     /*!< Set baud rate to 128000 when UART clock is 16MHz */
    UART_153600   = 10,    /*!< Set baud rate to 153600 when UART clock is 16MHz */
    UART_230400   = 11,    /*!< Set baud rate to 230400 when UART clock is 16MHz */
    UART_256000   = 12,    /*!< Set baud rate to 256000 when UART clock is 16MHz */
    UART_460800   = 13,    /*!< Set baud rate to 460800 when UART clock is 16MHz */
    UART_691200   = 14,    /*!< Set baud rate to 691200 when UART clock is 16MHz */
    UART_921600   = 15,    /*!< Set baud rate to 921600 when UART clock is 16MHz */
    UART_1000000  = 16,    /*!< Set baud rate to 1000000 when UART clock is 16MHz */
#else // default: __USART_CLK == 8000000UL
    UART_1200     = 0,     /*!< Set baud rate to 1200 when UART clock is 8MHz */
    UART_2400     = 1,     /*!< Set baud rate to 2400 when UART clock is 8MHz */
    UART_4800     = 2,     /*!< Set baud rate to 4800 when UART clock is 8MHz */
    UART_9600     = 3,     /*!< Set baud rate to 9600 when UART clock is 8MHz */
    UART_14400    = 4,     /*!< Set baud rate to 14400 when UART clock is 8MHz */
    UART_19200    = 5,     /*!< Set baud rate to 19200 when UART clock is 8MHz */
    UART_28800    = 6,     /*!< Set baud rate to 28800 when UART clock is 8MHz */
    UART_38400    = 7,     /*!< Set baud rate to 38400 when UART clock is 8MHz */
    UART_57600    = 8,     /*!< Set baud rate to 57600 when UART clock is 8MHz */
    UART_64000    = 9,     /*!< Set baud rate to 64000 when UART clock is 8MHz */
    UART_76800    = 10,    /*!< Set baud rate to 76800 when UART clock is 8MHz */
    UART_115200   = 11,    /*!< Set baud rate to 115200 when UART clock is 8MHz */
    UART_128000   = 12,    /*!< Set baud rate to 128000 when UART clock is 8MHz */
    UART_230400   = 13,    /*!< Set baud rate to 230400 when UART clock is 8MHz */
    UART_345600   = 14,    /*!< Set baud rate to 345600 when UART clock is 8MHz */
    UART_460800   = 15,    /*!< Set baud rate to 460800 when UART clock is 8MHz */
    UART_500000   = 16,    /*!< Set baud rate to 500000 when UART clock is 8MHz */
#endif
    UART_BAUD_MAX = 17

#else
    UART_1200     = 1200,       /*!< Set baud rate to 1200 */
    UART_2400     = 2400,       /*!< Set baud rate to 2400 */
    UART_4800     = 4800,       /*!< Set baud rate to 4800 */
    UART_9600     = 9600,       /*!< Set baud rate to 9600 */
    UART_14400    = 14400,      /*!< Set baud rate to 14400 */
    UART_19200    = 19200,      /*!< Set baud rate to 19200 */
    UART_28800    = 28800,      /*!< Set baud rate to 28800 */
    UART_38400    = 38400,      /*!< Set baud rate to 38400 */
    UART_57600    = 57600,      /*!< Set baud rate to 57600 */
    UART_64000    = 64000,      /*!< Set baud rate to 64000 */
    UART_76800    = 76800,      /*!< Set baud rate to 76800 */
    UART_115200   = 115200,     /*!< Set baud rate to 115200 */
    UART_128000   = 128000,     /*!< Set baud rate to 128000 */
    UART_230400   = 230400,     /*!< Set baud rate to 230400 */
    UART_345600   = 345600,     /*!< Set baud rate to 345600 */
    UART_460800   = 460800,     /*!< Set baud rate to 460800 */
    UART_500000   = 500000,     /*!< Set baud rate to 500000 */
    UART_691200   = 691200,     /*!< Set baud rate to 691200 */
    UART_921600   = 921600,     /*!< Set baud rate to 921600 */
    UART_1000000  = 1000000,    /*!< Set baud rate to 1000000 */
    UART_1382400  = 1382400,    /*!< Set baud rate to 1382400 */
    UART_1843200  = 1843200,    /*!< Set baud rate to 1843200 */
    UART_2000000  = 2000000,    /*!< Set baud rate to 2000000 */
#endif
};

/// UART TX status
enum UART_TX_STATE
{
    UART_TX_BUF_BUSY,           /*!< Uart TX busy */
    UART_LAST_BYTE_ONGOING,     /*!< Uart last byte ongoing */
    UART_TX_FREE                /*!< Uart Tx free */
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Get one byte form UART.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @return          uint8_t       One byte data
 * @description
 * Receive 1 byte data from specified UART FIFO.
 *****************************************************************************************
 */
__STATIC_INLINE uint8_t uart_read_one_byte(QN_UART_TypeDef *UART)
{
    // As long as Receive FIFO is not empty, I can always receive
    while ( !(uart_uart_GetIntFlag(UART) & UART_MASK_RX_IF) );
    return (uart_uart_GetRXD(UART));
}

/**
 ****************************************************************************************
 * @brief Send one byte to UART.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       data          data which want to send
 * @description
 * Send 1 byte data from UART
 *****************************************************************************************
 */
__STATIC_INLINE void uart_write_one_byte(QN_UART_TypeDef *UART, uint8_t data)
{
    // Move on only if TX FIFO is empty
    while ( !(uart_uart_GetIntFlag(UART) & UART_MASK_TX_IF) );
    uart_uart_SetTXD(UART, data);
    // Wait until the Busy bit is cleared
    while ( uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY );
}

/**
 ****************************************************************************************
 * @brief Enable/Disable UART RX.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       able          MASK_ENABLE or MASK_DISABLE
 * @description
 * Enable or disable specified UART RX port
 *****************************************************************************************
 */
__STATIC_INLINE void uart_rx_enable(QN_UART_TypeDef *UART, uint32_t able)
{
#if UART_RX_ACTIVE_BIT_EN==TRUE
    uint32_t dev = 0;

    if (UART == QN_UART0) {
        dev = PM_MASK_UART0_RX_ACTIVE_BIT;
    }
    else {
        dev = PM_MASK_UART1_RX_ACTIVE_BIT;
    }
        
    if(able == MASK_DISABLE)
    {
        dev_allow_sleep(dev);
    }
#endif

    if (able == MASK_DISABLE) {
        while ( uart_uart_GetIntFlag(UART) & UART_MASK_RX_BUSY);
    }
    uart_uart_SetCRWithMask(UART, UART_MASK_RX_EN, able);
}

/**
 ****************************************************************************************
 * @brief Enable/Disable UART TX.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       able          MASK_ENABLE or MASK_DISABLE
 * @description
 * Enable or disable specified UART TX port
 *****************************************************************************************
 */
__STATIC_INLINE void uart_tx_enable(QN_UART_TypeDef *UART, uint32_t able)
{
    uint32_t dev = 0;
    
    if (UART == QN_UART0) {
        dev = PM_MASK_UART0_TX_ACTIVE_BIT;
    }
    else {
        dev = PM_MASK_UART1_TX_ACTIVE_BIT;
    }
        
    if(able == MASK_DISABLE)
    {
        dev_allow_sleep(dev);
    }
    
    if (able == MASK_DISABLE) {
        while ( uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY);
    }
    uart_uart_SetCRWithMask(UART, UART_MASK_TX_EN, able);
}

#if UART_RX_DMA_EN==FALSE
/**
 ****************************************************************************************
 * @brief Enable/Disable all UART RX interrupt.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       able          MASK_ENABLE or MASK_DISABLE
 * @description
 * Enable or disable specified UART RX interrupt
 *****************************************************************************************
 */
__STATIC_INLINE void uart_rx_int_enable(QN_UART_TypeDef *UART, uint32_t able)
{
    uint32_t reg;
#if UART_RX_ACTIVE_BIT_EN==TRUE
    uint32_t dev = 0;

    if (UART == QN_UART0) {
        dev = PM_MASK_UART0_RX_ACTIVE_BIT;
    }
    else {
        dev = PM_MASK_UART1_RX_ACTIVE_BIT;
    }
        
    if(able == MASK_DISABLE)
    {
        dev_allow_sleep(dev);
    }
    else {
        dev_prevent_sleep(dev);
    }
#endif

    /*
     * Mask all RX interrupt in UART component
     */
    reg = UART_MASK_BE_IE           // Enable Break error interrupt
        | UART_MASK_PE_IE           // Enable Parity error interrupt
        | UART_MASK_FE_IE           // Enable Framing error interrupt
        | UART_MASK_OE_IE           // Enable Overrun error interrupt
        | UART_MASK_RX_IE;          // Enable RX buffer not empty interrupt
    uart_uart_SetCRWithMask(UART, reg, able);
}
#endif

#if UART_TX_DMA_EN==FALSE
/**
 ****************************************************************************************
 * @brief Enable/Disable UART TX interrupt.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       able          MASK_ENABLE or MASK_DISABLE
 * @description
 * Enable or disable specified UART TX interrupt
 *****************************************************************************************
 */
__STATIC_INLINE void uart_tx_int_enable(QN_UART_TypeDef *UART, uint32_t able)
{
    uint32_t dev = 0;

    if (UART == QN_UART0) {
        dev = PM_MASK_UART0_TX_ACTIVE_BIT;
    }
    else {
        dev = PM_MASK_UART1_TX_ACTIVE_BIT;
    }
        
    if(able == MASK_DISABLE)
    {
        dev_allow_sleep(dev);
    }
    else {
        dev_prevent_sleep(dev);
    }
    
    /*
     * Mask all TX interrupt in UART component
     */
    uart_uart_SetCRWithMask(UART, UART_MASK_TX_IE, able); // Enable TX buffer empty interrupt
}
#endif

/**
 ****************************************************************************************
 * @brief Enable UART module clock.
 * @param[in]       UART           QN_UART0 or QN_UART1
 * @description
 *  This function is used to enable UART module clock
 *****************************************************************************************
 */
__STATIC_INLINE void uart_clock_on(QN_UART_TypeDef *UART)
{
#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if (UART == QN_UART0) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_UART0);
    }
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if (UART == QN_UART1) {
        syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_UART1);
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Disable UART module clock
 * @param[in]       UART           QN_UART0 or QN_UART1
 * @description
 *  This function is used to disable UART module clock
 *****************************************************************************************
 */
__STATIC_INLINE void uart_clock_off(QN_UART_TypeDef *UART)
{
    // Wait until the Busy bit is cleared
    while ( uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY );

#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if (UART == QN_UART0) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_UART0); 
    }
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if (UART == QN_UART1) {
        syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_UART1); 
    }
#endif
}

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

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_TX_DEFAULT_IRQHANDLER==TRUE)
void UART0_TX_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_RX_DEFAULT_IRQHANDLER==TRUE)
void UART0_RX_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_TX_DEFAULT_IRQHANDLER==TRUE)
void UART1_TX_IRQHandler(void);
#endif
#if (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_RX_DEFAULT_IRQHANDLER==TRUE)
void UART1_RX_IRQHandler(void);
#endif

extern void uart_init(QN_UART_TypeDef *UART, uint32_t uartclk, enum UART_BAUDRATE baudrate);
extern void uart_read(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*rx_callback)(void));
extern void uart_write(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*tx_callback)(void));
extern void uart_printf(QN_UART_TypeDef *UART, uint8_t *bufptr);
extern void uart_finish_transfers(QN_UART_TypeDef *UART);
extern int uart_check_tx_free(QN_UART_TypeDef *UART);
extern void uart_flow_on(QN_UART_TypeDef *UART);
extern bool uart_flow_off(QN_UART_TypeDef *UART);
#if defined (CFG_DBG_PRINT) && defined (CFG_STD_PRINTF)
#include <stdio.h>
extern unsigned char UartPutc(unsigned char my_ch);
extern unsigned char UartGetc(void);
extern void uart_dump_register(uint32_t start, uint32_t end);
#endif


/// @} UART
#endif /* (CONFIG_ENABLE_DRIVER_UART0==TRUE || CONFIG_ENABLE_DRIVER_UART1==TRUE) */
#endif /* end _UART_H_ */
