/**
 ****************************************************************************************
 *
 * @file uart.c
 *
 * @brief UART Driver for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @addtogroup  UART
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "uart.h"
#if ((CONFIG_ENABLE_DRIVER_UART0==TRUE || CONFIG_ENABLE_DRIVER_UART1==TRUE))
#include "gpio.h"
#if UART_DMA_EN==TRUE
#include "dma.h"
#endif

/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */
///UART baudrate config parameters
struct uart_divider_cfg
{
    uint8_t integer_h;
    uint8_t integer_l;
    uint8_t fractional;
};

///UART TX/RX parameters
struct uart_txrxchannel
{
    uint8_t  *bufptr;
    int32_t  size;
    #if UART_CALLBACK_EN==TRUE
    void     (*callback)(void);
    #endif
};

///UART environment parameters
struct uart_env_tag
{
    struct uart_txrxchannel tx;
    struct uart_txrxchannel rx;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if CONFIG_ENABLE_DRIVER_UART0==TRUE 
///UART0 environment variable
static struct uart_env_tag uart0_env;
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE 
///UART1 environment variable
static struct uart_env_tag uart1_env;
#endif

#if UART_BAUDRATE_TABLE_EN==TRUE
/**
 * @description
 *  HOW TO CONFIGURATE BAUD RATE?
 *
 *   If oversample is 16, the required baud rate is 230400 and UARTCLK = 8MHz, then:
 *   Baud Rate Divisor = (8*1000000)/(16*230400) = 2.170,
 *   This means BRDI = 2 and BRDF = 0.170,
 *   Therefore, fractional part, m = integer((0.170*64)+0.5) = 11.
 *
 *   If the required baud rate is 921600 and UARTCLK = 16MHz then:
 *   Baud Rate Divisor = (16*1000000)/(16*921600) = 1.085,
 *   This means BRDI = 1 and BRDF = 0.085,
 *   Therefore, fractional part, m = integer((0.085*64)+0.5) = 5.
 *
 */
const struct uart_divider_cfg uart_divider[UART_BAUD_MAX] =
{
    {0x01, 0xA0, 0x2B, },
    {0x00, 0xD0, 0x15, },
    {0x00, 0x68, 0x0B, },
    {0x00, 0x34, 0x05, },
    {0x00, 0x22, 0x2E, },
    {0x00, 0x1A, 0x03, },
    {0x00, 0x11, 0x17, },
    {0x00, 0x0D, 0x01, },
    {0x00, 0x08, 0x2C, },
    {0x00, 0x07, 0x34, },
    {0x00, 0x06, 0x21, },
    {0x00, 0x04, 0x16, },
    {0x00, 0x03, 0x3A, },
    {0x00, 0x02, 0x0B, },
    {0x00, 0x01, 0x1D, },
    {0x00, 0x01, 0x05, },
    {0x00, 0x01, 0x00, },
};
#endif


/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */


/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if ((CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_TX_ENABLE_INTERRUPT==FALSE) \
  || (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_TX_ENABLE_INTERRUPT==FALSE))
/**
 ****************************************************************************************
 * @brief Transmit data to UART TX FIFO.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       uart_env      Environment Variable of specified UART port
 * @description
 * Start to transmit data to specified UART port until expected tramsmitting data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_TX_DMA_EN==FALSE
static void uart_transmit_data(QN_UART_TypeDef * UART, struct uart_env_tag *uart_env)
{
    while ( uart_env->tx.size > 0 )
    {
        uart_write_one_byte(UART, *uart_env->tx.bufptr++);
        uart_env->tx.size--;
    }

    #if UART_CALLBACK_EN==TRUE
    // Call end of transmission callback
    if(uart_env->tx.callback != NULL)
    {
        uart_env->tx.callback();
    }
    #endif
}
#endif
#endif

#if ((CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_RX_ENABLE_INTERRUPT==FALSE) \
  || (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_RX_ENABLE_INTERRUPT==FALSE))
/**
 ****************************************************************************************
 * @brief Receive data from UART RX FIFO.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       uart_env      Environment Variable of specified UART port
 * @description
 * Start to receive data from specified UART port until expected receiving data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_RX_DMA_EN==FALSE
static void uart_receive_data(QN_UART_TypeDef * UART, struct uart_env_tag *uart_env)
{
    while ( uart_env->rx.size > 0 )
    {
        *uart_env->rx.bufptr++ = uart_read_one_byte(UART);
         uart_env->rx.size--;
    }

    #if UART_CALLBACK_EN==TRUE
    // Call end of reception callback
    if(uart_env->rx.callback != NULL)
    {
        uart_env->rx.callback();
    }
    #endif
}
#endif
#endif

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_TX_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief UART0 TX interrupt handler.
 * @description
 *  Transmit data to port UART0 until expected tramsmitting data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_TX_DMA_EN==FALSE
void UART0_TX_IRQHandler(void)
{
    uint32_t reg;

    reg = uart_uart_GetIntFlag(QN_UART0);
    if ( reg & UART_MASK_TX_IF )    // TX FIFO is empty interrupt
    {
        // Wait until the Busy bit is cleared
        //while ( uart_uart_GetIntFlag(QN_UART0) & UART_MASK_TX_BUSY );

        if (uart0_env.tx.size > 0) {

            // clear interrupt
            uart_uart_SetTXD(QN_UART0, *uart0_env.tx.bufptr++);
            uart0_env.tx.size--;
        }
        else if (uart0_env.tx.size == 0) {
            // Disable UART all tx int
            uart_tx_int_enable(QN_UART0, MASK_DISABLE);

            #if UART_CALLBACK_EN==TRUE
            // Call end of transmission callback
            if(uart0_env.tx.callback != NULL)
            {
                uart0_env.tx.callback();
            }
            #endif
        }
    }
}
#endif
#endif /* CONFIG_UART0_TX_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_UART0==TRUE && CONFIG_UART0_RX_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief UART0 RX interrupt handler.
 * @description
 *  Receive data from port UART0 until expected receiving data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_RX_DMA_EN==FALSE
void UART0_RX_IRQHandler(void)
{
    uint32_t reg;

    reg = uart_uart_GetIntFlag(QN_UART0);
    if ( reg & UART_MASK_BE_IF) {  // Break error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART0, UART_MASK_BE_IF);
    }
    else if ( reg & UART_MASK_PE_IF ) {  // Parity error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART0, UART_MASK_PE_IF);
    }
    else if ( reg & UART_MASK_FE_IF ) {  // Framing error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART0, UART_MASK_FE_IF);
    }
    else if ( reg & UART_MASK_OE_IF ) {  // Overrun error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART0, UART_MASK_OE_IF);
    }
    else if ( reg & UART_MASK_RX_IF ) {  // RX FIFO is not empty interrupt
        // clear interrupt
        reg = uart_uart_GetRXD(QN_UART0);
        if (uart0_env.rx.size > 0) {
            *uart0_env.rx.bufptr++ = reg;
            uart0_env.rx.size--;

            if (uart0_env.rx.size == 0) {
                // Disable UART all rx int
                uart_rx_int_enable(QN_UART0, MASK_DISABLE);

                #if UART_CALLBACK_EN==TRUE
                // Call end of reception callback
                if(uart0_env.rx.callback != NULL)
                {
                    uart0_env.rx.callback();
                }
                #endif
            }
        }
    }
}
#endif
#endif /* CONFIG_UART0_RX_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_TX_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief UART1 TX interrupt handler.
 * @description
 *  Transmit data to port UART1 until expected tramsmitting data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_TX_DMA_EN==FALSE
void UART1_TX_IRQHandler(void)
{
    uint32_t reg;

    reg = uart_uart_GetIntFlag(QN_UART1);
    if ( reg & UART_MASK_TX_IF )    // TX FIFO is empty interrupt
    {
        // Wait until the Busy bit is cleared
        //while ( uart_uart_GetIntFlag(QN_UART1) & UART_MASK_TX_BUSY );

        if (uart1_env.tx.size > 0) {

            // clear interrupt
            uart_uart_SetTXD(QN_UART1, *uart1_env.tx.bufptr++);
            uart1_env.tx.size--;
        }
        else if (uart1_env.tx.size == 0) {
            // Disable UART all tx int
            uart_tx_int_enable(QN_UART1, MASK_DISABLE);

            #if UART_CALLBACK_EN==TRUE
            // Call end of transmission callback
            if(uart1_env.tx.callback != NULL)
            {
                uart1_env.tx.callback();
            }
            #endif
        }
    }
}
#endif
#endif /* CONFIG_UART1_TX_DEFAULT_IRQHANDLER==TRUE */

#if (CONFIG_ENABLE_DRIVER_UART1==TRUE && CONFIG_UART1_RX_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief UART1 RX interrupt handler.
 * @description
 *  Receive data from port UART1 until expected receiving data size is decreased to zero.
 ****************************************************************************************
 */
#if UART_RX_DMA_EN==FALSE
void UART1_RX_IRQHandler(void)
{
    uint32_t reg;

    reg = uart_uart_GetIntFlag(QN_UART1);
    if ( reg & UART_MASK_BE_IF ) {  // Break error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART1, UART_MASK_BE_IF);
    }
    else if ( reg & UART_MASK_PE_IF ) {  // Parity error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART1, UART_MASK_PE_IF);
    }
    else if ( reg & UART_MASK_FE_IF ) {  // Framing error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART1, UART_MASK_FE_IF);
    }
    else if ( reg & UART_MASK_OE_IF ) {  // Overrun error interrupt
        // clear interrupt
        uart_uart_ClrIntFlag(QN_UART1, UART_MASK_OE_IF);
    }
    else if ( reg & UART_MASK_RX_IF ) {  // RX FIFO is not empty interrupt
        // clear interrupt
        reg = uart_uart_GetRXD(QN_UART1);
        if (uart1_env.rx.size > 0) {
            *uart1_env.rx.bufptr++ = reg;
            uart1_env.rx.size--;

           if (uart1_env.rx.size == 0) {
                // Disable UART all rx int
                uart_rx_int_enable(QN_UART1, MASK_DISABLE);

                #if UART_CALLBACK_EN==TRUE
                // Call end of reception callback
                if(uart1_env.rx.callback != NULL)
                {
                    uart1_env.rx.callback();
                }
                #endif
            }
        }
    }
}
#endif
#endif /* CONFIG_UART1_RX_DEFAULT_IRQHANDLER==TRUE */

#if UART_BAUDRATE_TABLE_EN==FALSE
/**
 ****************************************************************************************
 * @brief Configurate UART baud rate.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       uartclk       USARTx_CLK(div)
 * @param[in]       oversample    UART_OVS16 or UART_OVS8
 * @param[in]       buadrate      115200
 * @description
 *  an example of how to configurate baud rate:
 *  If the required baud rate is 230400, UARTCLK = 8MHz, and overs = 16, then:
 *  Baud Rate Divisor = (8¡Á1000000)/(16¡Á230400) = 2.170
 *  This means BRDI = 2 and BRDF = 0.170.
 *  Therefore, fractional part, m = integer((0.170¡Á64)+0.5) = 11
 *****************************************************************************************
 */
static void uart_baudrate_set(QN_UART_TypeDef *UART, uint32_t uartclk, enum UART_OVERSAMPLE_TYPE oversample, uint32_t baudrate)
{
    uint32_t inter_div;
    uint32_t frac_div;
    uint32_t overs;
    uint32_t tmp;

    overs = (oversample == UART_OVS16) ? 16 : 8;
    tmp = (overs * baudrate);

    inter_div = uartclk / tmp;
    frac_div = uartclk - inter_div * tmp;

    frac_div  = (frac_div*64 + tmp / 2 ) / tmp;

    // Set UART baudrate
    tmp = (inter_div << UART_POS_DIVIDER_INT) + frac_div;
    uart_uart_SetBaudDivider(UART, tmp);
}
#endif

/**
 ****************************************************************************************
 * @brief Initialize the UART to default values.
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       uartclk       USARTx_CLK(div)
 * @param[in]       baudrate      baud rate
 * @description
 *  This function is used to initialize UART, it consists of baud-rate, parity, data-bits, stop-bits,
 *  over sample rate and bit order. The function is also used to enable specified UART interrupt, and
 *  enable NVIC UART IRQ.
 *****************************************************************************************
 */
void uart_init(QN_UART_TypeDef *UART, uint32_t uartclk, enum UART_BAUDRATE baudrate)
{
    // UART0 and UART1 are arranged in cross over configuration
    uint32_t reg;
    struct uart_env_tag *uart_env = &uart0_env;
    
    uart_clock_on(UART);

    // Set UART baudrate
#if UART_BAUDRATE_TABLE_EN==TRUE
    reg = (uart_divider[baudrate].integer_h << (UART_POS_DIVIDER_INT + 8))
        | (uart_divider[baudrate].integer_l << UART_POS_DIVIDER_INT)
        | uart_divider[baudrate].fractional;
    uart_uart_SetBaudDivider(UART, reg);
#else
    uart_baudrate_set(UART, uartclk, UART_OVS16, baudrate);
#endif

#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if (UART == QN_UART0) {
        /*
        * Set UART config:
        *
        * - oversample rate is 16
        * - HW flow control disable
        * - 1 stop bit
        * - parity type unused
        * - no parity
        * - bitorder = LSB
        */
        reg = UART_MASK_UART_EN         // uart enable
            | UART_MASK_UART_IE         // uart int enable
            //| UART_MASK_CTS_EN
            //| UART_MASK_RTS_EN
            | UART_OVS16
            | UART_MASK_LEVEL_INV
            | UART_MASK_BIT_ORDER;
        
        uart_env = &uart0_env;

        #if CONFIG_UART0_TX_ENABLE_INTERRUPT==TRUE && UART_TX_DMA_EN==FALSE
        // Enable the UART0 TX Interrupt
        NVIC_EnableIRQ(UART0_TX_IRQn);
        #endif
        
        #if CONFIG_UART0_RX_ENABLE_INTERRUPT==TRUE && UART_RX_DMA_EN==FALSE
        // Enable the UART0 RX Interrupt
        NVIC_EnableIRQ(UART0_RX_IRQn);
        #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if (UART == QN_UART1) {
        /*
        * Set UART config:
        *
        * - oversample rate is 16
        * - HW flow control disable
        * - 1 stop bit
        * - parity type unused
        * - no parity
        * - bitorder = LSB
        */
        reg = UART_MASK_UART_EN         // uart enable
            | UART_MASK_UART_IE         // uart int enable
            | UART_OVS16
            | UART_MASK_LEVEL_INV
            | UART_MASK_BIT_ORDER;
        
        uart_env = &uart1_env;
        
        #if CONFIG_UART1_TX_ENABLE_INTERRUPT==TRUE && UART_TX_DMA_EN==FALSE
        // Enable the UART1 TX Interrupt
        NVIC_EnableIRQ(UART1_TX_IRQn);
        #endif
        
        #if CONFIG_UART1_RX_ENABLE_INTERRUPT==TRUE && UART_RX_DMA_EN==FALSE
        // Enable the UART1 RX Interrupt
        NVIC_EnableIRQ(UART1_RX_IRQn);
        #endif
    }
#endif

    // Set UART config
    uart_uart_SetCR(UART, reg);

#if UART_DMA_EN==TRUE
    dma_init();
#endif
    //Configure UART environment
    uart_env->rx.size = 0;
    uart_env->tx.size = 0;
    uart_env->rx.bufptr = NULL;
    uart_env->tx.bufptr = NULL;
    #if UART_CALLBACK_EN==TRUE
    uart_env->rx.callback = NULL;
    uart_env->tx.callback = NULL;
    #endif

}

/**
 ****************************************************************************************
 * @brief Start a data reception.
 * @param[in]      UART           QN_UART0 or QN_UART1
 * @param[in,out]  bufptr         Pointer to the RX buffer
 * @param[in]      size           Size of the expected reception
 * @param[in]      rx_callback    Callback for end of reception
 * @description
 * This function is used to read Rx data from RX FIFO and the data will be stored in bufptr.
 * As soon as the end of the data transfer is detected, the callback function is executed.
 *
 *****************************************************************************************
 */
void uart_read(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*rx_callback)(void))
{
#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if (UART == QN_UART0) {
    #if UART_RX_DMA_EN==TRUE
        dma_rx(DMA_TRANS_BYTE, DMA_UART0_RX, (uint32_t)bufptr, size, rx_callback);
    #else
        //Store environment parameters
        uart0_env.rx.size = size;
        uart0_env.rx.bufptr = bufptr;
        #if UART_CALLBACK_EN==TRUE
        uart0_env.rx.callback = rx_callback;
        #endif
    
        #if CONFIG_UART0_RX_ENABLE_INTERRUPT==TRUE
        // Enable UART and all RX int
        uart_rx_int_enable(UART, MASK_ENABLE);
        #else
        // Start data transmission
        uart_receive_data(UART, &uart0_env);
        #endif
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if (UART == QN_UART1) {
    #if UART_RX_DMA_EN==TRUE
        dma_rx(DMA_TRANS_BYTE, DMA_UART1_RX, (uint32_t)bufptr, size, rx_callback);
    #else
        //Store environment parameters
        uart1_env.rx.size = size;
        uart1_env.rx.bufptr = bufptr;
        #if UART_CALLBACK_EN==TRUE
        uart1_env.rx.callback = rx_callback;
        #endif
        
        #if CONFIG_UART1_RX_ENABLE_INTERRUPT==TRUE
        // Enable UART and all RX int
        uart_rx_int_enable(UART, MASK_ENABLE);
        #else
        // Start data transmission
        uart_receive_data(UART, &uart1_env);
        #endif
    #endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Start a data transmission.
 * @param[in]  UART            QN_UART0 or QN_UART1
 * @param[in]  bufptr          Pointer to the TX data buffer
 * @param[in]  size            Size of the transmission
 * @param[in]  tx_callback     Callback for end of transmission
 * @description
 * This function is used to write data into TX buffer to transmit data by UART.
 * As soon as the end of the data transfer is detected, the callback function is executed.
 *
 *****************************************************************************************
 */
void uart_write(QN_UART_TypeDef *UART, uint8_t *bufptr, uint32_t size, void (*tx_callback)(void))
{
#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if (UART == QN_UART0) {
    #if UART_TX_DMA_EN==TRUE
        dma_tx(DMA_TRANS_BYTE, (uint32_t)bufptr, DMA_UART0_TX, size, tx_callback);
    #else
        //Store environment parameters
        uart0_env.tx.size = size;
        uart0_env.tx.bufptr = bufptr;
        #if UART_CALLBACK_EN==TRUE
        uart0_env.tx.callback = tx_callback;
        #endif
        
        #if CONFIG_UART0_TX_ENABLE_INTERRUPT==TRUE
        // Enable UART tx int
        uart_tx_int_enable(UART, MASK_ENABLE);
        #else
        // Start data transmission
        uart_transmit_data(UART, &uart0_env);
        #endif
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if (UART == QN_UART1) {
    #if UART_TX_DMA_EN==TRUE
        dma_tx(DMA_TRANS_BYTE, (uint32_t)bufptr, DMA_UART1_TX, size, tx_callback);
    #else        
        //Store environment parameters
        uart1_env.tx.size = size;
        uart1_env.tx.bufptr = bufptr;
        #if UART_CALLBACK_EN==TRUE
        uart1_env.tx.callback = tx_callback;
        #endif

        #if CONFIG_UART1_TX_ENABLE_INTERRUPT==TRUE
        // Enable UART tx int
        uart_tx_int_enable(UART, MASK_ENABLE);
        #else
        // Start data transmission
        uart_transmit_data(UART, &uart1_env);
        #endif
    #endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief  Send a string to UART
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @param[in]       bufptr        buffer pointer of tx data
 * @description
 * Print a string to specified UART port
 *****************************************************************************************
 */
#if 0
void uart_printf(QN_UART_TypeDef *UART, uint8_t *bufptr)
{
    uart_write(UART, bufptr, strlen((const char *)bufptr), NULL);
}
#else
void uart_printf(QN_UART_TypeDef *UART, uint8_t *bufptr)
{
    while ( *bufptr != '\0' ) {
        // Wait until the Busy bit is cleared
        //while ( uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY );

        // Move on only if NOT busy and TX FIFO not full
        while ( !(uart_uart_GetIntFlag(UART) & UART_MASK_TX_IF) );

        uart_uart_SetTXD(UART, *bufptr++);
    }
}
#endif

/**
 ****************************************************************************************
 * @brief  Wait until transfer finish
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @description
 *  Waiting for specified UART port transfer finished
 *****************************************************************************************
 */
void uart_finish_transfers(QN_UART_TypeDef *UART)
{
    // Wait until the Busy bit is cleared 
    while ( uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY );
}

/**
 ****************************************************************************************
 * @brief  Check if tx is ongoing
 * @return uart tx/rx status 
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @description
 *  This function is used to check UART TX status
 *****************************************************************************************
 */
int uart_check_tx_free(QN_UART_TypeDef *UART)
{

#if UART_TX_DMA_EN==TRUE
    // check DMA status
    if (dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_BUSY)
        return UART_TX_BUF_BUSY;
#else
    #if CONFIG_ENABLE_DRIVER_UART0==TRUE
    if(UART == QN_UART0)
    {
        // check tx buffer
        if (uart0_env.tx.size > 0)
            return UART_TX_BUF_BUSY;
    }
    #endif    

    #if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if(UART == QN_UART1)
    {
        // check tx buffer
        if(uart1_env.tx.size > 0)
            return UART_TX_BUF_BUSY;
    }
    #endif
#endif

    // check tx busy
    if(uart_uart_GetIntFlag(UART) & UART_MASK_TX_BUSY)
        return UART_LAST_BYTE_ONGOING;

    return UART_TX_FREE;
}

/**
 ****************************************************************************************
 * @brief  Enable hardware flow control
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @description
 *  Enable specified UART port hardware flow control 
 *****************************************************************************************
 */
void uart_flow_on(QN_UART_TypeDef *UART)
{
    //uart_uart_SetCRWithMask(UART, UART_MASK_CTS_EN|UART_MASK_RTS_EN, MASK_ENABLE);

#if CONFIG_ENABLE_DRIVER_UART0==TRUE
    // Switch RTS/CTS back
    if(UART == QN_UART0)
    {
        syscon_SetPMCR0WithMask(QN_SYSCON, 
                                P02_MASK_PIN_CTRL | P01_MASK_PIN_CTRL,
                                P02_UART0_RTS_PIN_CTRL | P01_UART0_CTS_PIN_CTRL);
    }
#endif

#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if(UART == QN_UART1)
    {
        // NOTE!!!!!
        // Assume UART1 used P2.2 and P3.6 as RTS/CTS. Other case this snippet should be modified.
        syscon_SetPMCR1WithMask(QN_SYSCON, 
                                P22_MASK_PIN_CTRL | P36_MASK_PIN_CTRL,
                                P22_UART1_RTS_PIN_CTRL | P36_UART1_CTS_PIN_CTRL);
    }
#endif
}

/**
 ****************************************************************************************
 * @brief  Disable hardware flow control
 * @param[in]       UART          QN_UART0 or QN_UART1
 * @return TRUE
 * @description
 *  Disable specified UART port hardware flow control 
 *****************************************************************************************
 */
bool uart_flow_off(QN_UART_TypeDef *UART)
{
    //uart_uart_SetCRWithMask(UART, UART_MASK_CTS_EN|UART_MASK_RTS_EN, MASK_DISABLE);
    //return true;

    bool rt = false;
    uint32_t int_restore = 0;

    // Disable UART interrupt
#if CONFIG_ENABLE_DRIVER_UART0==TRUE   
    if(UART == QN_UART0)
        int_restore = NVIC->ISER[0] & ((1<<UART0_TX_IRQn) | (1<<UART0_RX_IRQn));
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE
    if(UART == QN_UART1)
        int_restore = NVIC->ISER[0] & ((1<<UART1_TX_IRQn) | (1<<UART1_RX_IRQn));
#endif
    NVIC->ICER[0] = int_restore;

    do
    {
        // Check if no tx is ongoing
        if(UART_TX_FREE == uart_check_tx_free(UART))
            break;

        // Disable rx (RTS/CTS -> GPIO high)
#if CONFIG_ENABLE_DRIVER_UART0==TRUE
        if(UART == QN_UART0)
        {
            syscon_SetPMCR0WithMask(QN_SYSCON,
                                    P02_MASK_PIN_CTRL | P01_MASK_PIN_CTRL,
                                    P02_GPIO_2_PIN_CTRL | P01_GPIO_1_PIN_CTRL);

            gpio_write_pin(GPIO_P02, GPIO_HIGH);
            gpio_write_pin(GPIO_P01, GPIO_HIGH);
        }
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE
        if(UART == QN_UART1)
        {
            // NOTE!!!!!
            // Assume UART1 used P2.2 and P3.6 as RTS/CTS. Other case this snippet should be modified.
            syscon_SetPMCR1WithMask(QN_SYSCON,
                                    P22_MASK_PIN_CTRL | P36_MASK_PIN_CTRL,
                                    P22_GPIO_18_PIN_CTRL | P36_GPIO_30_PIN_CTRL);

            gpio_write_pin(GPIO_P22, GPIO_HIGH);
            gpio_write_pin(GPIO_P36, GPIO_HIGH);
        }
#endif
        // Wait for 1 bytes duration to guarantee host has not started a tx at this time.
        // Assume buadrate 115200
        delay(100);

        // Check if data has been received during the waiting time
        if(uart_uart_GetIntFlag(UART) & UART_MASK_RX_IF)
        {
            // Switch RTS/CTS back
#if CONFIG_ENABLE_DRIVER_UART0==TRUE
            if(UART == QN_UART0)
            {
                syscon_SetPMCR0WithMask(QN_SYSCON, 
                                        P02_MASK_PIN_CTRL | P01_MASK_PIN_CTRL,
                                        P02_UART0_RTS_PIN_CTRL | P01_UART0_CTS_PIN_CTRL);
            }
#endif
#if CONFIG_ENABLE_DRIVER_UART1==TRUE
            if(UART == QN_UART1)
            {
                // NOTE!!!!!
                // Assume UART1 used P2.2 and P3.6 as RTS/CTS. Other case this snippet should be modified.
                syscon_SetPMCR1WithMask(QN_SYSCON, 
                                        P22_MASK_PIN_CTRL | P36_MASK_PIN_CTRL,
                                        P22_UART1_RTS_PIN_CTRL | P36_UART1_CTS_PIN_CTRL);
            }
#endif
            // failed.
            break;
        }
        
        // success.
        rt = true;
    }
    while(false);

    // Restore uart interrupt status
    NVIC->ISER[0] = int_restore;

    return rt;
}

#if defined (CFG_DBG_PRINT) && defined (CFG_STD_PRINTF)
/**
 ****************************************************************************************
 * @brief Output a character.
 * @param[in]       my_ch         output char
 * @return output char 
 * @description
 * Output a character to uart debug port.
 ****************************************************************************************
 */
unsigned char UartPutc(unsigned char my_ch)
{
    /* Move on only if NOT busy and TX FIFO not full. */
    while ( !(uart_uart_GetIntFlag(QN_DEBUG_UART) & UART_MASK_TX_IF) );

    uart_uart_SetTXD(QN_DEBUG_UART, my_ch);

    while ( uart_uart_GetIntFlag(QN_DEBUG_UART) & UART_MASK_TX_BUSY );
    return (my_ch);
}

/**
 ****************************************************************************************
 * @brief Get a character.
 * @return received char 
 * @description
 * Get a character form uart debug port.
 ****************************************************************************************
 */
unsigned char UartGetc(void)
{
    /* As long as Receive FIFO is not empty, I can always receive. */
    while ( !(uart_uart_GetIntFlag(QN_DEBUG_UART) & UART_MASK_RX_IF) );

    return (uart_uart_GetRXD(QN_DEBUG_UART));
}

/**
 ****************************************************************************************
 * @brief Dump register
 * @description
 * Output register to uart
 ****************************************************************************************
 */
void uart_dump_register(uint32_t start, uint32_t end)
{
    for (int i = start; i <= end; i+=4) {
        printf("0x%08x = 0x%08x \r\n", i, inp32(i));
    }
}
#endif

#endif /* CONFIG_ENABLE_DRIVER_UART0==TRUE || CONFIG_ENABLE_DRIVER_UART1==TRUE */
/// @} UART
