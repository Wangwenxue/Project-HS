/**
 ****************************************************************************************
 *
 * @file spi.c
 *
 * @brief SPI driver for QN9020.
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
 * @addtogroup  SPI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "spi.h"
#if ((CONFIG_ENABLE_DRIVER_SPI0==TRUE || CONFIG_ENABLE_DRIVER_SPI1==TRUE))
#if SPI_DMA_EN==TRUE
#include "dma.h"
#endif



/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
///SPI0 environment variable
volatile struct spi_env_tag spi0_env;
#endif
#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
///SPI1 environment variable
volatile struct spi_env_tag spi1_env;
#endif

/*
 * LOCAL FUNCTION DECLARATION
 ****************************************************************************************
 */
#if ((CONFIG_ENABLE_DRIVER_SPI0==TRUE && CONFIG_SPI0_TX_ENABLE_INTERRUPT==FALSE) \
  || (CONFIG_ENABLE_DRIVER_SPI1==TRUE && CONFIG_SPI1_TX_ENABLE_INTERRUPT==FALSE))
/**
 ****************************************************************************************
 * @brief Transmit data to SPI TX FIFO.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @param[in]       spi_env       Environment variable of specified SPI port
 * @description
 * Start to transmit date to specified SPI port until expected tramsmitting data size is reduced to zero.
 ****************************************************************************************
 */
#if SPI_TX_DMA_EN==FALSE
static void spi_transmit_data(QN_SPI_TypeDef * SPI, struct spi_env_tag *spi_env)
{
    while ( spi_env->tx.size > 0 )
    {
        while ( !(spi_spi_GetSR(SPI) & SPI_MASK_TX_FIFO_NFUL_IF) );

        spi_tx_data(SPI, spi_env);
    }
    
#if SPI_CALLBACK_EN==TRUE
    // Call end of transmission callback
    if(spi_env->tx.callback != NULL)
    {
        spi_env->tx.callback();
    }
#endif
}
#endif
#endif

#if ((CONFIG_ENABLE_DRIVER_SPI0==TRUE && CONFIG_SPI0_RX_ENABLE_INTERRUPT==FALSE) \
  || (CONFIG_ENABLE_DRIVER_SPI1==TRUE && CONFIG_SPI1_RX_ENABLE_INTERRUPT==FALSE))
/**
 ****************************************************************************************
 * @brief Receives data from SPI RX FIFO.
 * @param[in]       SPI           QN_SPI0 or QN_SPI1
 * @param[in]       spi_env       Environment variable of specified SPI port
 * @description
 * Start to receive date from specified SPI port until expected receiving data size is reduced to zero.
 ****************************************************************************************
 */
#if SPI_RX_DMA_EN==FALSE
static void spi_receive_data(QN_SPI_TypeDef * SPI, struct spi_env_tag *spi_env)
{
    while ( spi_env->rx.size > 0 )
    {
        /* As long as Receive FIFO is not empty, we can always receive. */
        /* if it's a peer-to-peer communication, TXD needs to be written before a read can take place. */
        if ( (spi_env->mode == SPI_MASTER_MOD) && (spi_check_tx_free(SPI) == SPI_TX_FREE) ) {
            spi_spi_SetTXD(SPI, SPI_DUMMY_DATA);
            /* Wait until the Busy bit is cleared */
            //while ( spi_spi_GetSR(SPI) & SPI_MASK_BUSY );
        }
        while ( !(spi_spi_GetSR(SPI) & SPI_MASK_RX_FIFO_NEMT_IF) );

        spi_rx_data(SPI, spi_env);
    }

#if SPI_CALLBACK_EN==TRUE
    // Call end of reception callback
    if(spi_env->rx.callback != NULL)
    {
        spi_env->rx.callback();
    }
#endif
}
#endif
#endif

/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if (CONFIG_ENABLE_DRIVER_SPI0==TRUE && CONFIG_SPI0_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief SPI0 RX interrupt handler.
 * @description
 * If SPI0 RX FIFO is not empty, it then generates interrupt. In this handler, data is received from
 * port SPI0 until expected receiving data size is reduced to zero. After last data received, the callback function is called.
 ****************************************************************************************
 */
void SPI0_IRQHandler(void)
{
#if (CONFIG_SPI0_RX_ENABLE_INTERRUPT==TRUE)
    while ( spi_spi_GetSR(QN_SPI0) & SPI_MASK_RX_FIFO_NEMT_IF ) { // RX FIFO not empty interrupt

        if (spi0_env.rx.size > 0) {

            spi_rx_data(QN_SPI0, (struct spi_env_tag *)&spi0_env);

            if (spi0_env.rx.size <= 0) {
                // Disable RX interrupt
                spi_int_enable(QN_SPI0, SPI_RX_INT, MASK_DISABLE);
                #if SPI_CALLBACK_EN==TRUE
                // Call end of reception callback
                if (spi0_env.rx.callback != NULL)
                {
                    spi0_env.rx.callback();
                }
                #endif
            }
        }
        else {
            spi_spi_GetRXD(QN_SPI0);  // clear interrupt
        }
    }
#endif

    if ( spi_spi_GetSR(QN_SPI0) & SPI_MASK_TX_FIFO_NFUL_IF )   /* TX FIFO not full interrupt */
    {
#if (CONFIG_SPI0_TX_ENABLE_INTERRUPT==TRUE)
        if (spi0_env.tx.size > 0) {

            spi_tx_data(QN_SPI0, (struct spi_env_tag *)&spi0_env);
            NVIC_ClearPendingIRQ(SPI0_TX_IRQn);

            if (spi0_env.tx.size <= 0) {
                // Disable TX interrupt
                spi_int_enable(QN_SPI0, SPI_TX_INT, MASK_DISABLE);
                #if SPI_CALLBACK_EN==TRUE
                // Call end of transmission callback
                if (spi0_env.tx.callback != NULL)
                {
                    spi0_env.tx.callback();
                }
                #endif
            }
        }
        else
#endif
        {
            if ( (spi0_env.mode == SPI_MASTER_MOD)
                && (spi0_env.rx.size > 0) ) {
#if (CONFIG_SPI0_TX_ENABLE_INTERRUPT==FALSE)
                if (spi_check_tx_free(QN_SPI0) == SPI_TX_FREE)
#endif
                spi_spi_SetTXD(QN_SPI0, SPI_DUMMY_DATA);
            }
        }
    }
}
#endif


#if (CONFIG_ENABLE_DRIVER_SPI1==TRUE && CONFIG_SPI1_DEFAULT_IRQHANDLER==TRUE)
/**
 ****************************************************************************************
 * @brief SPI1 RX interrupt handler.
 * @description
 * If SPI1 RX FIFO is not empty, it then generates interrupt. In this handler, data is received from port SPI1
 * until expected receiving data size is reduced to zero. After last data received, the callback function is called.
 ****************************************************************************************
 */
void SPI1_IRQHandler(void)
{
#if (CONFIG_SPI1_RX_ENABLE_INTERRUPT==TRUE)
    while ( spi_spi_GetSR(QN_SPI1) & SPI_MASK_RX_FIFO_NEMT_IF ) { // RX FIFO not empty interrupt

        if (spi1_env.rx.size > 0) {

            spi_rx_data(QN_SPI1, (struct spi_env_tag *)&spi1_env);

            if (spi1_env.rx.size <= 0) {
                // Disable RX interrupt
                spi_int_enable(QN_SPI1, SPI_RX_INT, MASK_DISABLE);
                #if SPI_CALLBACK_EN==TRUE
                // Call end of reception callback
                if (spi1_env.rx.callback != NULL)
                {
                    spi1_env.rx.callback();
                }
                #endif
            }
        }
        else {
            spi_spi_GetRXD(QN_SPI1);  // clear interrupt
        }
    }
#endif

    if ( spi_spi_GetSR(QN_SPI1) & SPI_MASK_TX_FIFO_NFUL_IF )   /* TX FIFO not full interrupt */
    {
#if (CONFIG_SPI1_TX_ENABLE_INTERRUPT==TRUE)
        if (spi1_env.tx.size > 0) {

            spi_tx_data(QN_SPI1, (struct spi_env_tag *)&spi1_env);
            NVIC_ClearPendingIRQ(SPI1_TX_IRQn);

            if (spi1_env.tx.size <= 0) {
                // Disable TX interrupt
                spi_int_enable(QN_SPI1, SPI_TX_INT, MASK_DISABLE);
                #if SPI_CALLBACK_EN==TRUE
                // Call end of transmission callback
                if (spi1_env.tx.callback != NULL)
                {
                    spi1_env.tx.callback();
                }
                #endif
            }
        }
        else 
#endif
        {
            if ( (spi1_env.mode == SPI_MASTER_MOD)
                && (spi1_env.rx.size > 0)
#if (CONFIG_SPI1_TX_ENABLE_INTERRUPT==FALSE) 
                && (spi_check_tx_free(QN_SPI1) == SPI_TX_FREE)
#endif
            ) {
                spi_spi_SetTXD(QN_SPI1, SPI_DUMMY_DATA);
            }
        }
    }
}
#endif

/**
 ****************************************************************************************
 * @brief Initialize the SPI
 * @param[in]  SPI            QN_SPI0 or QN_SPI1
 * @param[in]  bitrate        sck speed: SPI_BITRATE(1000) means 1Kbps
 * @param[in]  width          32bits or 8bits
 * @param[in]  mode           master or slave
 * @description
 *  This function is used to initialize SPI. It consists of bit rate, transmit width, SPI mode, big/little endian,
 *  MSB/LSB first, master/salve. The function is also used to enable specified SPI interrupt, and
 *  enable NVIC SPI IRQ.
 *****************************************************************************************
 */
void spi_init(QN_SPI_TypeDef * SPI, uint32_t bitrate, enum SPI_BUFFER_WIDTH width, enum SPI_MODE mode)
{
    uint32_t reg = 0;
    struct spi_env_tag *spi_env;

    spi_clock_on(SPI);

#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if (SPI == QN_SPI0) {
        spi_env = (struct spi_env_tag *)&spi0_env;

        reg = SPI_SSx_CFG
            | SPI_LITTLE_ENDIAN
            | width
            | SPI_BITORDER_CFG
            | mode
#if SPI0_MOD_3WIRE_EN==TRUE
            | SPI_MASK_DATA_IO_MODE
#endif
            | SPI_CPHA_0
            | SPI_CPOL_0;

        #if CONFIG_SPI0_TX_ENABLE_INTERRUPT==TRUE && SPI_TX_DMA_EN==FALSE
        // Enable the SPI0 TX Interrupt
        NVIC_EnableIRQ(SPI0_TX_IRQn);
        #endif

        #if CONFIG_SPI0_RX_ENABLE_INTERRUPT==TRUE && SPI_RX_DMA_EN==FALSE
        // Enable the SPI0 RX Interrupt
        NVIC_EnableIRQ(SPI0_RX_IRQn);
        #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if (SPI == QN_SPI1) {
        spi_env = (struct spi_env_tag *)&spi1_env;

        reg = SPI_SSx_CFG
            | SPI_LITTLE_ENDIAN
            | width
            | SPI_BITORDER_CFG
            | mode
#if SPI1_MOD_3WIRE_EN==TRUE
            | SPI_MASK_DATA_IO_MODE
#endif
            | SPI_CPHA_0
            | SPI_CPOL_0;

        #if CONFIG_SPI1_TX_ENABLE_INTERRUPT==TRUE && SPI_TX_DMA_EN==FALSE
        // Enable the SPI1 TX Interrupt
        NVIC_EnableIRQ(SPI1_TX_IRQn);
        #endif

        #if CONFIG_SPI1_RX_ENABLE_INTERRUPT==TRUE && SPI_RX_DMA_EN==FALSE
        // Enable the SPI1 RX Interrupt
        NVIC_EnableIRQ(SPI1_RX_IRQn);
        #endif
    }
#endif

    reg |= bitrate;
    spi_spi_SetCR0(SPI, reg);

#if SPI_DMA_EN==TRUE
    dma_init();
#endif
    //Configure SPI environment
    spi_env->width = width;     // 8-bits or 32-bits
    spi_env->mode = mode;       // master or slave
    spi_env->rx.size = 0;
    spi_env->tx.size = 0;
    spi_env->rx.bufptr = NULL;
    spi_env->tx.bufptr = NULL;
#if SPI_CALLBACK_EN==TRUE
    spi_env->rx.callback = NULL;
    spi_env->tx.callback = NULL;
#endif
}

/**
 ****************************************************************************************
 * @brief Start a data reception.
 * @param[in]      SPI            QN_SPI0 or QN_SPI1
 * @param[in,out]  bufptr         Pointer to the RX data buffer
 * @param[in]      size           Size of the expected reception, must be multiple of 4 at 32bit mode
 * @param[in]      rx_callback    Callback for end of reception
 * @description
 * This function is used to read Rx data from RX FIFO and the data will be stored in bufptr.
 * As soon as the end of the data transfer or a buffer overflow is detected, the callback function is called.
 *
 *****************************************************************************************
 */
void spi_read(QN_SPI_TypeDef * SPI, uint8_t *bufptr, int32_t size, void (*rx_callback)(void))
{
#if SPI_RX_DMA_EN==TRUE
    enum DMA_TRANS_MODE trans_mod;
#endif
    // option: clear RX buffer
    //spi_spi_SetCR1(SPI, SPI_MASK_RX_FIFO_CLR);

#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if (SPI == QN_SPI0) {

    #if SPI0_MOD_3WIRE_EN==TRUE
        if (spi0_env.mode == SPI_MASTER_MOD) {
            spi_spi_SetCR1WithMask(QN_SPI0, SPI_MASK_M_SDIO_EN, MASK_DISABLE);
        }
        else {
            spi_spi_SetCR1WithMask(QN_SPI0, SPI_MASK_S_SDIO_EN, MASK_DISABLE);
        }
    #endif

    #if SPI_RX_DMA_EN==TRUE
        if (spi0_env.width == SPI_8BIT) {
            trans_mod = DMA_TRANS_BYTE;
        }
        else {
            trans_mod = DMA_TRANS_WORD;
        }
        dma_rx(trans_mod, DMA_SPI0_RX, (uint32_t)bufptr, size, rx_callback);
    #else
        //Store environment parameters
        spi0_env.rx.bufptr = bufptr;
        spi0_env.rx.size = size;
        #if SPI_CALLBACK_EN==TRUE
        spi0_env.rx.callback = rx_callback;
        #endif
        
        #if CONFIG_SPI0_RX_ENABLE_INTERRUPT==TRUE
        #else
        // Start data reception
        spi_receive_data(SPI, (struct spi_env_tag *)&spi0_env);
        #endif
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if (SPI == QN_SPI1) {

    #if SPI1_MOD_3WIRE_EN==TRUE
        if (spi1_env.mode == SPI_MASTER_MOD) {
            spi_spi_SetCR1WithMask(QN_SPI1, SPI_MASK_M_SDIO_EN, MASK_DISABLE);
        }
        else {
            spi_spi_SetCR1WithMask(QN_SPI1, SPI_MASK_S_SDIO_EN, MASK_DISABLE);
        }
    #endif

    #if SPI_RX_DMA_EN==TRUE
        if (spi1_env.width == SPI_8BIT) {
            trans_mod = DMA_TRANS_BYTE;
        }
        else {
            trans_mod = DMA_TRANS_WORD;
        }
        dma_rx(trans_mod, DMA_SPI1_RX, (uint32_t)bufptr, size, rx_callback);
    #else
        //Store environment parameters
        spi1_env.rx.bufptr = bufptr;
        spi1_env.rx.size = size;
        #if SPI_CALLBACK_EN==TRUE
        spi1_env.rx.callback = rx_callback;
        #endif
        
        #if CONFIG_SPI1_RX_ENABLE_INTERRUPT==TRUE
        #else
        // Start data reception
        spi_receive_data(SPI, (struct spi_env_tag *)&spi1_env);
        #endif
    #endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief Start a data transmission.
 * @param[in]  SPI            QN_SPI0 or QN_SPI1
 * @param[in]  bufptr         Pointer to the TX data buffer
 * @param[in]  size           Size of the transmission, must be multiple of 4 at 32bit mode
 * @param[in]  tx_callback    Callback for end of transmission
 * @description
 * This function is used to write data into TX buffer to transmit data by SPI.
 * As soon as the end of the data transfer is detected, the callback function is called.
 *
 *****************************************************************************************
 */
void spi_write(QN_SPI_TypeDef * SPI, uint8_t *bufptr, int32_t size, void (*tx_callback)(void))
{
#if SPI_TX_DMA_EN==TRUE
    enum DMA_TRANS_MODE trans_mod;
#endif
    // option: clear TX buffer
    //spi_spi_SetCR1(SPI, SPI_MASK_TX_FIFO_CLR);

#if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if (SPI == QN_SPI0) {

    #if SPI0_MOD_3WIRE_EN==TRUE
        if (spi0_env.mode == SPI_MASTER_MOD) {
            spi_spi_SetCR1WithMask(QN_SPI0, SPI_MASK_M_SDIO_EN, MASK_ENABLE);
        }
        else {
            spi_spi_SetCR1WithMask(QN_SPI0, SPI_MASK_S_SDIO_EN, MASK_ENABLE);
        }
    #endif

    #if SPI_TX_DMA_EN==TRUE
        if (spi0_env.width == SPI_8BIT) {
            trans_mod = DMA_TRANS_BYTE;
        }
        else {
            trans_mod = DMA_TRANS_WORD;
        }
        dma_tx(trans_mod, (uint32_t)bufptr, DMA_SPI0_TX, size, tx_callback);
    #else
        //Store environment parameters
        spi0_env.tx.bufptr = bufptr;
        spi0_env.tx.size = size;
        #if SPI_CALLBACK_EN==TRUE
        spi0_env.tx.callback = tx_callback;
        #endif
        
        #if CONFIG_SPI0_TX_ENABLE_INTERRUPT==TRUE
        #else
        // Start data transmission
        spi_transmit_data(SPI, (struct spi_env_tag *)&spi0_env);
        #endif
    #endif
    }
#endif

#if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if (SPI == QN_SPI1) {

    #if SPI1_MOD_3WIRE_EN==TRUE
        if (spi1_env.mode == SPI_MASTER_MOD) {
            spi_spi_SetCR1WithMask(QN_SPI1, SPI_MASK_M_SDIO_EN, MASK_ENABLE);
        }
        else {
            spi_spi_SetCR1WithMask(QN_SPI1, SPI_MASK_S_SDIO_EN, MASK_ENABLE);
        }
    #endif

    #if SPI_TX_DMA_EN==TRUE
        if (spi1_env.width == SPI_8BIT) {
            trans_mod = DMA_TRANS_BYTE;
        }
        else {
            trans_mod = DMA_TRANS_WORD;
        }
        dma_tx(trans_mod, (uint32_t)bufptr, DMA_SPI1_TX, size, tx_callback);
    #else
        //Store environment parameters
        spi1_env.tx.bufptr = bufptr;
        spi1_env.tx.size = size;
        #if SPI_CALLBACK_EN==TRUE
        spi1_env.tx.callback = tx_callback;
        #endif

        #if CONFIG_SPI1_TX_ENABLE_INTERRUPT==TRUE
        #else
        // Start data transmission
        spi_transmit_data(SPI, (struct spi_env_tag *)&spi1_env);
        #endif
    #endif
    }
#endif
}

/**
 ****************************************************************************************
 * @brief  Check if tx is ongoing
 * @return spi tx/rx status 
 * @param[in]       SPI          QN_SPI0 or QN_SPI1
 * @description
 *  This function is used to check SPI TX status
 *****************************************************************************************
 */
int spi_check_tx_free(QN_SPI_TypeDef *SPI)
{

#if SPI_TX_DMA_EN==TRUE
    // check DMA status
    if (dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_BUSY)
        return SPI_TX_BUF_BUSY;
#else
    #if CONFIG_ENABLE_DRIVER_SPI0==TRUE
    if(SPI == QN_SPI0)
    {
        // check tx buffer
        if (spi0_env.tx.size > 0)
            return SPI_TX_BUF_BUSY;
    }
    #endif    

    #if CONFIG_ENABLE_DRIVER_SPI1==TRUE
    if(SPI == QN_SPI1)
    {
        // check tx buffer
        if(spi1_env.tx.size > 0)
            return SPI_TX_BUF_BUSY;
    }
    #endif
#endif

    // check tx busy
    if ((spi_spi_GetSR(SPI) & (SPI_MASK_BUSY|SPI_MASK_TX_FIFO_EMPT)) == SPI_MASK_TX_FIFO_EMPT) {
        return SPI_TX_FREE;
    }
    else {
        return SPI_LAST_BYTE_ONGOING;
    }
}

#endif /* CONFIG_ENABLE_DRIVER_SPI==TRUE */
/// @} SPI
