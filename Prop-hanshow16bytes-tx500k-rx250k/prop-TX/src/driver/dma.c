/**
 ****************************************************************************************
 *
 * @file dma.c
 *
 * @brief DMA driver for QN9020.
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
 * @addtogroup  DMA
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "dma.h"
#if CONFIG_ENABLE_DRIVER_DMA==TRUE
#include "uart.h"

///DMA environment parameters
struct dma_env_tag
{
    void     (*callback)(void);
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */
#if DMA_CALLBACK_EN==TRUE
///Variable used to store DMA environment
static struct dma_env_tag dma_env;
#endif

/// DMA TX peripheral address
static const uint32_t peripheral_dst[DMA_TX_MAX] =
{
    QN_UART0_BASE+0x00,     /*!< UART0 TX buffer address */
    QN_UART1_BASE+0x00,     /*!< UART1 TX buffer address */
    QN_SPI0_BASE+0x10,      /*!< SPI0 TX buffer address */
    QN_SPI1_BASE+0x10,      /*!< SPI1 TX buffer address */
    QN_PROP_BASE+0x00       /*!< Porprietary TX buffer address */
};

/// DMA RX peripheral address
static const uint32_t peripheral_src[DMA_RX_MAX] =
{
    QN_UART0_BASE+0x04,     /*!< UART0 RX buffer address */
    QN_UART1_BASE+0x04,     /*!< UART1 RX buffer address */
    QN_SPI0_BASE+0x14,      /*!< SPI0 RX buffer address */
    QN_SPI1_BASE+0x14,      /*!< SPI1 RX buffer address */
    QN_PROP_BASE+0x04,      /*!< Porprietary RX buffer address */
    QN_ADC_BASE+0x10        /*!< ADC buffer address */
};


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

#if CONFIG_DMA_DEFAULT_IRQHANDLER==TRUE
/**
 ****************************************************************************************
 * @brief DMA interrupt handler
 ****************************************************************************************
 */
void DMA_IRQHandler(void)
{
    uint32_t reg;

    dev_allow_sleep(PM_MASK_DMA_ACTIVE_BIT);
    reg = dma_dma_GetIntStatus(QN_DMA);
    if (reg & DMA_MASK_DONE) {
        /* clear interrupt flag */
        dma_dma_ClrIntStatus(QN_DMA, DMA_MASK_DONE);

#if DMA_CALLBACK_EN==TRUE
        // Call end of Transfer callback
        if (dma_env.callback != NULL) {
            dma_env.callback();
        }
#endif
    }
    else if (reg & DMA_MASK_ERRO) {
         /* clear interrupt flag */
        dma_dma_ClrIntStatus(QN_DMA, DMA_MASK_ERRO);
    }
}
#endif /* CONFIG_DMA_DEFAULT_IRQHANDLER==TRUE */


/**
 ****************************************************************************************
 * @brief Initialize DMA controller
 * @description
 *  This function is used to clear callback pointer and enable DMA NVIC IRQ.
 ****************************************************************************************
 */
void dma_init(void)
{
//    uint32_t reg;
#if DMA_CALLBACK_EN==TRUE
    dma_env.callback = NULL;
#endif

    dma_clock_on();
    dma_reset();

    /* Wait until current DMA complete */
    while (dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_BUSY);

#if CONFIG_DMA_ENABLE_INTERRUPT==TRUE
    reg = DMA_MASK_DONE_IE                   /* dma done interrupt enable */
        | DMA_MASK_ERROR_IE                  /* dma error interrupt enable */
        | DMA_MASK_INT_EN                    /* enable dma int */
        ;
    dma_dma_SetCR(QN_DMA, reg);

    /* Enable the DMA Interrupt */
    NVIC_EnableIRQ(DMA_IRQn);
#endif
}

/**
 ****************************************************************************************
 * @brief  Check DMA status
 * @return DMA status 
 * @description
 *  This function is used to check DMA status.
 *****************************************************************************************
 */
int dma_check_status(void)
{
    if (dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_BUSY) {
        return DMA_BUSY;
    }
    else {
        return DMA_FREE;
    }
}

/**
 ****************************************************************************************
 * @brief DMA abort
 * @description
 *  This function is used to abort current DMA transfer, and usually used in undefined transfer length mode.
 ****************************************************************************************
 */
void dma_abort(void)
{
    dma_dma_SetAbort(QN_DMA);
    dev_allow_sleep(PM_MASK_DMA_ACTIVE_BIT);
}

/**
 ****************************************************************************************
 * @brief DMA memory copy
 * @param[in]    src_addr     source start address
 * @param[in]    dst_addr     destination start address
 * @param[in]    size         size of transfer
 * @param[in]    callback     callback after transfer
 * @description
 *  This function is used to transfer data from memory to memory by DMA.
 ****************************************************************************************
 */
void dma_memory_copy (uint32_t src_addr, uint32_t dst_addr, uint32_t size, void (*callback)(void))
{
    uint32_t mask, reg;

#if DMA_CALLBACK_EN==TRUE
    dma_env.callback = callback;
#endif

    dma_dma_SetSRC(QN_DMA, src_addr);
    dma_dma_SetDST(QN_DMA, dst_addr);

    dev_prevent_sleep(PM_MASK_DMA_ACTIVE_BIT);

    mask = ~DMA_MASK_ALL_INT_EN;
    reg = (size << DMA_POS_TRANS_SIZE)        /* dma transfer size */
        | DMA_MASK_START;                     /* enable dma */
    dma_dma_SetCRWithMask(QN_DMA, mask, reg);
}

/**
 ****************************************************************************************
 * @brief DMA form memory to fix
 * @param[in]    mode           transfer mode: byte, half word, word
 * @param[in]    src_addr       source address
 * @param[in]    dst_index      destination peripheral index
 * @param[in]    size           size of transfer
 * @param[in]    tx_callback    callback after transfer
 * @description
 *  This function is used to transfer data from memory to peripheral by DMA.
 ****************************************************************************************
 */
void dma_tx(enum DMA_TRANS_MODE mode, uint32_t src_addr, enum DMA_PERIPHERAL_TX dst_index, uint32_t size, void (*tx_callback)(void))
{
    uint32_t mask, reg;

#if DMA_CALLBACK_EN==TRUE
    dma_env.callback = tx_callback;
#endif

    dma_dma_SetSRC(QN_DMA, src_addr);
    dma_dma_SetDST(QN_DMA, peripheral_dst[dst_index]);

    dev_prevent_sleep(PM_MASK_DMA_ACTIVE_BIT);

    mask = ~DMA_MASK_ALL_INT_EN;   
    reg = (size << DMA_POS_TRANS_SIZE)        /* dma transfer size */
        | (dst_index << DMA_POS_DST_MUX)      /* select dst peripheral */
        | (mode << DMA_POS_TRANS_MODE)        /* transfer mode */
        | DMA_MASK_DST_REQ_EN
        | DMA_MASK_DST_ADDR_FIX               /* fix dst address */
        | DMA_MASK_START;                     /* enable dma */
    dma_dma_SetCRWithMask(QN_DMA, mask, reg);
}

/**
 ****************************************************************************************
 * @brief DMA form fix to memory
 * @param[in]    mode         transfer mode: byte, half word, word
 * @param[in]    src_index    source peripheral index
 * @param[in]    dst_addr     destination address
 * @param[in]    size         size of transfer, the max size is 0x7FF
 * @param[in]    rx_callback  callback after transfer
 * @description
 *  This function is used to transfer data from peripheral to memory by DMA.
 ****************************************************************************************
 */
void dma_rx(enum DMA_TRANS_MODE mode, enum DMA_PERIPHERAL_RX src_index, uint32_t dst_addr, uint32_t size, void (*rx_callback)(void))
{
    uint32_t mask, reg;

#if DMA_CALLBACK_EN==TRUE
    dma_env.callback = rx_callback;
#endif

    dma_dma_SetSRC(QN_DMA, peripheral_src[src_index]);
    dma_dma_SetDST(QN_DMA, dst_addr);
    
#if UART_RX_DMA_EN==FALSE
    dev_prevent_sleep(PM_MASK_DMA_ACTIVE_BIT);
#endif
    
    mask = ~DMA_MASK_ALL_INT_EN; 
    reg = (size << DMA_POS_TRANS_SIZE)        /* dma transfer size */
        | (src_index << DMA_POS_SRC_MUX)      /* select src peripheral */
        | (mode << DMA_POS_TRANS_MODE)        /* transfer mode */
        | DMA_MASK_SRC_REQ_EN
        | DMA_MASK_SRC_ADDR_FIX               /* fix src address */
        | DMA_MASK_START;                     /* enable dma */
    dma_dma_SetCRWithMask(QN_DMA, mask, reg);
}

/**
 ****************************************************************************************
 * @brief DMA form peripheral to peripheral
 * @param[in]    src_index          source peripheral index
 * @param[in]    dst_index          destination peripheral index
 * @param[in]    size               size of transfer
 * @param[in]    trans_callback     callback after transfer finish
 * @description
 *  This function is used to transfer data from peripheral to peripheral by DMA.
 ****************************************************************************************
 */
void dma_transfer(enum DMA_PERIPHERAL_RX src_index, enum DMA_PERIPHERAL_TX dst_index, uint32_t size, void (*trans_callback)(void))
{
    uint32_t mask, reg;

#if DMA_CALLBACK_EN==TRUE
    dma_env.callback = trans_callback;
#endif

    dma_dma_SetSRC(QN_DMA, peripheral_src[src_index]);
    dma_dma_SetDST(QN_DMA, peripheral_dst[dst_index]);

    dev_prevent_sleep(PM_MASK_DMA_ACTIVE_BIT);

    mask = ~DMA_MASK_ALL_INT_EN;
    reg = (size << DMA_POS_TRANS_SIZE)        /* dma transfer size */
        | (src_index << DMA_POS_SRC_MUX)      /* select src peripheral */
        | (dst_index << DMA_POS_DST_MUX)      /* select dst peripheral */
        | (DMA_TRANS_WORD << DMA_POS_TRANS_MODE) /* transfer mode */
        | DMA_MASK_SRC_REQ_EN
        | DMA_MASK_SRC_ADDR_FIX               /* fix src address */
        | DMA_MASK_DST_REQ_EN
        | DMA_MASK_DST_ADDR_FIX               /* fix dst address */
#if DMA_UNDEFINE_LENGTH_EN==TRUE
        | DMA_MASK_SRC_UDLEN
#endif
        | DMA_MASK_START;                     /* enable dma */
    dma_dma_SetCRWithMask(QN_DMA, mask, reg);
}

#endif /* CONFIG_ENABLE_DRIVER_DMA==TRUE */
/// @} DMA
