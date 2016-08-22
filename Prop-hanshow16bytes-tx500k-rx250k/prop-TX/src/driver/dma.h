/**
 ****************************************************************************************
 *
 * @file dma.h
 *
 * @brief Header file of DMA for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _DMA_H_
#define _DMA_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_DMA==TRUE
#include "syscon.h"

/**
 ****************************************************************************************
 * @defgroup DMA DMA Driver
 * @ingroup DRIVERS
 * @brief DMA driver
 *
 *  QN9020 contains a single channel DMA controller, which supports 4 types transfer modes.
 *  Its features are listed as follow:
 *    - Support fixed and increment address transfer
 *    - Support 4 types transfer modes:
 *      - Memory to memory: support word, half word, byte aligned address
 *      - Peripheral to memory: support word, half word, byte aligned address
 *      - Memory to Peripheral: support word, half word, byte aligned address
 *      - Peripheral to Peripheral: only support word aligned address
 *    - Programmable source address and destination address
 *    - Support undefined length transfer
 *    - Maximum fixed transfer length up to 2047 bytes
 *    - There isn't arbitration among several DMA requests, only one DMA request is selected
 *    - Support mask or unmask DMA request of peripheral
 *    - Data FIFO width is 32bits with depth one
 *    - DMA can be aborted immediately when in a transfer process by configuring DMA ABORT register.
 *      At the same time, DMA done interrupt will be generated
 *    - A DMA done interrupt is generated after DMA done
 *    - A DMA error interrupt is generated when AHB returns an error response
 *
 * @{
 *
 ****************************************************************************************
 */

/* \example dma_example.c
 * This is an example of how to use the DMA driver.
 */

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/// Enable undefined length transfers
#define DMA_UNDEFINE_LENGTH_EN        FALSE 
/// Mask of all DMA interrupt enable
#define DMA_MASK_ALL_INT_EN           (DMA_MASK_DONE_IE|DMA_MASK_ERROR_IE|DMA_MASK_INT_EN)

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/** Dma transfer mode */
enum DMA_TRANS_MODE
{
    DMA_TRANS_BYTE      = 0,        /*!< Set DMA transfer mode as byte transfer */
    DMA_TRANS_HALF_WORD = 1,        /*!< Set DMA transfer mode as half word transfer */
    DMA_TRANS_WORD      = 2         /*!< Set DMA transfer mode as word transfer */
};

/** Dma tx peripheral index */
enum DMA_PERIPHERAL_TX
{
    DMA_UART0_TX = 0,               /*!< Set DMA TX peripheral to UART0 TX */
    DMA_UART1_TX = 1,               /*!< Set DMA TX peripheral to UART1 TX */
    DMA_SPI0_TX  = 2,               /*!< Set DMA TX peripheral to SPI0 TX */
    DMA_SPI1_TX  = 3,               /*!< Set DMA TX peripheral to SPI1 TX */
    DMA_PROP_TX  = 4,               /*!< Set DMA TX peripheral to Proprietary TX */
    DMA_TX_MAX   = 5                /*!< DMA TX peripheral Total bumber */
};

/** Dma rx peripheral index */
enum DMA_PERIPHERAL_RX
{
    DMA_UART0_RX = 0,               /*!< Set DMA RX peripheral to UART0 RX */
    DMA_UART1_RX = 1,               /*!< Set DMA RX peripheral to UART1 RX */
    DMA_SPI0_RX  = 2,               /*!< Set DMA RX peripheral to SPI0 RX */
    DMA_SPI1_RX  = 3,               /*!< Set DMA RX peripheral to SPI1 RX */
    DMA_PROP_RX  = 4,               /*!< Set DMA RX peripheral to Proprietary RX */
    DMA_ADC      = 5,               /*!< Set DMA RX peripheral to ADC */
    DMA_RX_MAX   = 6                /*!< DMA RX peripheral Total bumber */
};

/// DMA status
enum DMA_STATE
{
    DMA_BUSY = 0,                       /*!< DMA busy */
    DMA_FREE = 2                        /*!< DMA free */
};

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
  
/**
 ****************************************************************************************
 * @brief   Enable DMA module clock
 * @description
 *  This function is used to enable DMA module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void dma_clock_on(void)
{
    // enable DMA module clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_DMA);
}

/**
 ****************************************************************************************
 * @brief   Disable DMA module clock
 * @description
 *  This function is used to disable DMA module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void dma_clock_off(void)
{
    // disable DMA module clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_DMA);
}

/**
 ****************************************************************************************
 * @brief   Reset DMA module
 * @description
 *  This function is used to reset DMA module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void dma_reset(void)
{
    // Reset DMA module
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_DMA_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_DMA_RST);
}

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

#if CONFIG_DMA_DEFAULT_IRQHANDLER==TRUE
void DMA_IRQHandler(void);
#endif
extern void dma_init(void);
extern int dma_check_status(void);
extern void dma_abort(void);
extern void dma_memory_copy (uint32_t src, uint32_t dst, uint32_t size, void (*callback)(void));
extern void dma_tx(enum DMA_TRANS_MODE mode, uint32_t src, enum DMA_PERIPHERAL_TX dst, uint32_t size, void (*tx_callback)(void));
extern void dma_rx(enum DMA_TRANS_MODE mode, enum DMA_PERIPHERAL_RX src, uint32_t dst, uint32_t size, void (*rx_callback)(void));
extern void dma_transfer(enum DMA_PERIPHERAL_RX src_index, enum DMA_PERIPHERAL_TX dst_index, uint32_t size, void (*trans_callback)(void));


/// @} DMA
#endif /* CONFIG_ENABLE_DRIVER_DMA==TRUE */
#endif /* _DMA_H_ */
