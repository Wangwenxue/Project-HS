/**
 ****************************************************************************************
 *
 * @file proprietary.h
 *
 * @brief Header file of proprietary for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#ifndef _PROP_H_
#define _PROP_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_PROP==TRUE

/*
 * Global variable
 ****************************************************************************************
 */


/*
 * INCLUDE FILES
 ****************************************************************************************
 */
 


/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */
 /* Including: Preamble + Access Address + Payload */
#define PROP_TX_BUF_SIZE        (4+4+256+2)  /* Size in byte */


/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

///Proprietary preamble num type
enum PROP_PREAMBLE_NUM_TYPE
{
    PROP_PREAMBLE_NUM_1BYTE = 0,
    PROP_PREAMBLE_NUM_2BYTE = 1,
    PROP_PREAMBLE_NUM_3BYTE = 2,
    PROP_PREAMBLE_NUM_4BYTE = 3,
    PROP_PREAMBLE_NUM_5BYTE = 4,
    PROP_PREAMBLE_NUM_6BYTE = 5,
    PROP_PREAMBLE_NUM_7BYTE = 6,
    PROP_PREAMBLE_NUM_8BYTE = 7
};

///Proprietary CRC num type
enum PROP_CRC_NUM_TYPE
{
    PROP_CRC_NUM_NO     = 0,
    PROP_CRC_NUM_8BITS  = 1,
    PROP_CRC_NUM_16BITS = 2,
    PROP_CRC_NUM_24BITS = 3
};

///Proprietary bit order type
enum PROP_BIT_ORDER_TYPE
{
    PROP_BIT_ORDER_LSB  = 0,
    PROP_BIT_ORDER_MSB  = 1
};

///Proprietary data rate type
enum PROP_DATA_RATE_TYPE
{
    PROP_DATA_RATE_1M   = 0,
    PROP_DATA_RATE_500K = 1,
    PROP_DATA_RATE_250K = 3
};

///Proprietary access address num
enum PROP_AA_NUM
{
    PROP_AA_NUM_3BYTES  = 1,
    PROP_AA_NUM_4BYTES  = 2,
    PROP_AA_NUM_5BYTES  = 3
};

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
 
void prop_init(enum PROP_PREAMBLE_NUM_TYPE preamble_num, enum PROP_CRC_NUM_TYPE crc_num, enum PROP_DATA_RATE_TYPE data_rate);
void prop_mode_tx(uint32_t ble_ch_idx, uint8_t *aa_buf, enum PROP_AA_NUM aa_num, uint8_t *data_buf, uint16_t data_len);
void prop_mode_rx(uint32_t ble_ch_idx, uint8_t *aa_buf, enum PROP_AA_NUM aa_num, uint8_t *data_buf, uint16_t data_len);

#endif /* CONFIG_ENABLE_DRIVER_PROP */

#endif /* _PROP_H_ */
