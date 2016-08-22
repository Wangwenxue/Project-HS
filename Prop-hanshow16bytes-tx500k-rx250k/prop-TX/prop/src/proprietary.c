/**
 ****************************************************************************************
 *
 * @file proprietary.c
 *
 * @brief Proprietary mode driver for QN9020.
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
 * @addtogroup  PROPRIETARY
 * @{
 ****************************************************************************************
 */
 
/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "proprietary.h"
#include "dma.h"
#include "qnrf.h"

extern void expandbits1toN(uint8_t input,uint8_t *output);

#if CONFIG_ENABLE_DRIVER_PROP==TRUE

/**
 *  Proprietary packet:
 *               +---------------------------------------------------------------------------------+
 *               |                             Little  Endian                                      |
 * +-------------+--------------+--------------------+----------------------------+----------------+
 * | Organization|   Preamble   |   Access Address   |           Payload          |       CRC      |
 * +-------------+--------------+--------------------+----------------------------+----------------+
 * |   Length    |   1~8Bytes   |     3/4/5Bytes     |         0~2048Bytes        | 0/8/16/24Bits  |
 * +-------------+--------------+--------------------+----------------------------+----------------+
 *  
 */

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/* Prop transmit buffer include: Preamble, Access address and Payload */
static uint8_t prop_tx_buf[PROP_TX_BUF_SIZE];

const uint32_t rf_freq_tab[] =
{
                // Freq (GHz)
    0x00000,    // 0  2.400
    0x05555,    // 1  2.400999985
    0x0aaab,    // 2  2.402000015
    0x10000,    // 3  2.403
    0x15555,    // 4  2.403999985
    0x1aaab,    // 5  2.405000015
    0x20000,    // 6  2.406
    0x25555,    // 7  2.406999985
    0x2aaab,    // 8  2.408000015
    0x30000,    // 9  2.409
    0x35555,    // 10 2.409999985
    0x3aaab,    // 11 2.411000015
    0x40000,    // 12 2.412
    0x45555,    // 13 2.412999985
    0x4aaab,    // 14 2.414000015
    0x50000,    // 15 2.415
    0x55555,    // 16 2.415999985
    0x5aaab,    // 17 2.417000015
    0x60000,    // 18 2.418
    0x65555,    // 19 2.418999985
    0x6aaab,    // 20 2.420000015
    0x70000,    // 21 2.421
    0x75555,    // 22 2.421999985
    0x7aaab,    // 23 2.423000015
    0x80000,    // 24 2.424
    0x85555,    // 25 2.424999985
    0x8aaab,    // 26 2.426000015
    0x90000,    // 27 2.427
    0x95555,    // 28 2.427999985
    0x9aaab,    // 29 2.429000015
    0xa0000,    // 30 2.430
    0xa5555,    // 31 2.430999985
    0xaaaab,    // 32 2.432000015
    0xb0000,    // 33 2.433
    0xb5555,    // 34 2.433999985
    0xbaaab,    // 35 2.435000015
    0xc0000,    // 36 2.436
    0xc5555,    // 37 2.436999985
    0xcaaab,    // 38 2.438000015
    0xd0000,    // 39 2.439
    0xd5555,    // 40 2.439999985
    0xdaaab,    // 41 2.441000015
    0xe0000,    // 42 2.442
    0xe5555,    // 43 2.442999985
    0xeaaab,    // 44 2.444000015
    0xf0000,    // 45 2.445
    0xf5555,    // 46 2.445999985
    0xfaaab,    // 47 2.447000015
    0x0    ,    // 48 2.448
    0x5555 ,    // 49 2.448999985
    0x0aaab,    // 50 2.450000015
    0x10000,    // 51 2.451
    0x15555,    // 52 2.451999985
    0x1aaab,    // 53 2.453000015
    0x20000,    // 54 2.454
    0x25555,    // 55 2.454999985
    0x2aaab,    // 56 2.456000015
    0x30000,    // 57 2.457
    0x35555,    // 58 2.457999985
    0x3aaab,    // 59 2.459000015
    0x40000,    // 60 2.46       
    0x45555,    // 61 2.460999985
    0x4aaab,    // 62 2.462000015
    0x50000,    // 63 2.463
    0x55555,    // 64 2.463999985
    0x5aaab,    // 65 2.465000015
    0x60000,    // 66 2.466
    0x65555,    // 67 2.466999985
    0x6aaab,    // 68 2.468000015
    0x70000,    // 69 2.469
    0x75555,    // 70 2.469999985
    0x7aaab,    // 71 2.471000015
    0x80000,    // 72 2.472
    0x85555,    // 73 2.472999985
    0x8aaab,    // 74 2.474000015
    0x90000,    // 75 2.475
    0x95555,    // 76 2.475999985
    0x9aaab,    // 77 2.477000015
    0xa0000,    // 78 2.478
    0xa5555,    // 79 2.478999985
    0xaaaab,    // 80 2.480000015
    0xb0000,    // 81 2.481
    0xb5555,    // 82 2.481999985
    0xbaaab,    // 83 2.483000015
};
/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

#define PROP_PREAMBLE_NUM       (((dp_dp_GetReg(0x04) & DP_MASK_PROP_PRE_NUM) >> DP_POS_PROP_PRE_NUM) + 1)

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief  Proprietary control register reset
 *****************************************************************************************
 */
__STATIC_INLINE void prop_ctrl_reset(void)
{
    prop_prop_SetCrWithMask(QN_PROP, PROP_MASK_PROP_CLR, PROP_MASK_PROP_CLR);
    prop_prop_SetCrWithMask(QN_PROP, PROP_MASK_PROP_CLR, 0);
}

/**
 ****************************************************************************************
 * @brief  Proprietary initialization
 * @param[in]    preamble_num   Preamble length
 * @param[in]    crc_num        CRC length
 * @param[in]    bit_order      Bit order
 * @param[in]    data_rate      Data rate
 *****************************************************************************************
 */
void prop_init(enum PROP_PREAMBLE_NUM_TYPE preamble_num, enum PROP_CRC_NUM_TYPE crc_num, enum PROP_DATA_RATE_TYPE data_rate)
{
    dp_dp_SetReg(0x00, ((uint32_t)0x1 << DP_POS_DET_MODE)
                     | ((uint32_t)0x3 << DP_POS_RX_MODE)
                     | ((uint32_t)0x1 << DP_POS_TX_EN_SEL)
                     | ((uint32_t)0x1 << DP_POS_RX_EN_SEL)
                     | ((uint32_t)0x80<< DP_POS_RX_H_IDX)
                     | ((uint32_t)0x1 << DP_POS_PDU_LEN_SEL)
                     | ((uint32_t)0x1 << DP_POS_AA_SEL));
    
    dp_dp_SetReg(0x04, ((uint32_t)0x5 << DP_POS_TX_POWER_DN_TIME)
                     | ((uint32_t)0x0 << DP_POS_PROP_DIRECTION_MODE)
                     | ((uint32_t)0x0 << DP_POS_PROP_DIRECTION_RATE)
                     | ((uint32_t)data_rate << DP_POS_PROP_DATA_RATE)
                     | ((uint32_t)preamble_num << DP_POS_PROP_PRE_NUM)
                     | ((uint32_t)crc_num << DP_POS_PROP_CRC_NUM));
    
    prop_prop_SetCrWithMask(QN_PROP, PROP_MASK_BIT_ORDER, PROP_BIT_ORDER_MSB);
    
    prop_ctrl_reset();
    
    dma_init();
}


/**
 ****************************************************************************************
 * @brief  Proprietary frequency set
 * @param[in]    rf_mode        RF in tx or rx mode
 * @param[in]    rf_idx         Proprietary RF frquency table index
 * @note
 *  rf_freq_tab[] is LO_FRAC part value
 *  RF frequency = 48 * (32 + LO_INT + LO_FRAC / 2 ^ 20) => 48 * (32 + LO_INT + rf_freq_tab[] / 2 ^ 20)
 *  eg. 
 *      rf_idx = 24 ===> rf_freq_tab[rf_idx] = 0x80000
 *      48 * (32 + 0x12 + 0x80000 / 2 ^ 20) = 2424Mhz
 *
 *      rf_idx = 76 ===> rf_freq_tab[rf_idx] = 0x95555
 *      48 * (32 + 0x13 + 0x95555 / 2 ^ 20) = 2475.999985MHz
 *
 *  If RF_TX frequecy = 2424MHz => RF_Rx_IMR0 = 2424 - 1 = 2423MHz
 *                              => RF_Rx_IMR1 = 2424 + 1 = 2425MHz
 *****************************************************************************************
 */
static void rf_set_freq(enum RF_MODE rf_mode, uint8_t rf_idx)
{
    uint32_t mask, reg;

    mask = SYSCON_MASK_EN_DATA_VLD
         | SYSCON_MASK_LO_REG
         | SYSCON_MASK_LO_FRAC
         | SYSCON_MASK_LO_INT;

    if (rf_mode == RF_RX_IMR0) {
        rf_idx--;
    }
    else if (rf_mode == RF_RX_IMR1) {
        rf_idx++;
    }

    reg =  SYSCON_MASK_LO_REG
         | (rf_freq_tab[rf_idx] << SYSCON_POS_LO_FRAC)  /* LO_FRAC */
         | ((rf_idx > 47) ? 0x13 : 0x12);               /* LO_INT */

    syscon_SetLO2WithMask(QN_SYSCON, mask, reg);
}

/**
 ****************************************************************************************
 * @brief  Proprietary mode transmit
 * @param[in]    ble_ch_idx     Channel frequency index
 *      Get this parameter from rf_freq_tab[](in file qnrf.c) accroding to the frequency
 *      chosen
 * @param[in]    aa_buf         Access address buffer, destination device address
 * @param[in]    aa_num         Access address length
 * @param[in]    data_buf       Transmit data buffer
 * @param[in]    data_len       Transmit data length
 * @param[in]    tx_callback    Called after transmission complete
 *****************************************************************************************
 */
void prop_mode_tx(uint32_t ble_ch_idx, uint8_t *aa_buf, enum PROP_AA_NUM aa_num, uint8_t *data_buf, uint16_t data_len)
{
    uint8_t preamble_byte;
    uint8_t preamble_num;
    
    uint16_t i;
    
    /* Wait for last transfer complete */
    while(dp_dp_GetReg(0x38) & DP_MASK_TX_BUSY);
    
    /* Channel configure */
    rf_set_freq(RF_TX, ble_ch_idx);
    
    /* Access Address and Payload length, Note: Payload length in bit */
    dp_dp_SetRegWithMask(0x00, DP_MASK_RX_PDU_LEN_IN, data_len << 3);
    dp_dp_SetRegWithMask(0x04, DP_MASK_PROP_AA_NUM, aa_num << DP_POS_PROP_AA_NUM);
    
    /* Preamble pack */
    preamble_num  = PROP_PREAMBLE_NUM;
    preamble_byte = (prop_tx_buf[0] & 0x80) ? 0xAA : 0x55;
    for(i = 0; i < preamble_num; i++)
    {
        prop_tx_buf[i] = preamble_byte;
    }
    
    /* Access Address pack */
    aa_num += 2;
    preamble_num  = PROP_PREAMBLE_NUM;
    for(i = 0; i < aa_num; i++)
    {
        prop_tx_buf[preamble_num+i] = aa_buf[aa_num-1-i];
    }
    
    /* Payload pack */
    for(i = 0; i < data_len; i++)
    {
        prop_tx_buf[preamble_num + aa_num + i] = data_buf[i];
    }
    
    /* Enable transmit request */
    dp_dp_SetRegWithMask(0x00, DP_MASK_TX_REQ, 0);
    dp_dp_SetRegWithMask(0x00, DP_MASK_TX_REQ, DP_MASK_TX_REQ);
    
    //Wait for ready(40us)
    delay(70);
    
    /* Transmit pack by DMA */
    prop_ctrl_reset();
    dma_tx(DMA_TRANS_BYTE, (uint32_t)prop_tx_buf, DMA_PROP_TX, (preamble_num + aa_num + data_len), NULL);
    
    /* Wait for transfer complete */
    while(!(dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_DONE));
    dma_dma_ClrIntStatus(QN_DMA, DMA_MASK_DONE);
    
    /* Wait for complete */
    while(dp_dp_GetReg(0x38) & DP_MASK_TX_BUSY);
}



/**
 ****************************************************************************************
 wenxue
 * @brief  Proprietary mode transmit used for expand every byte two times
 * @param[in]    ble_ch_idx     Channel frequency index
 *      Get this parameter from rf_freq_tab[](in file qnrf.c) accroding to the frequency
 *      chosen
 * @param[in]    aa_buf         Access address buffer, destination device address
 * @param[in]    aa_num         Access address length
 * @param[in]    data_buf       Transmit data buffer
 * @param[in]    data_len       Transmit data length
 * @param[in]    tx_callback    Called after transmission complete
 *****************************************************************************************
 */
void prop_mode_tx2(uint32_t ble_ch_idx, uint8_t *aa_buf, enum PROP_AA_NUM aa_num, uint8_t *data_buf, uint16_t data_len)
{
    uint8_t preamble_byte;
    uint8_t preamble_num;
    
	  uint8_t preamble_byte_expand[2]; // wenxue 把0x55 或者0xAA 扩展为2字节
	  uint8_t aa_buf_expand[4*2]; // wenxue 把字节Access Address 扩展2倍
	  uint8_t data_buf_expand[data_len*2]; // wenxue 把Data数据扩展2倍
    uint16_t i;
    
    /* Wait for last transfer complete */
    while(dp_dp_GetReg(0x38) & DP_MASK_TX_BUSY);
    
    /* Channel configure */
    rf_set_freq(RF_TX, ble_ch_idx);
    
	
	  /* pdu length user programmed
       AA number 
       注意：这里设置data的长度,需要扩展, Acess Address 长度不变
   */
    /* Access Address and Payload length, Note: Payload length in bit */
    dp_dp_SetRegWithMask(0x00, DP_MASK_RX_PDU_LEN_IN, ((data_len+4)*2) << 3); // 前导和接入地址的扩展的字节数要在这里补上
    dp_dp_SetRegWithMask(0x04, DP_MASK_PROP_AA_NUM, aa_num << DP_POS_PROP_AA_NUM);
    
    /* Preamble pack */
   // preamble_num  = PROP_PREAMBLE_NUM;
    // preamble_byte = (prop_tx_buf[0] & 0x80) ? 0xAA : 0x55;
	//  preamble_byte = (aa_buf[0] & 0x80) ? 0xAA : 0x55; // wenxue 20160822
  //  for(i = 0; i < preamble_num; i++)
 //   {
  //      prop_tx_buf[i] = preamble_byte;
 //   }
 
    preamble_num  = PROP_PREAMBLE_NUM;
    preamble_byte = (aa_buf[0] & 0x80) ? 0xAA : 0x55;
    expandbits1toN(preamble_byte,preamble_byte_expand);
    for(i = 0; i < preamble_num; i++)
    {
        prop_tx_buf[2*i] = preamble_byte_expand[0];
			  prop_tx_buf[2*i+1] = preamble_byte_expand[1];		  
    }
    
//		expandbits1toN(aa_buf[0],&aa_buf_expand[0]);
//		expandbits1toN(aa_buf[1],&aa_buf_expand[2]);
//		expandbits1toN(aa_buf[2],&aa_buf_expand[4]);
//		expandbits1toN(aa_buf[3],&aa_buf_expand[6]);
		for(i = 0; i < preamble_num; i++)
    {
				expandbits1toN(aa_buf[i],&aa_buf_expand[2*i]);	  
    }

		/* Access Address pack */
    aa_num += 2; // wenxue aa_num=2+2=4
    preamble_num  = PROP_PREAMBLE_NUM;
		prop_tx_buf[preamble_num*2+0] = aa_buf_expand[aa_num*2-1-1]; //aa_buf_expand[6]; 
		prop_tx_buf[preamble_num*2+1] = aa_buf_expand[aa_num*2-1]; //aa_buf_expand[7]; 
		prop_tx_buf[preamble_num*2+2] = aa_buf_expand[4]; //aa_buf_expand[4]; 
		prop_tx_buf[preamble_num*2+3] = aa_buf_expand[5]; //aa_buf_expand[5]; 
	  prop_tx_buf[preamble_num*2+4] = aa_buf_expand[2]; //aa_buf_expand[2]; 
		prop_tx_buf[preamble_num*2+5] = aa_buf_expand[3]; //aa_buf_expand[3];
    prop_tx_buf[preamble_num*2+6] = aa_buf_expand[0]; //aa_buf_expand[0]; 
		prop_tx_buf[preamble_num*2+7] = aa_buf_expand[1]; //aa_buf_expand[1]; 		
		
		
		
//    /* Access Address pack */
//    aa_num += 2; // wenxue aa_num=2+2=4
//    preamble_num  = PROP_PREAMBLE_NUM;
//    for(i = 0; i < 2*aa_num; i++)
//    {
//        prop_tx_buf[preamble_num*2+i] = aa_buf_expand[aa_num*2-1-i];
//    }
    
	  for(i = 0; i < data_len; i++)
    {
		  	expandbits1toN(data_buf[i],&data_buf_expand[2*i]);
    }

			
    /* Payload pack */
    for(i = 0; i < data_len*2; i++)
    {
        prop_tx_buf[preamble_num*2 + aa_num*2 + i] = data_buf_expand[i];
    }
    
    /* Enable transmit request */
    dp_dp_SetRegWithMask(0x00, DP_MASK_TX_REQ, 0);
    dp_dp_SetRegWithMask(0x00, DP_MASK_TX_REQ, DP_MASK_TX_REQ);
    
    //Wait for ready(40us)
    delay(70);
    
    /* Transmit pack by DMA */
    prop_ctrl_reset();
    dma_tx(DMA_TRANS_BYTE, (uint32_t)prop_tx_buf, DMA_PROP_TX, (2*(preamble_num + aa_num + data_len)), NULL);
    
    /* Wait for transfer complete */
    while(!(dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_DONE));
    dma_dma_ClrIntStatus(QN_DMA, DMA_MASK_DONE);
    
    /* Wait for complete */
    while(dp_dp_GetReg(0x38) & DP_MASK_TX_BUSY);
}






/**
 ****************************************************************************************
 * @brief  Proprietary mode receive
 * @param[in]    ble_ch_idx     Channel frequency index
 *      Get this parameter from rf_freq_tab[](in file qnrf.c) accroding to the frequency
 *      chosen
 * @param[in]    aa_buf         Access address buffer, own device address
 * @param[in]    aa_num         Access address length
 * @param[in]    data_buf       Receive data buffer
 * @param[in]    data_len       Receive data length
 * @param[in]    tx_callback    Called after receive complete
 *****************************************************************************************
 */
void prop_mode_rx(uint32_t ble_ch_idx, uint8_t *aa_buf, enum PROP_AA_NUM aa_num, uint8_t *data_buf, uint16_t data_len)
{
    
    /* Wait for last transfer complete */
    while(dp_dp_GetReg(0x38)& DP_MASK_TX_BUSY);
    
    /* Channel configure */
    rf_set_freq(RF_RX_IMR0, ble_ch_idx);
    
    /* Access Address and Payload length, Note: Payload length in bit */
    dp_dp_SetRegWithMask(0x00, DP_MASK_RX_PDU_LEN_IN, data_len << 3);
    dp_dp_SetRegWithMask(0x04, 
                        (DP_MASK_PROP_AA_NUM | DP_MASK_PROP_AA_ADDR_IN),
                        (aa_num << DP_POS_PROP_AA_NUM) | (aa_buf[4]));
    dp_dp_SetReg(0x08, *(uint32_t *)aa_buf);
    
    /* Enable transmit request */
    dp_dp_SetRegWithMask(0x00, DP_MASK_RX_REQ, 0);
    dp_dp_SetRegWithMask(0x00, DP_MASK_RX_REQ, DP_MASK_RX_REQ);
    
    //Wait for ready(40us)
    delay(70);
    
    /* Receive pack by DMA */
    prop_ctrl_reset();
    dma_rx(DMA_TRANS_BYTE, DMA_PROP_RX, (uint32_t)data_buf, data_len, NULL);
    
    /* Wait for transfer complete */
    while(!(dma_dma_GetIntStatus(QN_DMA) & DMA_MASK_DONE));
    dma_dma_ClrIntStatus(QN_DMA, DMA_MASK_DONE);
}

/**
 ****************************************************************************************
 * @brief  Proprietary mode break
 *  This function should be called after transmission and receive complete to save power
 *****************************************************************************************
 */
void prop_mode_stop()
{
    /* DMA abort */
    dma_abort();
    
    /* Clear prop intface control register */
    prop_prop_SetCrWithMask(QN_PROP, PROP_MASK_PROP_CLR, PROP_MASK_PROP_CLR);
    
    /* Disable transmit and receive request */
    dp_dp_SetRegWithMask(0x00, DP_MASK_TX_REQ | DP_MASK_RX_REQ, 0);
}


#endif /* CONFIG_ENABLE_DRIVER_PROP==TRUE */

/// @} PROPRIETARY
