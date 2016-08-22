/****************************************************************************
 *   $Id:: mbist_main.c                                                      $
 *   Project: QUINTIC QN9020 MBIST test function
 *
 *   Description:
 *     This file contains QN9020 SRAM BIST function.
 *
 *   By Derek @2012.08.15
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * QUINTIC Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. QUINTIC Semiconductors
 * reserves the right to make changes in the software without
 * notification. QUINTIC Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/


#include "uart.h"
#include "proprietary.h"
#include "system.h"

#include "intc.h"


/**
 * Proprietary packet
 * +----------+----------------+-----------+--------+
 * | Preamble | Access Address |  Payload  |  CRC   |
 * +----------+----------------+-----------+--------+
 * |  4Bytes  |     4Bytes     |  256Bytes | 16Bits |
 * +----------+----------------+-----------+--------+
 */
#define PROP_TEST_TX            0


#define TRX_CHANNEL             10
#define PROP_PREAMBLE_LEN       4     // A7106 4 bytes
#define PROP_AA_LEN             4     // A7106 4 bytes
//#define PROP_PAYLOAD_LEN        256+2 // A7106 256 bytes
#define PROP_PAYLOAD_LEN        16+2 // A7106 256 bytes

#define PROP_CRC_LEN            0     // A7106 has special CRC

#if PROP_TEST_TX == 1
    #define PROP_TEST_RX        0
#else
    #define PROP_TEST_RX        1
#endif


// ---------------------------------------------------
#define PREAMBLE_NUM            PROP_PREAMBLE_NUM_4BYTE
#define CRC_NUM                 PROP_CRC_NUM_NO
#define DATA_RATE               PROP_DATA_RATE_250K // wenxue RX rate is 250K

//uint8_t AA[4] = {0x71, 0x76, 0x41, 0x29};
//uint8_t AA[4] = {0x52, 0x56, 0x78, 0x53};
uint8_t AA[4] = {0x53, 0x78, 0x56, 0x52};
uint8_t send_str[PROP_PAYLOAD_LEN];
uint8_t tx_buf[PROP_PREAMBLE_LEN + PROP_AA_LEN + PROP_PAYLOAD_LEN];
uint8_t rx_buf[PROP_PAYLOAD_LEN + PROP_CRC_LEN];
// ---------------------------------------------------


uint16_t sc_index = 0;
uint16_t start = 0;
uint16_t crc_val = 0;
extern uint16_t CRC16(uint8_t* data, uint16_t len);
int main (void)
{
//    int i=0,j=0,z=0;
//    for (j=0;j<4;j++)
//    {
//        start = j*64;
//        for (i = 0; i < 10; i++) // 1-10   - 10
//        {        
//            send_str[start+i] = 0x30+i;
//        }
//        z = 0;
//        for (i = 10; i < 36; i++) // 11-36 - 26
//        {        
//            send_str[start+i] = 0x41+z;
//            z++;
//        }
//        z = 0;
//        for (i = 36; i < 62; i++) // 37-62 - 26
//        {        
//            send_str[start+i] = 0x61+z;
//            z++;
//        }    
//            send_str[start+i] = 0x0d;
//            i++;
//            send_str[start+i] = 0x0a;
//    }
//    crc_val = CRC16(send_str,256);
//    send_str[256] = (crc_val >> 16) & 0xff;
//    send_str[257] = crc_val & 0xff;

     int i=0,j=0,z=0;
     for (i = 0; i < 10; i++) // 1-10   - 10
     {        
            send_str[start+i] = 0x30+i;
     }
		 z = 0;
		 for (i = 10; i < 14; i++) // 11-16 - 26
        {        
            send_str[start+i] = 0x41+z;
            z++;
        }
    
	 send_str[14] = 0x0d;
	 send_str[15] = 0x0a;
				
    crc_val = CRC16(send_str,16);
    send_str[16] = (crc_val >> 16) & 0xff;
    send_str[17] = crc_val & 0xff;
		
		

    SystemInit();   
    gpio_init(NULL);    
    uart_init(QN_UART0, __USART_CLK, UART_115200);
    uart_tx_enable(QN_UART0, MASK_ENABLE);
    uart_rx_enable(QN_UART0, MASK_ENABLE);    
    prop_init(PREAMBLE_NUM, CRC_NUM, DATA_RATE);
    
#if PROP_TEST_TX == 1
    printf("***********Welcome to Prop-Tx test*********** \r\n");
#else
    printf("***********Welcome to Prop-Rx test*********** \r\n");
#endif
    
    while(1)
    {        
#if PROP_TEST_TX == 1
        prop_mode_tx(TRX_CHANNEL, AA, PROP_AA_NUM_4BYTES, (uint8_t *)send_str, PROP_PAYLOAD_LEN);        
        delay(500000);	
        delay(500000);
        delay(500000);
        printf("Tx Data:%d,%d\r\n",sc_index++,sizeof(send_str));
        printf("%s",(uint8_t *)send_str);
#else
        memset(rx_buf, 0, (PROP_PAYLOAD_LEN + PROP_CRC_LEN));
        prop_mode_rx(TRX_CHANNEL, AA, PROP_AA_NUM_4BYTES, rx_buf, (PROP_PAYLOAD_LEN + PROP_CRC_LEN));
        // CRC check
        // if(dp_dp_GetReg(0x3c) & DP_MASK_CRC_ERROR)
        // {
        //    printf("==>CRC error\r\n");
        // }
        // else
        {
            // if (memcmp(rx_buf, (uint8_t *)send_str, PROP_PAYLOAD_LEN))
//            crc_val = CRC16(rx_buf,256);
//            if (
//               (rx_buf[256] == ((crc_val >> 16) & 0xff) ) &&
//               (rx_buf[257] == (crc_val & 0xff ))
//               )
            {
                printf("Rx Data:%d,%d\r\n",sc_index++,sizeof(rx_buf));
                printf("%s",rx_buf);
            }
        }         
#endif  // End of PROP_TEST_TX */
    }
}



