/**
  ******************************************************************************
  * @file    EXPANDITS.C
  * @author  Johnson.Li@NXP.COM
  * @version V1.0.0
  * @date    03-08-2016
  * @brief   This function use for expand 1 byte to 5 bytes
  ******************************************************************************
  */

#include "string.h" 

typedef unsigned char uint8_t;

#define EXPBIT 5
#define INBIT  8

const uint8_t BitTable[]      = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
const uint8_t BitShiftTable[] = {                          
                                 0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00, // 00-07
                                 0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00, // 08-15
                                 0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00, // 16-23
                                 0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00, // 24-31
                                 0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00  // 32-39
                                };

/**
  * @brief  This function use for expand 1 byte to 5 bytes.
  * @note   Using this function for 5 times divide the data transfer frenquency.
  * @param  input,*output
  * @retval None
  */
void expandbits1to5(uint8_t input,uint8_t *output)
{
    uint8_t i,j = 0;
    uint8_t InputBitVal     = 0; // record current deal input  bits value 
    uint8_t OutputBitCount  = 0; // record current deal output bits
    uint8_t OutputByteCount = 0; // record current deal output bytes    
    memset(output,0x00,EXPBIT);
    for (i = 0; i < INBIT; i++)
    {
        InputBitVal = (input & BitTable[i]) >> (INBIT-i-1);
        for (j = 0; j < EXPBIT; j++)
        {               
            OutputByteCount = (OutputBitCount) / INBIT;
            output[OutputByteCount] |= (InputBitVal << (BitShiftTable[OutputBitCount]));
            OutputBitCount++;
        }
    } 
}
