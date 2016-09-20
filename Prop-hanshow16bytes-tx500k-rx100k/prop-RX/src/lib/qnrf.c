/**
 ****************************************************************************************
 *
 * @file qnrf.c
 *
 * @brief RF driver for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.4 $
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup  QNRF
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "qnrf.h"
#if CONFIG_ENABLE_DRIVER_QNRF==TRUE


/*
 * STRUCTURE DEFINITIONS
 ****************************************************************************************
 */


/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

const uint8_t rf_tx_gain_tab[TX_GAIN_LEVEL_MAX] =
{
    0x00,        /*!< -20 dBm */
    0x01,        /*!< -18 dBm */
    0x02,        /*!< -16 dBm */
    0x04,        /*!< -14 dBm */
    0x05,        /*!< -12 dBm */
    0x06,        /*!< -10 dBm */
    0x08,        /*!<  -8 dBm */
    0x09,        /*!<  -6 dBm */
    0x0A,        /*!<  -4 dBm */
    0x0C,        /*!<  -2 dBm */
    0x0D,        /*!<   0 dBm */
    0x0E,        /*!<   2 dBm */
    0x0F         /*!<   4 dBm */
};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/**
 ****************************************************************************************
 * @brief  Set TX power level
 * @param[in]     txpwr             TX power level: TX_GAIN_LEVEL0 ~ TX_GAIN_LEVEL12
 *
 * 01111----------    4dBm
 * 11111----------    3dBm (#)
 * 01110----------    2dBm
 * 11110----------    1dBm (#)
 * 01101----------    0dBm
 *  
 * 01100----------   -2dBm
 * 01010----------   -4dBm
 * 01001----------   -6dBm
 * 01000----------   -8dBm
 * 00110----------   -10dBm
 * 00101----------   -12dBm
 * 00100----------   -14dBm
 * 00010----------   -16dBm
 * 00001----------   -18dBm
 * 00000----------   -20dBm
 * 
 ****************************************************************************************
 */
void rf_tx_power_level_set(enum TX_POWER txpwr)
{
    int level = TX_GAIN_LEVEL10;
    int reg98 = 0x00010000;
    int rega0 = 0x04001000;
    int regb8 = 0x00000000;
    int dc_dc_flag = syscon_GetPGCR0(QN_SYSCON) & SYSCON_MASK_PD_BUCK;

    switch (txpwr) {
    case TX_GAIN_LEVEL0:
    case TX_GAIN_LEVEL1:   
    case TX_GAIN_LEVEL2:
    case TX_GAIN_LEVEL3:
    case TX_GAIN_LEVEL4:
    case TX_GAIN_LEVEL5:
    case TX_GAIN_LEVEL6:
    case TX_GAIN_LEVEL8:
    case TX_GAIN_LEVEL9:
        level = txpwr + 1;
        break;
        
    case TX_GAIN_LEVEL7:  // -6dBm
    case TX_GAIN_LEVEL10: // 0dBm
    case TX_GAIN_LEVEL11: // 2dBm
        level = txpwr + 1;
        if (dc_dc_flag == 0) { //DC-DC enabled
            regb8 = 0x00000040;
        }
        break;
        
    case TX_GAIN_LEVEL12:
        level = txpwr;
        if (dc_dc_flag == 0) { //DC-DC enabled
            reg98 = 0x00030000;
            rega0 = 0xFC003000;
            regb8 = 0x00000040;
        }
        else {
            rega0 = 0xF4001000;
        }
        break;
        
    default:
        break;
    }
    
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A_BAK, reg98);
    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_BGSEL|SYSCON_MASK_VREG15|SYSCON_MASK_BUCK_VBG, rega0);
    syscon_SetAnalogCRWithMask(QN_SYSCON, SYSCON_MASK_PA_GAIN_BIT4_B1, regb8);

    syscon_SetGCRWithMask(QN_SYSCON, SYSCON_MASK_PA_GAIN, rf_tx_gain_tab[level]<<SYSCON_POS_PA_GAIN);
}


/**
 ****************************************************************************************
 * @brief  Get TX power level
 * @return  TX power level
 *****************************************************************************************
 */
uint32_t rf_tx_power_level_get(void)
{
    uint8_t gain;
    int level = TX_GAIN_LEVEL_ERR;

    gain = (syscon_GetGCR(QN_SYSCON) & SYSCON_MASK_PA_GAIN) >> SYSCON_POS_PA_GAIN;

    int flag;

    for (int i = 0; i < TX_GAIN_LEVEL_MAX; i++) {
        if (gain == rf_tx_gain_tab[i]) {
            if (i == 0) {
                level = 0;
            }
            else if (i == TX_GAIN_LEVEL12) {
                flag = syscon_GetIvrefX32(QN_SYSCON) & SYSCON_MASK_BGSEL;
                if (flag == SYSCON_MASK_BGSEL) {
                    level = i;
                }
                else {
                    level = i - 1;
                }
            }
            else {
                level = i - 1;
            }
            break;
        }
    }
    
    return (level);
}


#endif /* CONFIG_ENABLE_DRIVER_QNRF==TRUE */
/// @} QNRF
