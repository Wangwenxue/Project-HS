#include "ate_sys.h"
#include "driver_QN9020.h"
#include "gpio.h"
#include "syscon.h"
#include "calibration.h"
#include "qnrf.h"
//#include "rf.h"
#include "sleep.h"

extern RET_DATA_TYPE retention_data;

void ate_SystemIOCfg (void)
{
    /**
     *  P00 - USART0-TX
     *  P17 - USART0-RX
     */
    
    syscon_SetPMCR0(QN_SYSCON, (P00_UART0_TXD_PIN_CTRL) | 
                               (P17_UART0_RXD_PIN_CTRL) );
    
    return;
}

void SystemCoreClockUpdate(void)
{
    //// Switch on REF PLL power
    //syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_REF_PLL, MASK_DISABLE);
    //// PLL ready
    //while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));

// config ble clk into 16MHz
#if   __XTAL == CLK_16M
    // by pass clock divider
    //syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS, SYSCON_MASK_AHB_DIV_BYPASS);
      syscon_SetCMDCRWithMask(QN_SYSCON, (SYSCON_MASK_BLE_DIV_BYPASS
                                      | SYSCON_MASK_BLE_DIVIDER
                                      | SYSCON_MASK_APB_DIVIDER
                                      | SYSCON_MASK_AHB_DIV_BYPASS),
                                      ((0 << SYSCON_POS_BLE_DIV_BYPASS)
                                     | (0 << SYSCON_POS_BLE_DIVIDER)
                                     | (0 << SYSCON_POS_APB_DIVIDER)
                                     | SYSCON_MASK_AHB_DIV_BYPASS)
                                       );    
#elif __XTAL == CLK_32M
    syscon_SetCMDCRWithMask(QN_SYSCON, (SYSCON_MASK_BLE_DIV_BYPASS
                                      | SYSCON_MASK_BLE_DIVIDER
                                      | SYSCON_MASK_AHB_DIV_BYPASS),
                                      ((0 << SYSCON_POS_BLE_DIV_BYPASS)
                                     | (0 << SYSCON_POS_BLE_DIVIDER)
                                     | SYSCON_MASK_AHB_DIV_BYPASS)
                                       );
#else
#endif
    
    // close a lot of clock
    syscon_SetCRSS(QN_SYSCON, (SYSCON_MASK_GATING_TIMER0
                             | SYSCON_MASK_GATING_TIMER1
                             | SYSCON_MASK_GATING_TIMER2
                             | SYSCON_MASK_GATING_TIMER3
                             //| SYSCON_MASK_GATING_UART0
                             //| SYSCON_MASK_GATING_UART1
                             | SYSCON_MASK_GATING_SPI0
                             | SYSCON_MASK_GATING_SPI1
                             | SYSCON_MASK_GATING_ADC
                             | SYSCON_MASK_GATING_PWM
                             )
                             );

    return;
}

void ate_rst_gpio(void)
{
    gpio_write_pin(ATE_GPIO_RESULT_PIN, ATE_GPIO_FAILED);
    gpio_write_pin(ATE_GPIO_END_PIN, ATE_GPIO_FAILED);
}

void ate_gpio_init(void)
{
    //gpio_write_pin(ATE_GPIO_RESULT_PIN, ATE_GPIO_FAILED);
    //gpio_write_pin(ATE_GPIO_END_PIN, ATE_GPIO_FAILED);
    ate_rst_gpio();
    gpio_write_pin((ATE_WKUP_PIN | ATE_DUT_NOTE_PIN | ATE_DUT_RST_PIN | ATE_ENTER_SLP_PIN),
        GPIO_HIGH);
    gpio_set_direction((ATE_GPIO_RESULT_PIN | ATE_GPIO_END_PIN | ATE_PRNT_PIN), 
        GPIO_OUTPUT);
    
    gpio_set_direction((ATE_WKUP_PIN | ATE_DUT_NOTE_PIN | ATE_DUT_RST_PIN | ATE_ENTER_SLP_PIN),
    //gpio_set_direction((ATE_WKUP_PIN | ATE_ENTER_SLP_PIN),
        GPIO_INPUT);
    
    return;
}

void ate_gpio_output(bool r)
{
    gpio_write_pin(ATE_GPIO_RESULT_PIN, r ? ATE_GPIO_FAILED : ATE_GPIO_OK);
    gpio_write_pin(ATE_GPIO_END_PIN, ATE_GPIO_OK);
}

void test_wait(void)
{
    // wait reset signal
    while(1)
    {
        if (gpio_read_pin(ATE_DUT_RST_PIN) == GPIO_LOW)
        {
            ate_rst_gpio();
            break;
        }
    }
    
    // wait note gpio reverse
    while(1)
    {
        if (gpio_read_pin(ATE_DUT_NOTE_PIN) == GPIO_LOW)
        {
            break;
        }
    }
}


void powerup_calibration(enum CLK_TYPE cry_type)
{
    //uint32_t ppm;

    /*
     **************************
     * RF calibartion
     **************************
     */

    calibration_init(cry_type);

    // Do not set dis, prevent software reset
    cal_cal_SetCAL0WithMask(QN_CALIB, 
                            (CALIB_MASK_REF_CAL_DIS
                            | CALIB_MASK_RC_CAL_DIS),
                            MASK_DISABLE);

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS, MASK_DISABLE);

    cal_cal_SetCAL3WithMask(QN_CALIB,
                            (CALIB_MASK_ROS_CAL_Q_DIS
                            | CALIB_MASK_ROS_CAL_I_DIS
                            | CALIB_MASK_R_CAL_DIS),
                            MASK_DISABLE);

    dp_dp_SetRegWithMask(0x78, 0x40000000, 0x0);
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RX_PKDET, MASK_ENABLE);

    // REF PLL
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_DISABLE);
    cal_cal_SetCAL0WithMask(QN_CALIB, CALIB_MASK_REF_CAL_REQ, MASK_ENABLE);
    while (!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));    // Wait for REF PLL ready
    //ref_pll_calibration();

    // RC->LO->GMR->PA(skip)
    seq_calibration(0);
    // write back LO code
    cal_cal_SetCAL1(QN_CALIB, cal_cal_GetCAL1(QN_CALIB));

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS|CALIB_MASK_LO_FCAL_DIS, MASK_ENABLE);
    delay(500);

    // LO2->KVCO->DC
    freq_hop_calibration(1, 0);
    // write back kvco code
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_KCAL, cal_cal_GetCAL1(QN_CALIB));

    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS|CALIB_MASK_LO_FCAL_DIS, MASK_DISABLE);

    // Set PA code
    cal_cal_SetCAL4WithMask(QN_CALIB,
                            (CALIB_MASK_PA_CODE_TX
                            | CALIB_MASK_PA_CODE_RX),
                            ((0xf<<CALIB_POS_PA_CODE_TX)
                            | (0x0<<CALIB_POS_PA_CODE_RX)));

    // PA auto mode.
    cal_cal_SetCAL4WithMask(QN_CALIB, CALIB_MASK_PA_CAL_EN, MASK_DISABLE);

    // Dis REF PLL
    // Dis RC calibration
    // Skip LO calibration
    // skip KVCO calibration when channel switch
    // skip ROS calibration when channel switch
    cal_cal_SetCAL0WithMask(QN_CALIB, 
                            (CALIB_MASK_REF_CAL_DIS
                            | CALIB_MASK_RC_CAL_DIS
                            | CALIB_MASK_LO_CAL_SKIP
                            | CALIB_MASK_LO_KCAL_SKIP
                            | CALIB_MASK_CAL_DONE_DIS),
                            MASK_ENABLE);

    // Disable LO amplitude cal
    cal_cal_SetCAL1WithMask(QN_CALIB, CALIB_MASK_LO_ACAL_DIS, MASK_ENABLE);

    // Disable Q & I channel ROS calibration
    // Disable R cal
    //cal_cal_SetCAL3WithMask(QN_CALIB,
    //                        (CALIB_MASK_ROS_CAL_Q_DIS
    //                        | CALIB_MASK_ROS_CAL_I_DIS
    //                        | CALIB_MASK_R_CAL_DIS),
    //                        MASK_ENABLE);

    /*
    **************************
    * 32K clock calibartion
    * 32k xtal - set fixed ppm here
    * 32k rco  - shall do calibration periodically
    **************************
    */

    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, SYSCON_MASK_SEL_CLK_32K);    // Internal 32k RCO
    
    //RCO CAL - 32k
    rco_calibration();
    
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, ~SYSCON_MASK_SEL_CLK_32K);    // Internal 32k RCO

    // Enable Timer1 for 32k calibration
    //syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_TIMER1);

    // 32.768k crystal or internal 32k rco
    //syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_SEL_CLK_32K, SYSCON_MASK_SEL_CLK_32K);    // Internal 32k RCO

    // Calculate PPM of 32k clock
    //ppm = clock_32k_calibration(XTAL_16M, 16);
    //ppm = 0x80006000;

    // Set ppm for sleep compensation of BLE stack
    //set_32k_ppm(ppm);

    // Disable Timer for 32k calibration
    //syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER1);

    // Internal RCO can be powered down if do not use it.
    //syscon_SetPGCR0WithMask(QN_SYSCON, SYSCON_MASK_PD_RCO, MASK_ENABLE);
    //syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RCO, MASK_ENABLE);

    /*
     **************************
     * Save calibration result
     **************************
     */
    save_cal_data();

    // disable RF set by SW, enable set by HW
    rf_enable_sw_set_freq(MASK_DISABLE);
    rf_enable(RF_TX, MASK_DISABLE, MASK_DISABLE, 0x0);

    // Enable SD ADC RST again 2013 04 07 ljb
    dp_dp_SetRegWithMask(0x78, 0x40000000, ~0x0);
}

void ate_sys_init(uint32_t xtal)
{
    uint32_t mask, val;
    uint8_t xcsel;
    uint32_t dc_dc_flag = 1;
    
    /*
     **************************
     * Step 1. System Clock
     **************************
     */

    // external 16M/32M clk ready
    while(!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_CLK_RDY));
    // High frequency crystal
    syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_CLK_MUX, CLK_XTAL);

    // Switch on REF PLL power
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_REF_PLL, MASK_DISABLE);
    // PLL ready
    while(!(syscon_GetBLESR(QN_SYSCON) & SYSCON_MASK_REF_PLL_RDY));

    /*
     **************************
     * Step 2. QN9020 patch
     **************************
     */

    // Register 0x400000a8
    // Analog XTAL setting
    // LO PLL CP current : 0.7x
    // LC VCO amplitude settting: VTH+50mV
    val = (xtal>XTAL_16MHz)?0x80000000:0;
    syscon_SetLO1WithMask(QN_SYSCON,
                            (SYSCON_MASK_XDIV | SYSCON_MASK_LO_TST_CP | SYSCON_MASK_LO_VCO_AMP),
                            (val | (0x2<<SYSCON_POS_LO_TST_CP) | (0x1<<SYSCON_POS_LO_VCO_AMP)));

    // Register 0x400000a0
    // BGSEL = 0000
    // 1.5v regulator for analog output voltage - 1.5v
    // Enable TR switch
    // VREG12_D = 00
    // Enable switch between dvdd12_core and dvdd12_pmu
    // BUCK_DPD
    // 32.768kHz crystal bias current control 100000 -> 001000
    // BUCK VBG = 0x1
    mask = SYSCON_MASK_BGSEL
         | SYSCON_MASK_VREG15
         | SYSCON_MASK_VREG12_A
         | SYSCON_MASK_TR_SWITCH
         | SYSCON_MASK_VREG12_D
         | SYSCON_MASK_DVDD12_SW_EN
         | SYSCON_MASK_BUCK_DPD
         | SYSCON_MASK_X32ICTRL
         | SYSCON_MASK_BUCK_VBG;
    if(dc_dc_flag)
    {
        // dc-dc
        val = (0X0<<SYSCON_POS_BGSEL)
            | (0x1<<SYSCON_POS_VREG15)
            | (0x1<<SYSCON_POS_VREG12_A)
            | (SYSCON_MASK_TR_SWITCH)
            | (0x0<<SYSCON_POS_VREG12_D)
            | (SYSCON_MASK_DVDD12_SW_EN)
            | (0x1<<SYSCON_POS_BUCK_VBG)
            | (0x1<<16)
            | (0x8);
    }
    else
    {
        val = (0X0<<SYSCON_POS_BGSEL)
            | (0x1<<SYSCON_POS_VREG15)
            | (0x1<<SYSCON_POS_VREG12_A)
            | (SYSCON_MASK_TR_SWITCH)
            | (0x0<<SYSCON_POS_VREG12_D)
            | (SYSCON_MASK_DVDD12_SW_EN)
            | (0x2<<SYSCON_POS_BUCK_VBG)
            | (0x1<<17)
            | (0x1<<16)
            | (0x8);
    }
    syscon_SetIvrefX32WithMask(QN_SYSCON, mask, val);

    // Register 0x40000098
    // VREG12_A_BAK = 00
    syscon_SetPGCR2WithMask(QN_SYSCON, SYSCON_MASK_VREG12_A_BAK, (0x1<<SYSCON_POS_VREG12_A_BAK));

    // Register 0x4000009c
    // PA BM = 00
    // PA gain 2dbm
    mask = SYSCON_MASK_PA_GAIN_BOOST
         | SYSCON_MASK_BM_PA
         | SYSCON_MASK_PA_GAIN;
    if(dc_dc_flag)
    {
        val = (0x1<<SYSCON_POS_PA_GAIN_BOOST)   //enable gain boost to reduce pa gain 0.5db and 0.5ma current in tx mode from zhangshun
            | (0x3<<SYSCON_POS_BM_PA)
            | (0xf<<SYSCON_POS_PA_GAIN);
    }
    else
    {
        val = (0x0<<SYSCON_POS_PA_GAIN_BOOST)   //diable gain boost in no dcdc mode from zhangshun
            | (0x2<<SYSCON_POS_BM_PA)           //change to '10' increase the PA power from zhangshun 20130821
            | (0xe<<SYSCON_POS_PA_GAIN);
    }
    syscon_SetGCRWithMask(QN_SYSCON, mask, val);
    
    // Register 0x400000a4
    // XTAL speed up
    // buck tmos = 111
    // buck bm = 10
    // REFPLL CP current 1.3x
    mask = SYSCON_MASK_XSP_CSEL_B0
         | SYSCON_MASK_BUCK_TMOS
         | SYSCON_MASK_BUCK_BM
         | SYSCON_MASK_TST_CPREF;
    if(dc_dc_flag)
    {
        val = 0x80000000
            | (0X6<<SYSCON_POS_BUCK_TMOS) //change from 7 to 6 to reduce current and reduce sensitivity from yangyu
            | (0X2<<SYSCON_POS_BUCK_BM)
            | 0x8;
        
    }
    else
    {
        val = 0x80000000
            | (0X5<<SYSCON_POS_BUCK_TMOS) //change from 7 to 5 from test.
            | (0X2<<SYSCON_POS_BUCK_BM)
            | 0x8;
    }
    syscon_SetXtalBuckWithMask(QN_SYSCON, mask, val);

    // Register 0x400000b0
    // BM_PPF = 00
    // ADC_OPA12I = 00
    // ADC_OPA4I = 00
    mask = SYSCON_MASK_BM_PPF
         | SYSCON_MASK_ADC_OPA12I
         | SYSCON_MASK_ADC_OPA4I;
    if(dc_dc_flag)
    {
        val = (0x0<<SYSCON_POS_BM_PPF)
            |(0x0<<SYSCON_POS_ADC_OPA12I)
            |(0x0<<SYSCON_POS_ADC_OPA4I);
    }
    else
    {
        val = (0x2<<SYSCON_POS_BM_PPF) //change to '10' to increase the current and sensitivity from yangyu
            |(0x0<<SYSCON_POS_ADC_OPA12I)
            |(0x0<<SYSCON_POS_ADC_OPA4I);
    }
    syscon_SetRXCRWithMask(QN_SYSCON, mask, val);
    
    // xtal speed up
    syscon_SetAnalogCRWithMask(QN_SYSCON, SYSCON_MASK_XSP_CSEL_B1, MASK_ENABLE);

    // Register 0x400000bc
    // buck tmos back (tx) = 111
    // buck bm back (tx) = 10
    // Half LO CP OPAMP current
    // DIS XPD DLY = 1 to fix the slow power down time of vreg-a
    // PA_CK enable at the end of SPEED_UP
    syscon_SetAdditionCRWithMask(QN_SYSCON,
                                (SYSCON_MASK_BUCK_TMOS_BAK
                                | SYSCON_MASK_BUCK_BM_BAK
                                | SYSCON_MASK_HALF_LO_OPCUR
                                | SYSCON_MASK_XADD_C
                                | SYSCON_MASK_DIS_XPD_DLY
                                | SYSCON_MASK_PA_CKEN_SEL
                                | SYSCON_MASK_DC_CAL_MODE),
                                ((0x7<<SYSCON_POS_BUCK_TMOS_BAK)
                                | (0x2<<SYSCON_POS_BUCK_BM_BAK)
                                | 0x200
                                | 0x8
                                | 0x4
                                | 0x2
                                | 0x0)); // WJ ???

    // Register 0x4000c010
    // PLL settling time in KVCO calibration: 24us
    // Speed up time in KVCO calibration: 8us
    cal_cal_SetCAL4WithMask(QN_CALIB,
                            (CALIB_MASK_PLL_RDY_DLY |CALIB_MASK_LO_SU_DLY),
                            ((0x18<<CALIB_POS_PLL_RDY_DLY) | (0x8<<CALIB_POS_LO_SU_DLY)));

    // Datapath
    // Register 0x4000b04c
    // cordic min vin threshold
    dp_dp_SetRegWithMask(0x4c, 0x0f000000, 0x0);

    // Register 0x4000b078
    // bit[16] do not reset sdadc after fsinc
    // bit[0:1] rf agc adc reset delay
    dp_dp_SetRegWithMask(0x78, 0x00013000, 0x11000);

    // Register 0x4000b084
    dp_dp_SetRegWithMask(0x84,0x00000026,0x26);

    /*
     **************************
     * Step 3. Platform initialization ()
     *         rf, deep sleep
     **************************
     */
    rf_init((struct rf_api*)_ble_rf);

    //sleep_patch();

    // XTAL load capacitance
    xcsel = 0x11;
    syscon_SetXtalBuckWithMask(QN_SYSCON, SYSCON_MASK_XCSEL, xcsel<<SYSCON_POS_XCSEL);

    /*
     **************************
     * Step 4. Initialize BLE hardware
     **************************
     */

    // calibration
    //val &= 1;
    val = (xtal>XTAL_16MHz)?XTAL_32M:XTAL_16M;
    powerup_calibration((enum CLK_TYPE)val);
#if 0
    // Switch off REF PLL/LO_VCO/LO PLL/PA/LNA/LNA PKDET/MIXER... power
    syscon_SetPGCR1WithMask(QN_SYSCON,
                            (SYSCON_MASK_DIS_LO_VCO
                            | SYSCON_MASK_DIS_LO_PLL
                            | SYSCON_MASK_DIS_PA
                            | SYSCON_MASK_DIS_LNA
                            | SYSCON_MASK_DIS_LNA_PKDET
                            | SYSCON_MASK_DIS_MIXER
                            | SYSCON_MASK_DIS_PPF_PKDET
                            | SYSCON_MASK_DIS_PPF
                            | SYSCON_MASK_DIS_RX_PKDET
                            | SYSCON_MASK_DIS_RX_ADC
                            | SYSCON_MASK_DIS_RCO
                            | SYSCON_MASK_DIS_SAR_ADC
                            | SYSCON_MASK_DIS_SAR_BUF
                            | SYSCON_MASK_DIS_BUCK
                            //| SYSCON_MASK_DIS_REF_PLL
                            ),
                            MASK_ENABLE);
#endif
    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_RCO, MASK_DISABLE);
    syscon_SetPGCR0WithMask(QN_SYSCON, (SYSCON_MASK_PD_RCO
                                      | SYSCON_MASK_PD_XTAL32
                                      ),
                                       (SYSCON_MASK_PD_RCO
                                      | SYSCON_MASK_PD_XTAL32
                                      )
                                      );

    syscon_SetPDCR(QN_SYSCON, 0x00000000);

}

void gpio_prnt(uint8_t data)
{
    uint32_t reg;

    // Read register value
    reg = gpio_gpio_GetOutputData(QN_GPIO);
    
    reg &= (~ATE_PRNT_PIN);
    reg |= (data & 0x3F) | ((data >> 6) << 19);
    
    // Write register value
    gpio_gpio_SetOutputData(QN_GPIO, reg);
}

void rx_alwayson(void)
{
    // set rx en
    // rx enable
    // rx en sel
    dp_dp_SetRegWithMask(0x00,
                         (DP_MASK_RX_REQ |
                          DP_MASK_RX_EN_SEL),
                         (DP_MASK_RX_REQ |
                          DP_MASK_RX_EN_SEL)
                          );
                          
    // rx_en mode
    dp_dp_SetRegWithMask(0x30,
                         (0 << DP_POS_RX_EN_MODE),
                         DP_MASK_RX_EN_MODE
                         );
}

void tx_alwayson(void)
{
    // set tx related register
    // tx always on
    dp_dp_SetRegWithMask(0x04, 0x04000000, 0x04000000);
    // tx enable
    // tx en sel
    dp_dp_SetRegWithMask(0x00,
                         (DP_MASK_TX_REQ |
                          DP_MASK_TX_EN_SEL),
                         (DP_MASK_TX_REQ |
                          DP_MASK_TX_EN_SEL)
                          );
    // pa_clk_en = 0
    syscon_SetLO2WithMask(QN_SYSCON, SYSCON_MASK_EN_DATA_VLD, (~SYSCON_MASK_EN_DATA_VLD));
    
    // set gain to 0 dB
    syscon_SetGCRWithMask(QN_SYSCON, SYSCON_MASK_PA_GAIN, (0x0d << SYSCON_POS_PA_GAIN));
}

/**
 ****************************************************************************************
 * @brief Restore configuration which is saved before entering sleep mode
 ****************************************************************************************
 */
void save_cal_data(void)
{
    retention_data.cal_data[0] = cal_cal_GetCAL0(QN_CALIB)
                               & (CALIB_MASK_CH_CHG_CAL_EN
                                | CALIB_MASK_LO_CAL_SKIP
                                | CALIB_MASK_LO_KCAL_SKIP
                                | CALIB_MASK_CAL_DONE_DIS
                                | CALIB_MASK_REF_CAL_DIS
                                | CALIB_MASK_REF_CAL
                                | CALIB_MASK_RC_CAL_DIS
                                | CALIB_MASK_RC_CAL_DLY
                                | CALIB_MASK_RC_CAL );
    
    retention_data.cal_data[1] = cal_cal_GetCAL1(QN_CALIB)
                               & (CALIB_MASK_LO_ACAL_DIS
                                | CALIB_MASK_LO_ACAL_E
                                | CALIB_MASK_LO_ACAL
                                | CALIB_MASK_LO_SPEED_UP
                                | CALIB_MASK_LO_FCAL_DIS
                                | CALIB_MASK_LO_FCAL
                                | CALIB_MASK_LO_KCAL_DIS
                                | CALIB_MASK_LO_KCAL
                                | CALIB_MASK_EN_KCAL_SD
                                | CALIB_MASK_DS_SEL );
    
    retention_data.cal_data[2] = cal_cal_GetCAL3(QN_CALIB)
                                & (CALIB_MASK_PA_CAL_DIS
                                 | CALIB_MASK_PA_CAL
                                 | CALIB_MASK_R_CAL_DIS
                                 | CALIB_MASK_R_CAL
                                 | CALIB_MASK_ROS_CAL_I_DIS
                                 | CALIB_MASK_ROS_CAL_I
                                 | CALIB_MASK_ROS_CAL_Q_DIS
                                 | CALIB_MASK_ROS_CAL_Q );

    retention_data.cal_data[3] = cal_cal_GetCAL4(QN_CALIB)
                                & (CALIB_MASK_RCO_CAL_DIS
                                 | CALIB_MASK_RCO_CAL
                                 | CALIB_MASK_PA_CODE_TX
                                 | CALIB_MASK_PA_CODE_RX
                                 | CALIB_MASK_PLL_RDY_DLY
                                 | CALIB_MASK_LO_SU_DLY
                                 | CALIB_MASK_PA_CAL_EN );
}

void load_cal_data(void)
{
    cal_cal_SetCAL0(QN_CALIB, retention_data.cal_data[0]);
    cal_cal_SetCAL1(QN_CALIB, retention_data.cal_data[1]);
    cal_cal_SetCAL3(QN_CALIB, retention_data.cal_data[2]);
    cal_cal_SetCAL4(QN_CALIB, retention_data.cal_data[3]);
}

void save_sys_data(void)
{
    retention_data.sys_data[0] = *(volatile uint32_t*)(0x40000014);
    retention_data.sys_data[1] = *(volatile uint32_t*)(0x40000028);
    retention_data.sys_data[2] = *(volatile uint32_t*)(0x4000002c);
    retention_data.sys_data[3] = *(volatile uint32_t*)(0x4000009c);
    retention_data.sys_data[4] = *(volatile uint32_t*)(0x400000a4);
    retention_data.sys_data[5] = *(volatile uint32_t*)(0x400000a8);
    retention_data.sys_data[6] = *(volatile uint32_t*)(0x400000ac);
    retention_data.sys_data[7] = *(volatile uint32_t*)(0x400000b0);
    retention_data.sys_data[8] = *(volatile uint32_t*)(0x400000b4);
}

void load_sys_data(void)
{
    *(volatile uint32_t*)(0x40000014) = retention_data.sys_data[0];
    *(volatile uint32_t*)(0x40000028) = retention_data.sys_data[1];
    *(volatile uint32_t*)(0x4000002c) = retention_data.sys_data[2];
    *(volatile uint32_t*)(0x4000009c) = retention_data.sys_data[3];
    *(volatile uint32_t*)(0x400000a4) = retention_data.sys_data[4];
    *(volatile uint32_t*)(0x400000a8) = retention_data.sys_data[5];
    *(volatile uint32_t*)(0x400000ac) = retention_data.sys_data[6];
    *(volatile uint32_t*)(0x400000b0) = retention_data.sys_data[7];
    *(volatile uint32_t*)(0x400000b4) = retention_data.sys_data[8];
}

void led_flash(enum gpio_pin pin)
{
    int32_t i;
    uint32_t level;
    
    level = GPIO_LOW;
    for (i = 0; i < 10; i ++)
    {
        gpio_write_pin(pin, level);
        level = ~level;
        delay(100000);
    }
    
}

void sleep_recover(void)
{
#if __XTAL == XTAL_32MHz
    // external clock is 32MHz
    syscon_SetLO1WithMask(QN_SYSCON, SYSCON_MASK_XDIV, MASK_ENABLE);
#endif

    load_sys_data();
    load_cal_data();
    
    // clear pending interrupt, except sleep mode wakeup source: gpio
    NVIC->ICPR[0] = 0xfffffff0;
    
}

//void dc_dc_enable(bool enable)
//{
//    //dc_dc_flag = enable;

//    // DC-DC always bypass
//    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_BUCK_BYPASS, MASK_ENABLE);
//    
//    // Disable DC-DC
//    syscon_SetPGCR1WithMask(QN_SYSCON, SYSCON_MASK_DIS_BUCK, MASK_ENABLE);
//}

void slp_test(void)
{
//    do {
//        delay(10);
//    } while (gpio_read_pin(ATE_ENTER_SLP_PIN) == GPIO_HIGH);
//    
//    sleep_init();
//    wakeup_by_gpio(ATE_WKUP_PIN, GPIO_WKUP_BY_LOW, NULL);

//    save_sys_data();
//    enter_sleep(SLEEP_DEEP, WAKEUP_BY_GPIO, NULL); //Led_flash);
//    sleep_recover();
}
