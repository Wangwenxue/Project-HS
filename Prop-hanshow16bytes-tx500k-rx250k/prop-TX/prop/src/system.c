/**
 ****************************************************************************************
 *
 * @file system.c
 *
 * @brief User system setup and initial configuration source file.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */

#include "system.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "timer.h"
#include "pwm.h"
#include "dma.h"
#include "serialflash.h"
#include "adc.h"
#include "analog.h"
#include "calibration.h"
#include "lib.h"

/**
 ****************************************************************************************
 * @brief  GPIO functionn configuration.
 *
 * Set Pins as gpio pin or function pin
 *****************************************************************************************
 */
static void SystemIOCfg(void)
{
    // pin mux
    syscon_SetPMCR0(QN_SYSCON, P00_UART0_TXD_PIN_CTRL
                             | P01_GPIO_1_PIN_CTRL
                             | P02_GPIO_2_PIN_CTRL
                             | P03_GPIO_3_PIN_CTRL
                             | P04_GPIO_4_PIN_CTRL
                             | P05_GPIO_5_PIN_CTRL
                             | P06_SW_DAT_PIN_CTRL
                             | P07_SW_CLK_PIN_CTRL

                             | P10_GPIO_8_PIN_CTRL
                             | P11_GPIO_9_PIN_CTRL
                             | P12_GPIO_10_PIN_CTRL
                             | P13_GPIO_11_PIN_CTRL
                             | P14_GPIO_12_PIN_CTRL
                             | P15_GPIO_13_PIN_CTRL
                             | P16_GPIO_14_PIN_CTRL
                             | P17_UART0_RXD_PIN_CTRL
                             );
    syscon_SetPMCR1(QN_SYSCON, P20_UART1_RXD_PIN_CTRL
                             | P21_UART1_TXD_PIN_CTRL
                             | P22_GPIO_18_PIN_CTRL
                             | P23_GPIO_19_PIN_CTRL
                             | P24_GPIO_20_PIN_CTRL
                             | P25_GPIO_21_PIN_CTRL
                             | P26_GPIO_22_PIN_CTRL
                             | P27_GPIO_23_PIN_CTRL

                             | P30_GPIO_24_PIN_CTRL
                             | P31_GPIO_25_PIN_CTRL
                             | P32_GPIO_26_PIN_CTRL
                             | P33_GPIO_27_PIN_CTRL
                             | P34_GPIO_28_PIN_CTRL
                             | P35_GPIO_29_PIN_CTRL
                             | P36_GPIO_30_PIN_CTRL
                             );

    /**
     * Pin select
     **** USART1
     * Pin37 <--> USART1_CTS
     * Pin20 <--> USART1_RXD
     **** SPI0
     * Pin32 <--> SPI0_DIN
     * Pin33 <--> SPI0_DAT
     * Pin34 <--> SPI0_CLK
     * Pin35 <--> SPI0_CS0
     */
    syscon_SetPMCR2(QN_SYSCON, SYSCON_MASK_UART1_PIN_SEL | SYSCON_MASK_SPI0_PIN_SEL);

    // driver ability
    syscon_SetPDCR(QN_SYSCON, 0x0); // 0 : low driver, 1 : high driver

    // pin pull ( 00 : High-Z,  01 : Pull-down,  10 : Pull-up,  11 : Reserved )
    syscon_SetPPCR0(QN_SYSCON, 0xAAAA5AAA);
    syscon_SetPPCR1(QN_SYSCON, 0x2AAAAAAA);
}

/**
 ****************************************************************************************
 * @brief  Setup the microcontroller system.
 *
 *  Initialize the system clock and pins.
 *****************************************************************************************
 */
void SystemInit(void)
{
    /*
     **************************
     * Sub module clock setting
     **************************
     */
    syscon_SetIvrefX32WithMask(QN_SYSCON, SYSCON_MASK_DVDD12_SW_EN, MASK_ENABLE);
    syscon_set_sysclk_src(CLK_XTAL, __XTAL);
    
#if __XTAL == XTAL_32MHz
    calibration_init(XTAL_32M);
#else
    calibration_init(XTAL_16M);
#endif
    ref_pll_calibration();

    clk32k_enable(__32K_TYPE);
#if QN_32K_RCO
    rco_calibration();
#endif

    // Reset SPI1 module(since the default register value was changed in bootloader)
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_USART1_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_USART1_RST);
    
    /*
        Disable all peripheral clock, will be enabled in the driver initilization.
        The next function performs the equivalent effect of a collection of these functions.
    
        timer_clock_off(QN_TIMER0);
        timer_clock_off(QN_TIMER1);
        timer_clock_off(QN_TIMER2);
        timer_clock_off(QN_TIMER3);
        uart_clock_off(QN_UART0);
        uart_clock_off(QN_UART1);
        spi_clock_off(QN_SPI0);
        spi_clock_off(QN_SPI1);
        flash_clock_off();
        gpio_clock_off();
        adc_clock_off();
        dma_clock_off();
        pwm_clock_off();
    */
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_TIMER0
                            | SYSCON_MASK_GATING_TIMER1
                            | SYSCON_MASK_GATING_TIMER2
                            | SYSCON_MASK_GATING_TIMER3
                            | SYSCON_MASK_GATING_UART0
                            | SYSCON_MASK_GATING_UART1
                            | SYSCON_MASK_GATING_SPI0
                            | SYSCON_MASK_GATING_SPI1
                            | SYSCON_MASK_GATING_SPI_AHB
                            | SYSCON_MASK_GATING_GPIO
                            | SYSCON_MASK_GATING_ADC
                            | SYSCON_MASK_GATING_DMA
                            | SYSCON_MASK_GATING_PWM);
    
    // calibration changed system clock setting
    // Configure sytem clock.  
    syscon_set_sysclk_src(CLK_XTAL, __XTAL);
    syscon_set_ahb_clk(__AHB_CLK);
    syscon_set_apb_clk(__APB_CLK);
    syscon_set_ble_clk(__BLE_CLK);
    syscon_set_timer_clk(__TIMER_CLK);
    syscon_set_usart_clk((uint32_t)QN_UART0, __USART_CLK);
    syscon_set_usart_clk((uint32_t)QN_UART1, __USART_CLK);  
    

    /*
     **************************
     * IO configuration
     **************************
     */

    SystemIOCfg();

    /*
     **************************
     * Peripheral setting
     **************************
     */

}


/// @} SYSTEM_CONTROLLER

