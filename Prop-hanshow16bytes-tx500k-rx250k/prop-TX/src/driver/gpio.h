/**
 ****************************************************************************************
 *
 * @file gpio.h
 *
 * @brief Header file of GPIO for QN9020.
 *
 * Copyright(C) 2015 NXP Semiconductors N.V.
 * All rights reserved.
 *
 * $Rev: 1.0 $
 *
 ****************************************************************************************
 */
#ifndef _GPIO_H_
#define _GPIO_H_
#include "driver_config.h"
#if CONFIG_ENABLE_DRIVER_GPIO==TRUE

/**
 ****************************************************************************************
 * @defgroup GPIO GPIO Driver
 * @ingroup DRIVERS
 *
 * @brief Declaration of GPIO functions and definitions
 *
 *  QN9020 has up to 31 General Purpose I/O pins which can be shared with other function pins,
 *  depending on the pin mux configuration. The main features of GPIO are listed as follow:
 *    - Each one of the GPIO pins is independent and has the corresponding register bits to
 *      control the pin function mode and data.
 *    - The type of each I/O pins can be independently software configured as input, output,
 *      open-drain or pull-up mode.
 *
 * @{
 ****************************************************************************************
 */


/* \example gpio_example.c
 * This is an example of how to use the GPIO driver.
 */


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * ENUMERATION DEFINITIONS
 *****************************************************************************************
 */

/// Enumeration of GPIO pins
enum gpio_pin
{
    GPIO_P00 = 1 << 0,           /*!< PIN0.0 */
    GPIO_P01 = 1 << 1,           /*!< PIN0.1 */
    GPIO_P02 = 1 << 2,           /*!< PIN0.2 */
    GPIO_P03 = 1 << 3,           /*!< PIN0.3 */
    GPIO_P04 = 1 << 4,           /*!< PIN0.4 */
    GPIO_P05 = 1 << 5,           /*!< PIN0.5 */
    GPIO_P06 = 1 << 6,           /*!< PIN0.6 */
    GPIO_P07 = 1 << 7,           /*!< PIN0.7 */

    GPIO_P10 = 1 << 8,           /*!< PIN1.0 */
    GPIO_P11 = 1 << 9,           /*!< PIN1.1 */
    GPIO_P12 = 1 << 10,          /*!< PIN1.2 */
    GPIO_P13 = 1 << 11,          /*!< PIN1.3 */
    GPIO_P14 = 1 << 12,          /*!< PIN1.4 */
    GPIO_P15 = 1 << 13,          /*!< PIN1.5 */
    GPIO_P16 = 1 << 14,          /*!< PIN1.6 */
    GPIO_P17 = 1 << 15,          /*!< PIN1.7 */

    GPIO_P20 = 1 << 16,          /*!< PIN2.0 */
    GPIO_P21 = 1 << 17,          /*!< PIN2.1 */
    GPIO_P22 = 1 << 18,          /*!< PIN2.2 */
    GPIO_P23 = 1 << 19,          /*!< PIN2.3 */
    GPIO_P24 = 1 << 20,          /*!< PIN2.4 */
    GPIO_P25 = 1 << 21,          /*!< PIN2.5 */
    GPIO_P26 = 1 << 22,          /*!< PIN2.6 */
    GPIO_P27 = 1 << 23,          /*!< PIN2.7 */

    GPIO_P30 = 1 << 24,          /*!< PIN3.0 */
    GPIO_P31 = 1 << 25,          /*!< PIN3.1 */
    GPIO_P32 = 1 << 26,          /*!< PIN3.2 */
    GPIO_P33 = 1 << 27,          /*!< PIN3.3 */
    GPIO_P34 = 1 << 28,          /*!< PIN3.4 */
    GPIO_P35 = 1 << 29,          /*!< PIN3.5 */
    GPIO_P36 = 1 << 30,          /*!< PIN3.6 */

};

#define GPIO_P0  (uint32_t)(GPIO_P00 | GPIO_P01 | GPIO_P02 | GPIO_P03 | \
                            GPIO_P04 | GPIO_P05 | GPIO_P06 | GPIO_P07)      /*!< P00 - P07 */
               
#define GPIO_P1  (uint32_t)(GPIO_P10 | GPIO_P11 | GPIO_P12 | GPIO_P13 | \
                            GPIO_P14 | GPIO_P15 | GPIO_P16 | GPIO_P17)      /*!< P10 - P17 */
               
#define GPIO_P2  (uint32_t)(GPIO_P20 | GPIO_P21 | GPIO_P22 | GPIO_P23 | \
                            GPIO_P24 | GPIO_P25 | GPIO_P26 | GPIO_P27)      /*!< P20 - P17 */
                
#define GPIO_P3  (uint32_t)(GPIO_P30 | GPIO_P31 | GPIO_P32 | GPIO_P33 | \
                            GPIO_P34 | GPIO_P35 | GPIO_P36           )      /*!< P30 - P36 */

#define GPIO_PIN_ALL \
                 (uint32_t)(GPIO_P0  | GPIO_P1  | GPIO_P2  | GPIO_P3)       /*!< All Pins */

#define GPIO_MAX_NUM 31

#define GPIO_PIN_MAX GPIO_P36

///GPIO states (low/high)
enum gpio_level
{
    GPIO_LOW = 0,                    /*!< Set GPIO to low level */
    GPIO_HIGH = (int)0xFFFFFFFF,     /*!< Set GPIO to high level */
};

///GPIO direction (input/output)
enum gpio_direction
{
    GPIO_INPUT = 0,                  /*!< Set GPIO direction to input */
    GPIO_OUTPUT = (int)0xFFFFFFFF,   /*!< Set GPIO direction to output */
};

///GPIO pull states (low/high)
enum gpio_pull
{
    GPIO_HIGH_Z,                     /*!< Set GPIO as high impedance mode */
    GPIO_PULL_DOWN,                  /*!< Set GPIO as pull-down mode */
    GPIO_PULL_UP,                    /*!< Set GPIO as pull-up mode */
    GPIO_PULL_RSVD                   /*!< Reserved */
};

///GPIO interrupt triger type (falling edge/rising edge/low level/high level)
enum gpio_int_trig_type
{
    GPIO_INT_FALLING_EDGE,           /*!< Set GPIO interrupt enabled by falling edge */
    GPIO_INT_RISING_EDGE,            /*!< Set GPIO interrupt enabled by rising edge */
    GPIO_INT_LOW_LEVEL,              /*!< Set GPIO interrupt enabled by low level */
    GPIO_INT_HIGH_LEVEL              /*!< Set GPIO interrupt enabled by high level */
};

///GPIO wakeup type
enum gpio_wakeup_type
{
    GPIO_WKUP_BY_HIGH,               /*!< Set GPIO wakeup by high level */
    GPIO_WKUP_BY_LOW,                /*!< Set GPIO wakeup by low level */
    GPIO_WKUP_BY_CHANGE              /*!< Set GPIO wakeup by level change */
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Callback function pointer type for level detection
typedef void (*gpio_callback_t)(enum gpio_pin pin);


/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
  
/**
 ****************************************************************************************
 * @brief   Enable GPIO module clock
 * @description
 *  This function is used to enable GPIO module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void gpio_clock_on(void)
{
    // enable GPIO module clock
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GATING_GPIO);
}

/**
 ****************************************************************************************
 * @brief   Disable GPIO module clock
 * @description
 *  This function is used to disable GPIO module clock
 *
 *****************************************************************************************
 */
__STATIC_INLINE void gpio_clock_off(void)
{
    // disable GPIO module clock
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GATING_GPIO);
}

/**
 ****************************************************************************************
 * @brief   Reset GPIO module
 * @description
 *  This function is used to reset GPIO module
 *
 *****************************************************************************************
 */
__STATIC_INLINE void gpio_reset(void)
{
    // Reset GPIO module
    syscon_SetCRSS(QN_SYSCON, SYSCON_MASK_GPIO_RST);
    syscon_SetCRSC(QN_SYSCON, SYSCON_MASK_GPIO_RST);
}

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */


#if CONFIG_GPIO_DEFAULT_IRQHANDLER==TRUE
void GPIO_IRQHandler(void);
#endif

extern void gpio_init(gpio_callback_t p_callback);
extern enum gpio_level gpio_read_pin(enum gpio_pin pin);
extern void gpio_write_pin(enum gpio_pin pin, enum gpio_level level);
extern void gpio_set_direction(enum gpio_pin pin, enum gpio_direction direction);

extern uint32_t gpio_read_pin_field(uint32_t pin_mask);
extern void gpio_write_pin_field(uint32_t pin_mask, uint32_t level_value);
extern void gpio_set_direction_field(uint32_t pin_mask, uint32_t direction_value);

extern void gpio_toggle_pin(enum gpio_pin pin);
extern void gpio_set_interrupt(enum gpio_pin pin, enum gpio_int_trig_type trig_type);
extern void gpio_enable_interrupt(enum gpio_pin pin);
extern void gpio_disable_interrupt(enum gpio_pin pin);
extern void gpio_pull_set(enum gpio_pin pin, enum gpio_pull pull_state);
extern void gpio_wakeup_config(enum gpio_pin pin, enum gpio_wakeup_type type);
extern bool gpio_sleep_allowed(void);


/// @} GPIO
#endif /* CONFIG_ENABLE_DRIVER_GPIO */
#endif /* end _GPIO_H_ */
