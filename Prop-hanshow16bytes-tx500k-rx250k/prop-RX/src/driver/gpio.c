/**
 ****************************************************************************************
 *
 * @file gpio.c
 *
 * @brief GPIO driver for QN9020.
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
 * @addtogroup GPIO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "gpio.h"
#if CONFIG_ENABLE_DRIVER_GPIO==TRUE
#include "sleep.h"
/*
 * STRUCT DEFINITIONS
 ****************************************************************************************
 */

/// GPIO environment structure
struct gpio_env_tag
{
    /// Callback function pointer for level detection
    gpio_callback_t callback;
};

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

#if GPIO_CALLBACK_EN==TRUE
/// GPIO environment variable
static struct gpio_env_tag  gpio_env = {NULL};
#endif

/// @cond


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/// @endcond


__STATIC_INLINE int ctz (uint32_t x)
{
    int n = 0;
    if (x == 0) return 32;
    if ((x & 0x0000FFFF) == 0) { n += 16; x >>= 16; }
    if ((x & 0x000000FF) == 0) { n +=  8; x >>=  8; }
    if ((x & 0x0000000F) == 0) { n +=  4; x >>=  4; }
    if ((x & 0x00000003) == 0) { n +=  2; x >>=  2; }
    if ((x & 0x00000001) == 0) { n +=  1; }
    return n;
}

/**
 ****************************************************************************************
 * @brief Handles GPIO interrupt, polling and process
 * @description
 *  Get interrupt pin by polling interrupt status register, and then execute the callback function
 *  if it was enabled.
 ****************************************************************************************
 */
#if CONFIG_GPIO_DEFAULT_IRQHANDLER==TRUE
void GPIO_IRQHandler(void)
{
#if QN_32K_LOW_POWER_MODE_EN==TRUE
    exit_low_power_mode();
#endif
    if (ahb_clock_flag & 0x01) {
        syscon_SetCMDCRWithMask(QN_SYSCON, SYSCON_MASK_AHB_DIV_BYPASS|SYSCON_MASK_AHB_DIVIDER, ahb_clock_flag);
        ahb_clock_flag &= ~0x01;
    }

    int pin;
    const uint32_t reg = gpio_gpio_IntStatus(QN_GPIO);

    // Parse all lines
    for(pin = (int)GPIO_PIN_MAX; pin != 0; pin >>= 1)
    {
        // Check if int staus has set
        if (reg & pin)
        {
            gpio_gpio_IntClear(QN_GPIO, pin);

#if GPIO_CALLBACK_EN==TRUE
            // Callback handler
            if(gpio_env.callback)
                gpio_env.callback((enum gpio_pin) pin);
#endif

        }
    }
}
#endif /* CONFIG_GPIO_DEFAULT_IRQHANDLER==TRUE */


/**
 ****************************************************************************************
 * @brief Initialize and configure the GPIO.
 * @param[in]  p_callback   Callback function pointer, which is called in IRQHandler.
 * @description
 * This function is used to initialize callback function pointer and enable GPIO NVIC IRQ.
 ****************************************************************************************
 */
void gpio_init(gpio_callback_t p_callback)
{
#if GPIO_CALLBACK_EN==TRUE
    // Initialize environment
    gpio_env.callback = p_callback;
#endif

    /* Enable AHB clock to the GPIO domain. */
    gpio_clock_on();

    /* Set up NVIC when I/O pins are configured as external interrupts. */
#if CONFIG_GPIO_ENABLE_INTERRUPT==TRUE
    NVIC_EnableIRQ(GPIO_IRQn);
#endif
}

/**
 ****************************************************************************************
 * @brief Read GPIO pin level.
 * @param[in]  pin     Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @return  The level of specified pin value: GPIO_LOW / GPIO_HIGH
 * @description
 * This function is used to get a specified GPIO pin's level..
 ****************************************************************************************
 */
enum gpio_level gpio_read_pin(enum gpio_pin pin)
{
    const uint32_t reg = gpio_gpio_GetInputData(QN_GPIO);

    return (reg & pin) ? GPIO_HIGH : GPIO_LOW;
}

/**
 ****************************************************************************************
 * @brief Write on GPIO pin.
 * @param[in]  pin     Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @param[in]  level   Level: GPIO_LOW or GPIO_HIGH
 * @description
 * This function is used to set level high(1) or low(0) to a specified GPIO pin.
 ****************************************************************************************
 */
void gpio_write_pin(enum gpio_pin pin, enum gpio_level level)
{
    uint32_t reg = gpio_gpio_GetOutputData(QN_GPIO);

    // Set or clear corresponding bit in register
    reg = (level == GPIO_HIGH) ? (reg | pin) : (reg & ~pin);

    // Write register value
    gpio_gpio_SetOutputData(QN_GPIO, reg);
}

/**
 ****************************************************************************************
 * @brief Set direction (input or output) of a set of GPIO pins.
 * @param[in]  pin        Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @param[in]  direction  Value: GPIO_INPUT / GPIO_OUTPUT
 * @description
 * It writes on direction register without impacting unselected GPIO pins.
 ****************************************************************************************
 */
void gpio_set_direction(enum gpio_pin pin, enum gpio_direction direction)
{
    if(direction == GPIO_INPUT)
    {
        gpio_gpio_ClrOutEnable(QN_GPIO, pin);
    }
    else /*if(direction == GPIO_OUTPUT)*/
    {
        gpio_gpio_SetOutEnable(QN_GPIO, pin);
    }
}

/**
 ****************************************************************************************
 * @brief Read a set of GPIO pins.
 * @param[in]  pin_mask     Pin mask of GPIO specify which pins to read
 * @return Masked GPIO DATA register value
 * @description
 * It reads from input register without unselected GPIO pins value.
 ****************************************************************************************
 */
uint32_t gpio_read_pin_field(uint32_t pin_mask)
{
    // Read IN register value
    const uint32_t reg = gpio_gpio_GetInputData(QN_GPIO);

    return reg & pin_mask;
}

/**
 ****************************************************************************************
 * @brief Write a set of GPIO pins.
 * @param[in]  pin_mask       Pin mask of GPIO specify which pins to set
 * @param[in]  level_value    Mask bit value to set: 1:high level; 0:low level.
 * @description
 * It writes on output register without impacting unselected GPIO pins.
 ****************************************************************************************
 */
void gpio_write_pin_field(uint32_t pin_mask, uint32_t level_value)
{
    // Read register value
    uint32_t reg = gpio_gpio_GetOutputData(QN_GPIO);

    level_value &= pin_mask;
    reg &= ~pin_mask;
    reg |= level_value;

    // Write register value
    gpio_gpio_SetOutputData(QN_GPIO, reg);
}

/**
 ****************************************************************************************
 * @brief Set direction (input or output) of a set of GPIO pins.
 * @param[in]  pin_mask         Pin mask of GPIO specify which pins to set
 * @param[in]  direction_value  Value: GPIO_INPUT / GPIO_OUTPUT
 * @description
 * It writes on direction register without impacting unselected GPIO pins.
 ****************************************************************************************
 */
void gpio_set_direction_field(uint32_t pin_mask, uint32_t direction_value)
{
    direction_value &= pin_mask;

    gpio_gpio_SetOutEnable(QN_GPIO, direction_value);
    direction_value ^= pin_mask;
    gpio_gpio_ClrOutEnable(QN_GPIO, direction_value);
}

/**
 ****************************************************************************************
 * @brief Toggle a GPIO pin.
 * @param[in]  pin     Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @description
 * This function is used to set a specified GPIO pin to the opposite level that is currently appied..
 ****************************************************************************************
 */
void gpio_toggle_pin(enum gpio_pin pin)
{
    // Read register value
    uint32_t reg = gpio_gpio_GetOutputData(QN_GPIO);

    // Set or clear corresponding bit in register
    reg ^= pin;

    // Write register value
    gpio_gpio_SetOutputData(QN_GPIO, reg);

}

/**
 ****************************************************************************************
 * @brief Set interrupt edge/level, sinlge/double, polarity.
 * @param[in]  pin          Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @param[in]  trig_type    4 types: high/low level, rising/falling edge
 * @description
 * This function is used to configure a specified GPIO pin's interrupt.
 *
 ****************************************************************************************
 */
void gpio_set_interrupt(enum gpio_pin pin, enum gpio_int_trig_type trig_type)
{
    switch(trig_type)
    {
        case GPIO_INT_FALLING_EDGE:
            gpio_gpio_SetIntFallingEdge(QN_GPIO, pin);
            break;
        case GPIO_INT_RISING_EDGE:
            gpio_gpio_SetIntRisingEdge(QN_GPIO, pin);
            break;
        case GPIO_INT_LOW_LEVEL:
            gpio_gpio_SetIntLowLevel(QN_GPIO, pin);
            break;
        case GPIO_INT_HIGH_LEVEL:
            gpio_gpio_SetIntHighLevel(QN_GPIO, pin);
            break;
        default:
            break;
    }
}

/**
 ****************************************************************************************
 * @brief Enable interrupt on GPIO pin.
 * @param[in]  pin          Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @description
 * This function is used to enable a specified GPIO pin's interrupt.
 *
 ****************************************************************************************
 */
void gpio_enable_interrupt(enum gpio_pin pin)
{
    gpio_gpio_SetIntEnable(QN_GPIO, pin);
}

/**
 ****************************************************************************************
 * @brief Disable interrupt on GPIO pin.
 * @param[in]  pin     Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @description
 * This function is used to disable a specified GPIO pin's interrupt.
 *
 ****************************************************************************************
 */
void gpio_disable_interrupt(enum gpio_pin pin)
{
    gpio_gpio_ClrIntEnable(QN_GPIO, pin);
}

/**
 ****************************************************************************************
 * @brief Set GPIO pin to specified mode.
 * @param[in]  pin            Specify pin of GPIO: GPIO_P00~GPIO_P07 GPIO_P10~GPIO_P17 GPIO_P20~GPIO_P27 GPIO_P30~GPIO_P36
 * @param[in]  pull_state     Pin mode: 00 : High-Z, 01 : Pull-down, 10 : Pull-up, 11 : Reserved
 * @description
 * This function is used to set a specified pin mode to a specified GPIO pin.
 *
 ****************************************************************************************
 */
void gpio_pull_set(enum gpio_pin pin, enum gpio_pull pull_state)
{
    const int gpio_index = ctz(pin);

    if (gpio_index < 16) 
    {
        syscon_SetPPCR0WithMask(QN_SYSCON, GPIO_PULL_RSVD << gpio_index*2, 
                                               pull_state << gpio_index*2);
    }
    else 
    {
        syscon_SetPPCR1WithMask(QN_SYSCON, GPIO_PULL_RSVD << ((gpio_index-16)*2), 
                                               pull_state << ((gpio_index-16)*2));
    }
}

/**
 ****************************************************************************************
 * @brief  configure GPIO wakeup
 * @param[in]    pin     Wakeup pin: P0, P1
 * @param[in]    type    Wakeup type: high, low, change
 * @description
 *  This function is used to configure GPIO wakeup pin.
 *****************************************************************************************
 */
void gpio_wakeup_config(enum gpio_pin pin, enum gpio_wakeup_type type)
{
    uint32_t reg, mask, value, iowake;

    // set gpio pin input
    gpio_set_direction(pin, GPIO_INPUT);

    mask = (pin & 0x0000FFFF);
    value = gpio_gpio_GetInputData(QN_GPIO) & mask;
    iowake = syscon_GetIOWCR(QN_SYSCON) & (~((mask << 16) | mask));
    
    if (type == GPIO_WKUP_BY_HIGH) {
        reg = mask | iowake;
        gpio_gpio_SetIntRisingEdge(QN_GPIO, mask);     // high wakeup, rising interrupt
        //gpio_gpio_SetIntHighLevel(QN_GPIO, mask);
    }
    else if (type == GPIO_WKUP_BY_LOW) {
        reg = mask | (iowake | (mask << 16));
        gpio_gpio_SetIntFallingEdge(QN_GPIO, mask);    // low wakeup, falling interrupt
        //gpio_gpio_SetIntLowLevel(QN_GPIO, mask);
    }
    else { // set gpio pin level change as deep sleep wakeup source
        reg = mask | (iowake | (value << 16));
        gpio_gpio_SetIntFallingEdge(QN_GPIO, value&mask);
        gpio_gpio_SetIntRisingEdge(QN_GPIO, value^mask);
    }
    syscon_SetIOWCR(QN_SYSCON, reg);
}

/**
 ****************************************************************************************
 * @brief  Check gpio sleep is allowed or not
 * @return  TRUE or FALSE
 * @description
 *  This function is used to check the gpio sleep is allowed or not.
 *****************************************************************************************
 */
bool gpio_sleep_allowed(void)
{
    uint32_t iowake;
    uint32_t level;

    iowake = syscon_GetIOWCR(QN_SYSCON);
    level = gpio_read_pin_field(iowake & 0xFFFF);

    if (((iowake >> 16) ^ level) & iowake) {
        return FALSE;
    }
    
    return TRUE;
}

void gpio_open_drain_out(enum gpio_pin pin, enum gpio_level level)
{
    if (level == GPIO_LOW) {
        gpio_set_direction(pin, GPIO_OUTPUT);
        gpio_write_pin(pin, GPIO_LOW);
    }
    else { // pull-up by external resistor
        gpio_set_direction(pin, GPIO_INPUT);
        gpio_pull_set(pin, GPIO_HIGH_Z);
    }
}

#endif /* CONFIG_ENABLE_DRIVER_GPIO */
/// @} GPIO
