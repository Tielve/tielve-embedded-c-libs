//
// Created by Jonathan Tielve
//

#ifndef TIELVE_STM32H743_GPIO_H
#define TIELVE_STM32H743_GPIO_H

#include <stdint.h>
#include <stdbool.h>
#include "tielve_stm32h743_spi.h"
#include "tielve_stm32h743_rcc.h"

/**
 * @brief GPIO Hardware Register Structure
 *
 * Maps directly to STM32H743 GPIO peripheral registers.
 * Each GPIO port (A, B, C, etc.) has this register layout.
 */
typedef struct {
    volatile uint32_t MODER;    ///< 0x00: Mode register
    volatile uint32_t OTYPER;   ///< 0x04: Output type register
    volatile uint32_t OSPEEDR;  ///< 0x08: Output speed register
    volatile uint32_t PUPDR;    ///< 0x0C: Pull-up/pull-down register
    volatile uint32_t IDR;      ///< 0x10: Input data register
    volatile uint32_t ODR;      ///< 0x14: Output data register
    volatile uint32_t BSRR;     ///< 0x18: Bit set/reset register
    volatile uint32_t LCKR;     ///< 0x1C: Configuration lock register
    volatile uint32_t AFRL;     ///< 0x20: Alternate function register (pins 0-7)
    volatile uint32_t AFRH;     ///< 0x24: Alternate function register (pins 8-15)
} GPIO_TypeDef;

#define GPIOA_BASE  0x58020000UL
#define GPIOB_BASE  0x58020400UL
#define GPIOC_BASE  0x58020800UL
#define GPIOD_BASE  0x58020C00UL
#define GPIOE_BASE  0x58021000UL
#define GPIOF_BASE  0x58021400UL
#define GPIOG_BASE  0x58021800UL
#define GPIOH_BASE  0x58021C00UL
#define GPIOI_BASE  0x58022000UL
#define GPIOJ_BASE  0x58022400UL
#define GPIOK_BASE  0x58022800UL

#define GPIOA ((GPIO_TypeDef*)GPIOA_BASE)
#define GPIOB ((GPIO_TypeDef*)GPIOB_BASE)
#define GPIOC ((GPIO_TypeDef*)GPIOC_BASE)
#define GPIOD ((GPIO_TypeDef*)GPIOD_BASE)
#define GPIOE ((GPIO_TypeDef*)GPIOE_BASE)
#define GPIOF ((GPIO_TypeDef*)GPIOF_BASE)
#define GPIOG ((GPIO_TypeDef*)GPIOG_BASE)
#define GPIOH ((GPIO_TypeDef*)GPIOH_BASE)
#define GPIOI ((GPIO_TypeDef*)GPIOI_BASE)
#define GPIOJ ((GPIO_TypeDef*)GPIOJ_BASE)
#define GPIOK ((GPIO_TypeDef*)GPIOK_BASE)

#define GPIO_PIN_0  0
#define GPIO_PIN_1  1
#define GPIO_PIN_2  2
#define GPIO_PIN_3  3
#define GPIO_PIN_4  4
#define GPIO_PIN_5  5
#define GPIO_PIN_6  6
#define GPIO_PIN_7  7
#define GPIO_PIN_8  8
#define GPIO_PIN_9  9
#define GPIO_PIN_10 10
#define GPIO_PIN_11 11
#define GPIO_PIN_12 12
#define GPIO_PIN_13 13
#define GPIO_PIN_14 14
#define GPIO_PIN_15 15

#define LED_PORT        GPIOH
#define LED_PIN         GPIO_PIN_7

/**
 * @brief GPIO Pin Modes
 */
typedef enum {
    GPIO_MODE_INPUT     = 0x00,  ///< Input mode 00
    GPIO_MODE_OUTPUT    = 0x01,  ///< Output mode 01
    GPIO_MODE_ALTERNATE = 0x02,  ///< Alternate function mode 10
    GPIO_MODE_ANALOG    = 0x03   ///< Analog mode 11
} gpio_mode_t;

/**
 * @brief GPIO Output Types
 */
typedef enum {
    GPIO_OUTPUT_PUSH_PULL  = 0x00,  ///< Push-pull output 00
    GPIO_OUTPUT_OPEN_DRAIN = 0x01   ///< Open-drain output 01
} gpio_output_type_t;

/**
 * @brief GPIO Output Speed
 */
typedef enum {
    GPIO_SPEED_LOW       = 0x00,  ///< Low speed 00
    GPIO_SPEED_MEDIUM    = 0x01,  ///< Medium speed 01
    GPIO_SPEED_HIGH      = 0x02,  ///< High speed 10
    GPIO_SPEED_VERY_HIGH = 0x03   ///< Very high speed 11
} gpio_speed_t;

/**
 * @brief GPIO Pull-up/Pull-down Configuration
 */
typedef enum {
    GPIO_PULL_NONE = 0x00,  ///< No pull-up/pull-down 00
    GPIO_PULL_UP   = 0x01,  ///< Pull-up enabled 01
    GPIO_PULL_DOWN = 0x02   ///< Pull-down enabled 10
} gpio_pull_t;

/**
 * @brief GPIO Pin Configuration Structure
 */
typedef struct {
    gpio_mode_t mode;                ///< Pin mode
    gpio_output_type_t output_type;  ///< Output type (only for output mode)
    gpio_speed_t speed;              ///< Output speed (only for output mode)
    gpio_pull_t pull;                ///< Pull-up/pull-down configuration
    uint8_t alternate_function;      ///< Alternate function (0-15, only for AF mode)
} gpio_config_t;

/**
 * @brief GPIO Status Codes
 */
typedef enum {
    GPIO_OK = 0,                     ///< Operation successful
    GPIO_ERROR_INVALID_PARAM,        ///< Invalid parameter
    GPIO_ERROR_INVALID_PORT,         ///< Invalid GPIO port
    GPIO_ERROR_INVALID_PIN,          ///< Invalid pin number
    GPIO_ERROR_CLOCK_NOT_ENABLED     ///< GPIO clock not enabled
} gpio_status_t;

#define GPIO_IS_VALID_PIN(pin) ((pin) <= 15)

#define GPIO_IS_VALID_PORT(port) \
((port) == GPIOA || (port) == GPIOB || (port) == GPIOC || (port) == GPIOD || \
(port) == GPIOE || (port) == GPIOF || (port) == GPIOG || (port) == GPIOH || \
(port) == GPIOI || (port) == GPIOJ || (port) == GPIOK)

/**
 * @brief Initialize GPIO clock for a port
 * @param port GPIO port to enable clock for
 * @return GPIO status code
 */
gpio_status_t gpio_enable_clock(GPIO_TypeDef* port);

/**
 * @brief Configure a GPIO pin
 * @param port GPIO port
 * @param pin Pin number (0-15)
 * @param config Pin configuration
 * @return GPIO status code
 */
gpio_status_t gpio_config_pin(GPIO_TypeDef* port, uint8_t pin, const gpio_config_t* config);

/**
 * @brief Write to a GPIO pin
 * @param port GPIO port
 * @param pin Pin number (0-15)
 * @param state Pin state (true = high, false = low)
 */
void gpio_write_pin(GPIO_TypeDef* port, uint8_t pin, bool state);

/**
 * @brief Read from a GPIO pin
 * @param port GPIO port
 * @param pin Pin number (0-15)
 * @return Pin state (true = high, false = low)
 */
bool gpio_read_pin(GPIO_TypeDef* port, uint8_t pin);

/**
 * @brief Toggle a GPIO pin
 * @param port GPIO port
 * @param pin Pin number (0-15)
 */
void gpio_toggle_pin(GPIO_TypeDef* port, uint8_t pin);

/**
 * @brief Write to entire GPIO port
 * @param port GPIO port
 * @param value 16-bit value to write to port
 */
void gpio_write_port(GPIO_TypeDef* port, uint16_t value);

/**
 * @brief Read entire GPIO port
 * @param port GPIO port
 * @return 16-bit port value
 */
uint16_t gpio_read_port(GPIO_TypeDef* port);

/**
 * @brief Set alternate function for a pin
 * @param port GPIO port
 * @param pin Pin number (0-15)
 * @param af_number Alternate function number (0-15)
 * @return GPIO status code
 */
gpio_status_t gpio_set_alternate_function(GPIO_TypeDef* port, uint8_t pin, uint8_t af_number);

/**
 * @brief Get default GPIO configuration
 * @param config Pointer to configuration structure to fill
 * @return GPIO status code
 */
gpio_status_t gpio_get_default_config(gpio_config_t* config);


/**
 * @brief Initialize on board LED
 */
void led_init();

/**
 * @brief Blink LED
 * @param count Number of times to blink LED
 * @param delay_ms_val delay in ms between blinks
 */
void blink_led(uint32_t count, uint32_t delay_ms_val);

/**
 * @brief Blink a number of times equivalent to SPI Error codes
 * @param status status code of SPI
 */
void spi_error_led(spi_status_t status);

#endif //TIELVE_STM32H743_GPIO_H
