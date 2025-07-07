//
// Created by Jonathan Tielve
//

#include "tielve_stm32h743_gpio.h"
#include "tielve_stm32h743_systick.h"

#define NULL ((void *)0)

static gpio_status_t get_port_clock_bit(GPIO_TypeDef* port, uint32_t* clock_bit);

/**
 * @brief Get the RCC clock enable bit for a GPIO port
 * @param port GPIO port
 * @param clock_bit Pointer to store the clock enable bit
 * @return GPIO status code
 */
static gpio_status_t get_port_clock_bit(GPIO_TypeDef* port, uint32_t* clock_bit)
{
    if (port == GPIOA) {
        *clock_bit = RCC_AHB4ENR_GPIOAEN;
    } else if (port == GPIOB) {
        *clock_bit = RCC_AHB4ENR_GPIOBEN;
    } else if (port == GPIOC) {
        *clock_bit = RCC_AHB4ENR_GPIOCEN;
    } else if (port == GPIOD) {
        *clock_bit = RCC_AHB4ENR_GPIODEN;
    } else if (port == GPIOE) {
        *clock_bit = RCC_AHB4ENR_GPIOEEN;
    } else if (port == GPIOF) {
        *clock_bit = RCC_AHB4ENR_GPIOFEN;
    } else if (port == GPIOG) {
        *clock_bit = RCC_AHB4ENR_GPIOGEN;
    } else if (port == GPIOH) {
        *clock_bit = RCC_AHB4ENR_GPIOHEN;
    } else if (port == GPIOI) {
        *clock_bit = RCC_AHB4ENR_GPIOIEN;
    } else if (port == GPIOJ) {
        *clock_bit = RCC_AHB4ENR_GPIOJEN;
    } else if (port == GPIOK) {
        *clock_bit = RCC_AHB4ENR_GPIOKEN;
    } else {
        return GPIO_ERROR_INVALID_PORT;
    }

    return GPIO_OK;
}

gpio_status_t gpio_enable_clock(GPIO_TypeDef* port) {
    if (!GPIO_IS_VALID_PORT(port)) {
        return GPIO_ERROR_INVALID_PORT;
    }


    gpio_status_t status = get_port_clock_bit(port, &clock_bit);


    // Enable clock
    RCC_AHB4ENR |= clock_bit;

    // Small delay to ensure clock is stable
    for (volatile int i = 0; i < 100; i++) {
        __asm("nop");
    }

    return GPIO_OK;
}

gpio_status_t gpio_config_pin(GPIO_TypeDef* port, uint8_t pin, const gpio_config_t* config) {
    // Parameter validation
    if (!GPIO_IS_VALID_PORT(port)) {
        return GPIO_ERROR_INVALID_PORT;
    }
    if (!GPIO_IS_VALID_PIN(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    if (config == NULL) {
        return GPIO_ERROR_INVALID_PARAM;
    }
    if (config->mode > GPIO_MODE_ANALOG) {
        return GPIO_ERROR_INVALID_PARAM;
    }
    if (config->alternate_function > 15) {
        return GPIO_ERROR_INVALID_PARAM;
    }

    // Check if clock is enabled
    uint32_t clock_bit;
    gpio_status_t status = get_port_clock_bit(port, &clock_bit);
    if (status != GPIO_OK) {
        return status;
    }
    if (!(RCC_AHB4ENR & clock_bit)) {
        return GPIO_ERROR_CLOCK_NOT_ENABLED;
    }

    // Configure mode
    uint32_t mode_reg = port->MODER;
    mode_reg &= ~(0x3UL << (pin * 2));  // Clear existing mode
    mode_reg |= ((uint32_t)config->mode << (pin * 2));  // Set new mode
    port->MODER = mode_reg;

    // Configure output type
    if (config->mode == GPIO_MODE_OUTPUT || config->mode == GPIO_MODE_ALTERNATE) {
        if (config->output_type == GPIO_OUTPUT_OPEN_DRAIN) {
            port->OTYPER |= (1UL << pin);
        } else {
            port->OTYPER &= ~(1UL << pin);
        }
    }

    // Configure speed
    if (config->mode == GPIO_MODE_OUTPUT || config->mode == GPIO_MODE_ALTERNATE) {
        uint32_t speed_reg = port->OSPEEDR;
        speed_reg &= ~(0x3UL << (pin * 2));  // Clear existing speed
        speed_reg |= ((uint32_t)config->speed << (pin * 2));  // Set new speed
        port->OSPEEDR = speed_reg;
    }

    // Configure pull-up/pull-down
    uint32_t pupd_reg = port->PUPDR;
    pupd_reg &= ~(0x3UL << (pin * 2));  // Clear existing pull configuration
    pupd_reg |= ((uint32_t)config->pull << (pin * 2));  // Set new pull configuration
    port->PUPDR = pupd_reg;

    // Configure alternate function
    if (config->mode == GPIO_MODE_ALTERNATE) {
        if (pin < 8) {
            // Use AFRL for pins 0-7
            uint32_t afr_reg = port->AFRL;
            afr_reg &= ~(0xFUL << (pin * 4));  // Clear existing AF
            afr_reg |= ((uint32_t)config->alternate_function << (pin * 4));  // Set new AF
            port->AFRL = afr_reg;
        } else {
            // Use AFRH for pins 8-15
            uint32_t pin_offset = pin - 8;
            uint32_t afr_reg = port->AFRH;
            afr_reg &= ~(0xFUL << (pin_offset * 4));  // Clear existing AF
            afr_reg |= ((uint32_t)config->alternate_function << (pin_offset * 4));  // Set new AF
            port->AFRH = afr_reg;
        }
    }

    return GPIO_OK;
}


void gpio_write_pin(GPIO_TypeDef* port, uint8_t pin, bool state)
{
    if (!GPIO_IS_VALID_PORT(port) || !GPIO_IS_VALID_PIN(pin)) {
        return;
    }

    // Use BSRR register for atomic set/reset
    if (state) {
        port->BSRR = (1UL << pin);          // Set bit
    } else {
        port->BSRR = (1UL << (pin + 16));   // Reset bit
    }
}

bool gpio_read_pin(GPIO_TypeDef* port, uint8_t pin)
{
    if (!GPIO_IS_VALID_PORT(port) || !GPIO_IS_VALID_PIN(pin)) {
        return false;
    }

    return ((port->IDR >> pin) & 0x1) != 0;
}

void gpio_toggle_pin(GPIO_TypeDef* port, uint8_t pin)
{
    if (!GPIO_IS_VALID_PORT(port) || !GPIO_IS_VALID_PIN(pin)) {
        return;
    }

    // Read current state and write opposite
    bool current_state = gpio_read_pin(port, pin);
    gpio_write_pin(port, pin, !current_state);
}

void gpio_write_port(GPIO_TypeDef* port, uint16_t value)
{
    if (!GPIO_IS_VALID_PORT(port)) {
        return;
    }

    // Write to ODR register
    port->ODR = (uint32_t)value;
}

uint16_t gpio_read_port(GPIO_TypeDef* port)
{
    if (!GPIO_IS_VALID_PORT(port)) {
        return 0;
    }

    // Read from IDR register
    return (uint16_t)(port->IDR & 0xFFFF);
}


gpio_status_t gpio_set_alternate_function(GPIO_TypeDef* port, uint8_t pin, uint8_t af_number)
{
    // Error handling
    if (!GPIO_IS_VALID_PORT(port)) {
        return GPIO_ERROR_INVALID_PORT;
    }
    if (!GPIO_IS_VALID_PIN(pin)) {
        return GPIO_ERROR_INVALID_PIN;
    }
    if (af_number > 15) {
        return GPIO_ERROR_INVALID_PARAM;
    }

    // Check if clock is enabled
    uint32_t clock_bit;
    gpio_status_t status = get_port_clock_bit(port, &clock_bit);
    if (status != GPIO_OK) {
        return status;
    }
    if (!(RCC_AHB4ENR & clock_bit)) {
        return GPIO_ERROR_CLOCK_NOT_ENABLED;
    }

    // Configure AFR
    if (pin < 8) {
        // Use AFRL for pins 0-7
        uint32_t afr_reg = port->AFRL;
        afr_reg &= ~(0xFUL << (pin * 4));  // Clear existing AF
        afr_reg |= ((uint32_t)af_number << (pin * 4));  // Set new AF
        port->AFRL = afr_reg;
    } else {
        // Use AFRH for pins 8-15
        uint32_t pin_offset = pin - 8;
        uint32_t afr_reg = port->AFRH;
        afr_reg &= ~(0xFUL << (pin_offset * 4));  // Clear existing AF
        afr_reg |= ((uint32_t)af_number << (pin_offset * 4));  // Set new AF
        port->AFRH = afr_reg;
    }

    return GPIO_OK;
}

gpio_status_t gpio_get_default_config(gpio_config_t* config)
{
    if (config == NULL) {
        return GPIO_ERROR_INVALID_PARAM;
    }

    config->mode = GPIO_MODE_INPUT;
    config->output_type = GPIO_OUTPUT_PUSH_PULL;
    config->speed = GPIO_SPEED_LOW;
    config->pull = GPIO_PULL_NONE;
    config->alternate_function = 0;

    return GPIO_OK;
}

void led_init() {
    gpio_config_t config;
    gpio_enable_clock(GPIOH);
    gpio_get_default_config(&config);
    config.mode = GPIO_MODE_OUTPUT;
    gpio_config_pin(GPIOH,LED_PIN, &config);
    gpio_toggle_pin(GPIOH, LED_PIN);
}

void blink_led(uint32_t count, uint32_t delay_ms_val) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_write_pin(LED_PORT, LED_PIN, true);
        delay_ms(delay_ms_val);
        gpio_write_pin(LED_PORT, LED_PIN, false);
        delay_ms(delay_ms_val);
    }
}

void spi_error_led(spi_status_t status) {
    if (status == SPI_OK) {
        blink_led(2, 500);
    }
    if (status == SPI_ERROR_INVALID_HANDLE) {
        blink_led(3, 500);
    }
    if (status == SPI_ERROR_INVALID_PARAM) {
        blink_led(4, 500);
    }
    if (status == SPI_ERROR_NOT_INITIALIZED) {
        blink_led(5, 500);
    }
    if (status == SPI_ERROR_ALREADY_INITIALIZED) {
        blink_led(6, 500);
    }
    if (status == SPI_ERROR_BUSY) {
        blink_led(7, 500);
    }
    if (status == SPI_ERROR_TIMEOUT) {
        blink_led(8, 500);
    }
    if (status == SPI_ERROR_HARDWARE_FAULT) {
        blink_led(9, 500);
    }
    if (status == SPI_ERROR_UNSUPPORTED_INSTANCE) {
        blink_led(10, 500);
    }
    if (status == SPI_ERROR_NO_AVAILABLE_HANDLES) {
        blink_led(11, 500);
    }
}