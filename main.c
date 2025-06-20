//
// Created by jonathan on 6/18/25.
//

/**
 * @file gpio_test.c
 * @brief GPIO Driver Test for FK743M2-IIT6 Board
 * @author Jonathan Tielve
 *
 * This test file validates the GPIO driver functionality on the FK743M2-IIT6 board.
 * You may need to adjust pin assignments based on your specific board layout.
 */

#include "drivers/tielve_stm32h743_gpio.h"
#include "drivers/tielve_stm32h743_systick.h"
#include "drivers/tielve_stm32h743_spi.h"

// Common pin assignments for STM32H743 development boards
// Adjust these based on your actual board pinout
#define LED_PORT        GPIOH
#define LED_PIN         GPIO_PIN_7    // Common user LED pin

#define SPI_PORT   GPIOA
#define SCK   GPIO_PIN_5
#define MOSI  GPIO_PIN_7
#define MISO  GPIO_PIN_6

#define RCC_APB2RSTR (*(volatile uint32_t*)(RCC_BASE + 0x0FC))
#define RCC_APB2RSTR_SPI1RST (1UL << 12)

static void blink_led(uint32_t count, uint32_t delay_ms_val) {
    for (uint32_t i = 0; i < count; i++) {
        gpio_write_pin(LED_PORT, LED_PIN, true);
        delay_ms(delay_ms_val);
        gpio_write_pin(LED_PORT, LED_PIN, false);
        delay_ms(delay_ms_val);
    }
}

void spi_force_reset(void) {
    // 1. Disable SPI completely
    SPI1->CR1 = 0;
    SPI1->CR2 = 0;

    // 2. Clear ALL interrupt flags aggressively
    SPI1->IFCR = 0xFFFFFFFF;

    // 3. Reset via RCC (this should force idle state)
    RCC_APB2RSTR |= RCC_APB2RSTR_SPI1RST;
    delay_ms(1);  // Longer delay
    RCC_APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
    delay_ms(1);  // Longer delay

    // 4. Check if reset worked
    volatile uint32_t sr_after_reset = SPI1->SR;
    bool is_idle = !(sr_after_reset & (SPI_SR_TXC | SPI_SR_RXWNE | SPI_SR_SUSP));

    if (is_idle) {
        blink_led(1, 300); // Reset worked!
    } else {
        blink_led(15, 50); // Still not idle after reset
    }

    // 5. Test CFG1 access after reset
    if (is_idle) {
        SPI1->CFG1 = 0x12345678;
        volatile uint32_t cfg1_test = SPI1->CFG1;
        if (cfg1_test == 0x12345678) {
            blink_led(20, 300); // CFG1 now writable!
        } else {
            blink_led(16, 50); // CFG1 still not writable
        }
    }
}

void spi_complete_debug(void) {
    // 1. Test RCC register access
    volatile uint32_t rcc_before = RCC_APB2ENR;
    RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
    volatile uint32_t rcc_after = RCC_APB2ENR;
    bool clock_set = (rcc_after & RCC_APB2ENR_SPI1EN) != 0;

    if (!clock_set) {
        blink_led(2, 50); // Clock enable still failing
        return;
    }

    // 2. Test basic SPI register access
    volatile uint32_t cr1_orig = SPI1->CR1;
    SPI1->CR1 = 0x00000000;
    volatile uint32_t cr1_test = SPI1->CR1;
    SPI1->CR1 = cr1_orig;

    if (cr1_test != 0) {
        blink_led(3, 50); // Can't write to CR1
        return;
    }

    // 3. Test CFG1 register access (with SPI disabled)
    SPI1->CR1 = 0;                    // Disable SPI
    SPI1->CR2 = 0;                    // Clear CR2
    SPI1->IFCR = 0xFFFFFFFF;          // Clear all flags


    // Additional H7-specific reset - force SPI peripheral reset via RCC
#define RCC_APB2RSTR (*(volatile uint32_t*)(RCC_BASE + 0x098))
#define RCC_APB2RSTR_SPI1RST (1UL << 12)

    RCC_APB2RSTR |= RCC_APB2RSTR_SPI1RST;  // Assert reset
    for (volatile int i = 0; i < 1000; i++) __asm("nop"); // Short delay
    RCC_APB2RSTR &= ~RCC_APB2RSTR_SPI1RST; // Release reset
    for (volatile int i = 0; i < 1000; i++) __asm("nop"); // Short delay

    // Now try CFG1 access
    volatile uint32_t cfg1_orig = SPI1->CFG1;
    SPI1->CFG1 = 0x12345678;
    volatile uint32_t cfg1_test = SPI1->CFG1;
    SPI1->CFG1 = cfg1_orig;

    if (cfg1_test != 0x12345678) {
        blink_led(4, 50); // Still can't write to CFG1 even after reset
        return;
    }

    // 4. Try minimal SPI configuration
    SPI1->CR1 = 0;
    SPI1->CFG1 = (7 << 0) | (1 << 5) | (7 << 28); // 8-bit, FTHLV=1, slowest clock
    SPI1->CFG2 = SPI_CFG2_MASTER | SPI_CFG2_SSM;
    SPI1->CR2 = 0;
    SPI1->IFCR = 0xFFFFFFFF;

    // Try to enable SPI
    SPI1->CR1 = SPI_CR1_SPE | SPI_CR1_SSI;
    volatile uint32_t cr1_enable_test = SPI1->CR1;
    bool spe_stuck = (cr1_enable_test & SPI_CR1_SPE) != 0;

    if (!spe_stuck) {
        blink_led(5, 50); // SPI enable failed
        return;
    }

    // 5. Test transfer start
    SPI1->CR2 = 1; // Set transfer size
    SPI1->CR1 |= SPI_CR1_CSTART;
    delay_ms(1);
    volatile uint32_t sr_test = SPI1->SR;
    bool txp_appeared = (sr_test & SPI_SR_TXP) != 0;

    if (txp_appeared) {
        blink_led(1, 500); // SUCCESS!
    } else {
        blink_led(6, 50); // TXP didn't appear
    }

    // Detailed SR flag analysis
    volatile uint32_t sr = SPI1->SR;

    // Check each flag individually
    bool rxp = (sr & SPI_SR_RXP) != 0;        // Bit 0
    bool txp = (sr & SPI_SR_TXP) != 0;        // Bit 1
    bool dxp = (sr & SPI_SR_DXP) != 0;        // Bit 2
    bool eot = (sr & SPI_SR_EOT) != 0;        // Bit 3
    bool txtf = (sr & SPI_SR_TXTF) != 0;      // Bit 4
    bool udr = (sr & SPI_SR_UDR) != 0;        // Bit 5
    bool ovr = (sr & SPI_SR_OVR) != 0;        // Bit 6
    bool crce = (sr & SPI_SR_CRCE) != 0;      // Bit 7
    bool tife = (sr & SPI_SR_TIFE) != 0;      // Bit 8
    bool modf = (sr & SPI_SR_MODF) != 0;      // Bit 9
    bool tserf = (sr & SPI_SR_TSERF) != 0;    // Bit 10
    bool susp = (sr & SPI_SR_SUSP) != 0;      // Bit 11
    bool txc = (sr & SPI_SR_TXC) != 0;        // Bit 12
    bool rxwne = (sr & SPI_SR_RXWNE) != 0;    // Bit 15

    // Blink the raw SR value in binary (for debugging)
    // First show the full 32-bit value by blinking groups
    uint32_t sr_bits = sr;
    blink_led(99, 100); // Signal start of SR dump
    delay_ms(1000);

    // Blink each set bit position
    for (int bit = 0; bit < 16; bit++) {
        if (sr_bits & (1 << bit)) {
            blink_led(bit + 1, 200); // Blink (bit + 1) times for each set bit
            delay_ms(500);
        }
    }
    // Reset SPI
    SPI1->CR1 = 0;
}

void error_led(spi_status_t status) {
    if (status == SPI_OK) {
        blink_led(2, 50);
    }
    if (status == SPI_ERROR_INVALID_HANDLE) {
        blink_led(3, 50);
    }
    if (status == SPI_ERROR_INVALID_PARAM) {
        blink_led(4, 50);
    }
    if (status == SPI_ERROR_NOT_INITIALIZED) {
        blink_led(5, 50);
    }
    if (status == SPI_ERROR_ALREADY_INITIALIZED) {
        blink_led(6, 50);
    }
    if (status == SPI_ERROR_BUSY) {
        blink_led(7, 50);
    }
    if (status == SPI_ERROR_TIMEOUT) {
        blink_led(8, 50);
    }
    if (status == SPI_ERROR_HARDWARE_FAULT) {
        blink_led(9, 50);
    }
    if (status == SPI_ERROR_UNSUPPORTED_INSTANCE) {
        blink_led(10, 50);
    }
    if (status == SPI_ERROR_NO_AVAILABLE_HANDLES) {
        blink_led(11, 50);
    }
}

static bool test_spi_single_byte(spi_handle_t* spi, uint8_t test_byte) {
    uint8_t received = 0;

    spi_status_t status = spi_transfer_byte(spi, test_byte, &received);
    error_led(status);
    return (status == SPI_OK) && (received == test_byte);
}


void SystemInit(void) {
    // Basic system initialization - can be empty for now
}

void ExitRun0Mode(void) {
    // Exit run mode - can be empty for now
}

int main() {
    gpio_config_t config;
    systick_init();
    gpio_enable_clock(GPIOH);
    gpio_get_default_config(&config);
    config.mode = GPIO_MODE_OUTPUT;
    gpio_config_pin(GPIOH,LED_PIN, &config);
    gpio_toggle_pin(LED_PORT, LED_PIN);

    gpio_config_t pin_config;
    gpio_enable_clock(GPIOA);

    // Configure SPI pins as alternate function
    gpio_get_default_config(&pin_config);
    pin_config.mode = GPIO_MODE_ALTERNATE;
    pin_config.output_type = GPIO_OUTPUT_PUSH_PULL;
    pin_config.speed = GPIO_SPEED_VERY_HIGH;
    pin_config.pull = GPIO_PULL_NONE;
    pin_config.alternate_function = 5; // AF5 = SPI1

    gpio_config_pin(SPI_PORT,MOSI,&pin_config);
    gpio_config_pin(SPI_PORT,MISO,&pin_config);
    gpio_config_pin(SPI_PORT,SCK,&pin_config);

    spi_handle_t spi_handle;
    spi_config_t spi_config;

    spi_get_default_config(&spi_config);
    spi_config.clock_speed_hz = 1000000;  // Start with 1MHz
    spi_config.mode = SPI_MODE_0;         // CPOL=0, CPHA=0
    spi_config.data_size_bits = 8;
    spi_config.master_mode = true;
    spi_config.lsb_first = false;        // MSB first
    spi_config.fifo_threshold = 1;

    spi_complete_debug();
    //spi_status_t status = spi_init(&spi_handle, SPI1, &spi_config);
    //error_led(status);
    //test_spi_single_byte(&spi_handle,0x08);
    return 0;
}