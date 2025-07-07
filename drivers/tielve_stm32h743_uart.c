//
// Created by Jonathan Tielve
//

#include "tielve_stm32h743_uart.h"

#include <stdarg.h>

#define NULL ((void*)0)

static uart_handle_t* active_handles[UART_MAX_HANDLES] = {NULL};
static uint8_t active_handle_count = 0;

// ----------------
// Static Functions
// ----------------
static uart_status_t enable_uart_clock(USART_TypeDef* hardware, volatile uint32_t** rcc_reg, uint32_t* rcc_bit);
static uint32_t get_uart_clock_frequency(USART_TypeDef* hardware);
static bool is_handle_active(uart_handle_t* handle);
static uart_status_t add_handle_to_pool(uart_handle_t* handle);
static uart_status_t remove_handle_from_pool(uart_handle_t* handle);
static void simple_strcpy(char* dest, const char* src);
static int simple_strlen(const char* str);
static void simple_itoa(int value, char* str, int base);
static void simple_utoa(uint32_t value, char* str, int base);

static uart_status_t enable_uart_clock(USART_TypeDef* hardware, volatile uint32_t** rcc_reg, uint32_t* rcc_bit)
{
    if (hardware == USART1) {
        *rcc_reg = &RCC_APB2ENR;
        *rcc_bit = RCC_APB2ENR_USART1EN;
        RCC_APB2ENR |= RCC_APB2ENR_USART1EN;
    } else if (hardware == USART2) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_USART2EN;
        RCC_APB1LENR |= RCC_APB1LENR_USART2EN;
    } else if (hardware == USART3) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_USART3EN;
        RCC_APB1LENR |= RCC_APB1LENR_USART3EN;
    } else if (hardware == UART4) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_UART4EN;
        RCC_APB1LENR |= RCC_APB1LENR_UART4EN;
    } else if (hardware == UART5) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_UART5EN;
        RCC_APB1LENR |= RCC_APB1LENR_UART5EN;
    } else if (hardware == USART6) {
        *rcc_reg = &RCC_APB2ENR;
        *rcc_bit = RCC_APB2ENR_USART6EN;
        RCC_APB2ENR |= RCC_APB2ENR_USART6EN;
    } else if (hardware == UART7) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_UART7EN;
        RCC_APB1LENR |= RCC_APB1LENR_UART7EN;
    } else if (hardware == UART8) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_UART8EN;
        RCC_APB1LENR |= RCC_APB1LENR_UART8EN;
    } else {
        return UART_ERROR_UNSUPPORTED_INSTANCE;
    }

    // Small delay for clock stabilization
    for (volatile int i = 0; i < 100; i++) {
        __asm("nop");
    }

    return UART_OK;
}

static uint32_t get_uart_clock_frequency(USART_TypeDef* hardware) {
    if (hardware == USART1 || hardware == USART6) {
        return APB2_FREQUENCY_HZ;
    }
    else {
        return APB1_FREQUENCY_HZ;
    }
}

static bool is_handle_active(uart_handle_t* handle) {
    for (uint8_t i = 0; i < active_handle_count; i++) {
        if (active_handles[i] == handle) {
            return true;
        }
    }
    return false;
}

static uart_status_t add_handle_to_pool(uart_handle_t* handle) {
    if (active_handle_count >= UART_MAX_HANDLES) {
        return UART_ERROR_NO_AVAILABLE_HANDLES;
    }

    if (is_handle_active(handle)) {
        return UART_ERROR_ALREADY_INITIALIZED;
    }

    active_handles[active_handle_count] = handle;
    active_handle_count++;

    return UART_OK;
}

static uart_status_t remove_handle_from_pool(uart_handle_t* handle) {
    for (uint8_t i = 0; i < active_handle_count; i++) {
        if (active_handles[i] == handle) {
            for (uint8_t j = i; j < active_handle_count - 1; j++) {
                active_handles[j] = active_handles[j + 1];
            }
            active_handles[active_handle_count - 1] = NULL;
            active_handle_count--;
            return UART_OK;
        }
    }
    return UART_ERROR_INVALID_HANDLE;
}

static void simple_strcpy(char* dest, const char* src) {
    while (*src) {
        *dest++ = *src++;
    }
    *dest = '\0';
}

static int simple_strlen(const char* str) {
    int len = 0;
    while (*str++) len++;
    return len;
}

static void simple_itoa(int value, char* str, int base) {
    char* ptr = str;
    char* ptr1 = str;
    char tmp_char;
    int tmp_value;

    if (value < 0 && base == 10) {
        value = -value;
        *ptr++ = '-';
    }

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
    } while (value);

    if (ptr > str && *(ptr-1) != '-') {
        *ptr-- = '\0';
        while (ptr1 < ptr) {
            tmp_char = *ptr;
            *ptr-- = *ptr1;
            *ptr1++ = tmp_char;
        }
    } else {
        *ptr = '\0';
    }
}

static void simple_utoa(uint32_t value, char* str, int base) {
    char* ptr = str;
    char* ptr1 = str;
    char tmp_char;
    uint32_t tmp_value;

    do {
        tmp_value = value;
        value /= base;
        *ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35 + (tmp_value - value * base)];
    } while (value);

    *ptr-- = '\0';
    while (ptr1 < ptr) {
        tmp_char = *ptr;
        *ptr-- = *ptr1;
        *ptr1++ = tmp_char;
    }
}

// ----------------
// Public Functions
// ----------------
uart_status_t uart_init(uart_handle_t* handle, USART_TypeDef* hardware, const uart_config_t* config) {
    if (handle == NULL) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (hardware == NULL || config == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }
    if (handle->is_initialized) {
        return UART_ERROR_ALREADY_INITIALIZED;
    }
    if (config->baudrate == 0 || config->baudrate > 10000000) {
        return UART_ERROR_INVALID_PARAM;
    }

    // Enable RCC clock
    volatile uint32_t* rcc_register;
    uint32_t rcc_bit_mask;
    uart_status_t status = enable_uart_clock(hardware, &rcc_register, &rcc_bit_mask);
    if (status != UART_OK) {
        return status;
    }

    // Disable UART for config
    hardware->CR1 &= ~USART_CR1_UE;

    // Configure baud rate
    uint32_t uart_clock = get_uart_clock_frequency(hardware);
    uint32_t baudrate;

    // Configure oversampling
    if (config->oversampling_8) {
        baudrate = uart_clock / config->baudrate;
        hardware->CR1 |= USART_CR1_OVER8;
    } else {
        baudrate = uart_clock / config->baudrate;
        hardware->CR1 &= ~USART_CR1_OVER8;
    }

    hardware->BRR = baudrate;

    // Configure word length
    hardware->CR1 &= ~(USART_CR1_M0 | USART_CR1_M1);
    switch (config->word_length) {
        case UART_WORDLENGTH_7B:
            hardware->CR1 |= USART_CR1_M1;
            break;
        case UART_WORDLENGTH_8B:
            break;
        case UART_WORDLENGTH_9B:
            hardware->CR1 |= USART_CR1_M0;
            break;
    }

    // Configure parity
    hardware->CR1 &= ~(USART_CR1_PCE | USART_CR1_PS);
    switch (config->parity) {
        case UART_PARITY_NONE:
            break;
        case UART_PARITY_EVEN:
            hardware->CR1 |= USART_CR1_PCE;
            break;
        case UART_PARITY_ODD:
            hardware->CR1 |= USART_CR1_PCE | USART_CR1_PS;
            break;
    }

    // Configure CR2
    hardware->CR2 &= ~(USART_CR2_STOP); // Reset Stop bit
    hardware->CR2 |= (config->stop_bits << USART_CR2_STOP_POS); // Set stop bits

    // Reset flow control
    hardware->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
    switch (config->flow_control) {
        case UART_FLOWCONTROL_NONE:
            break;
        case UART_FLOWCONTROL_RTS:
            hardware->CR3 |= USART_CR3_RTSE;
            break;
        case UART_FLOWCONTROL_CTS:
            hardware->CR3 |= USART_CR3_CTSE;
            break;
        case UART_FLOWCONTROL_RTS_CTS:
            hardware->CR3 |= USART_CR3_CTSE | USART_CR3_RTSE;
            break;
    }

    // Enable TX/RX as configured
    if (config->enable_tx) {
        hardware->CR1 |= USART_CR1_TE;
    }

    if (config->enable_rx) {
        hardware->CR1 |= USART_CR1_RE;
    }

    handle->hardware = hardware;
    handle->config = *config;
    handle->is_initialized = true;
    handle->tx_busy = false;
    handle->rx_busy = false;
    handle->rcc_enable_reg = rcc_register;
    handle->rcc_enable_bit = rcc_bit_mask;

    handle->tx_buffer = NULL;
    handle->rx_buffer = NULL;
    handle->tx_buffer_size = 0;
    handle->rx_buffer_size = 0;
    handle->tx_head = 0;
    handle->tx_tail = 0;
    handle->rx_head = 0;
    handle->rx_tail = 0;

    hardware->CR1 |= USART_CR1_UE;

    status = add_handle_to_pool(handle);
    if (status != UART_OK) {
        hardware->CR1 &= ~USART_CR1_UE;
        handle->is_initialized = false;
        return status;
    }

    return UART_OK;
}


uart_status_t uart_init_with_buffers(uart_handle_t* handle, USART_TypeDef* hardware, const uart_config_t* config, uint8_t* tx_buffer, uint16_t tx_size, uint8_t* rx_buffer, uint16_t rx_size) {
    uart_status_t status = uart_init(handle, hardware, config);
    if (status != UART_OK) {
        return status;
    }

    // Set up buffers
    handle->tx_buffer = tx_buffer;
    handle->tx_buffer_size = tx_size;
    handle->rx_buffer = rx_buffer;
    handle->rx_buffer_size = rx_size;

    return UART_OK;
}


uart_status_t uart_deinit(uart_handle_t* handle) {
    if (handle == NULL) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (!handle->is_initialized) {
        return UART_ERROR_NOT_INITIALIZED;
    }
    if (handle->tx_busy || handle->rx_busy) {
        return UART_ERROR_BUSY;
    }

    USART_TypeDef* uart = handle->hardware;

    // Wait for transmission to complete
    uint32_t timeout_start = get_system_tick();
    while (!(uart->ISR & USART_ISR_TC)) {
        if (systick_timeout_elapsed(timeout_start, 100)) {
            break;
        }
    }

    // Disable UART
    uart->CR1 &= ~USART_CR1_UE;

    uart_status_t status = remove_handle_from_pool(handle);

    // Clear handle
    handle->hardware = NULL;
    handle->is_initialized = false;
    handle->tx_busy = false;
    handle->rx_busy = false;
    handle->rcc_enable_reg = NULL;
    handle->rcc_enable_bit = 0;
    handle->tx_buffer = NULL;
    handle->rx_buffer = NULL;
    handle->tx_buffer_size = 0;
    handle->rx_buffer_size = 0;

    return status;

}

uart_status_t uart_send_byte(uart_handle_t* handle, uint8_t data) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (!handle->config.enable_tx) {
        return UART_ERROR_INVALID_PARAM;
    }

    USART_TypeDef* uart = handle->hardware;
    uint32_t timeout_start = get_system_tick();

    // Wait for TXE flag
    while (!(uart->ISR & USART_ISR_TXFE)) {
        if (systick_timeout_elapsed(timeout_start, UART_DEFAULT_TIMEOUT_MS)) {
            return UART_ERROR_TIMEOUT;
        }
    }

    // Send data
    uart->TDR = data;

    // Wait for transmission complete
    timeout_start = get_system_tick();
    while (!(uart->ISR & USART_ISR_TC)) {
        if (systick_timeout_elapsed(timeout_start, UART_DEFAULT_TIMEOUT_MS)) {
            return UART_ERROR_TIMEOUT;
        }
    }

    return UART_OK;
}

uart_status_t uart_receive_byte(uart_handle_t* handle, uint8_t* data) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (data == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }
    if (!handle->config.enable_rx) {
        return UART_ERROR_INVALID_PARAM;
    }

    USART_TypeDef* uart = handle->hardware;
    uint32_t timeout_start = get_system_tick();

    // Wait for RXNE flag
    while (!(uart->ISR & USART_ISR_RXNE)) {
        if (systick_timeout_elapsed(timeout_start, UART_DEFAULT_TIMEOUT_MS)) {
            return UART_ERROR_TIMEOUT;
        }
    }

    // Read data
    *data = (uint8_t)uart->RDR;

    return UART_OK;

}

uart_status_t uart_send_buffer(uart_handle_t* handle, const uint8_t* buffer, uint16_t length) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (buffer == NULL || length == 0) {
        return UART_ERROR_INVALID_PARAM;
    }

    for (uint16_t i = 0; i < length; i++) {
        uart_status_t status = uart_send_byte(handle, buffer[i]);
        if (status != UART_OK) {
            return status;
        }
    }

    return UART_OK;
}

uart_status_t uart_receive_buffer(uart_handle_t* handle, uint8_t* buffer, uint16_t length) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (buffer == NULL || length == 0) {
        return UART_ERROR_INVALID_PARAM;
    }

    for (uint16_t i = 0; i < length; i++) {
        uart_status_t status = uart_receive_byte(handle, &buffer[i]);
        if (status != UART_OK) {
            return status;
        }
    }

    return UART_OK;
}

uart_status_t uart_send_string(uart_handle_t* handle, const char* str) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (str == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    while (*str) {
        uart_status_t status = uart_send_byte(handle, (uint8_t)*str);
        if (status != UART_OK) {
            return status;
        }
        str++;
    }

    return UART_OK;
}

uart_status_t uart_printf(uart_handle_t* handle, const char* format, ...) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (format == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    char buffer[512]; // Adjust size as needed
    char* ptr = buffer;
    const char* fmt_ptr = format;
    va_list args;

    va_start(args, format);

    while (*fmt_ptr && (ptr - buffer) < (sizeof(buffer) - 1)) {
        if (*fmt_ptr == '%') {
            fmt_ptr++;
            switch (*fmt_ptr) {
                case 'd': {
                    int val = va_arg(args, int);
                    char temp[12];
                    simple_itoa(val, temp, 10);
                    char* temp_ptr = temp;
                    while (*temp_ptr && (ptr - buffer) < (sizeof(buffer) - 1)) {
                        *ptr++ = *temp_ptr++;
                    }
                    break;
                }
                case 'u': {
                    uint32_t val = va_arg(args, uint32_t);
                    char temp[12];
                    simple_utoa(val, temp, 10);
                    char* temp_ptr = temp;
                    while (*temp_ptr && (ptr - buffer) < (sizeof(buffer) - 1)) {
                        *ptr++ = *temp_ptr++;
                    }
                    break;
                }
                case 'x': {
                    uint32_t val = va_arg(args, uint32_t);
                    char temp[9];
                    simple_utoa(val, temp, 16);
                    char* temp_ptr = temp;
                    while (*temp_ptr && (ptr - buffer) < (sizeof(buffer) - 1)) {
                        *ptr++ = *temp_ptr++;
                    }
                    break;
                }
                case 'X': {
                    uint32_t val = va_arg(args, uint32_t);
                    // Print as 8-digit hex with leading zeros
                    for (int k = 28; k >= 0; k -= 4) {
                        int digit = (val >> k) & 0xF;
                        *ptr++ = (digit < 10) ? ('0' + digit) : ('A' + digit - 10);
                        if ((ptr - buffer) >= (sizeof(buffer) - 1)) break;
                    }
                    break;
                }
                case 's': {
                    const char* str = va_arg(args, const char*);
                    while (*str && (ptr - buffer) < (sizeof(buffer) - 1)) {
                        *ptr++ = *str++;
                    }
                    break;
                }
                case 'c': {
                    *ptr++ = (char)va_arg(args, int);
                    break;
                }
                case '%': {
                    *ptr++ = '%';
                    break;
                }
                default:
                    *ptr++ = '%';
                    *ptr++ = *fmt_ptr;
                    break;
            }
        } else {
            *ptr++ = *fmt_ptr;
        }
        fmt_ptr++;
    }

    *ptr = '\0';
    va_end(args);

    return uart_send_string(handle, buffer);
}

bool uart_data_available(uart_handle_t* handle) {
    if (!is_handle_active(handle)) {
        return false;
    }

    return (handle->hardware->ISR & USART_ISR_RXNE) != 0;
}

uart_status_t uart_get_status(uart_handle_t* handle, bool* tx_busy, bool* rx_busy) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }
    if (tx_busy == NULL || rx_busy == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    *tx_busy = handle->tx_busy;
    *rx_busy = handle->rx_busy;

    return UART_OK;
}

uart_status_t uart_get_default_config(uart_config_t* config) {
    if (config == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    config->baudrate = 115200;
    config->word_length = UART_WORDLENGTH_8B;
    config->stop_bits = UART_STOPBITS_1;
    config->parity = UART_PARITY_NONE;
    config->flow_control = UART_FLOWCONTROL_NONE;
    config->oversampling_8 = false;
    config->enable_rx = true;
    config->enable_tx = true;

    return UART_OK;
}

uart_status_t uart_enable_interrupts(uart_handle_t* handle, bool enable_tx, bool enable_rx) {
    if (!is_handle_active(handle)) {
        return UART_ERROR_INVALID_HANDLE;
    }

    USART_TypeDef* uart = handle->hardware;

    if (enable_tx) {
        uart->CR1 |= USART_CR1_TXFEIE;
    } else {
        uart->CR1 &= ~USART_CR1_TXFEIE;
    }

    if (enable_rx) {
        uart->CR1 |= USART_CR1_RXFNEIE;
    } else {
        uart->CR1 &= ~USART_CR1_RXFNEIE;
    }

    return UART_OK;
}

void uart_irq_handler(uart_handle_t* handle) {
    if (!is_handle_active(handle)) {
        return;
    }

    USART_TypeDef* uart = handle->hardware;
    uint32_t status = uart->ISR;

    // Handle RX interrupt
    if ((status & USART_ISR_RXNE) && (uart->CR1 & USART_CR1_RXFNEIE)) {
        if (handle->rx_buffer != NULL) {
            uint8_t data = (uint8_t)uart->RDR;
            uint16_t next_head = (handle->rx_head + 1) % handle->rx_buffer_size;

            if (next_head != handle->rx_tail) {
                handle->rx_buffer[handle->rx_head] = data;
                handle->rx_head = next_head;
            }
        }
    }

    // Handle TX interrupt
    if ((status & USART_ISR_TXFE) && (uart->CR1 & USART_CR1_TXFEIE)) {
        if (handle->tx_buffer != NULL && handle->tx_tail != handle->tx_head) {
            uart->TDR = handle->tx_buffer[handle->tx_tail];
            handle->tx_tail = (handle->tx_tail + 1) % handle->tx_buffer_size;
        } else {
            uart->CR1 &= ~USART_CR1_TXFEIE;
            handle->tx_busy = false;
        }
    }
}

