/**
* @file tielve_stm32h743_spi.c
* @brief SPI Driver Implementation for STM32H743
* @author Jonathan Tielve
*
* Implementation of hardware abstraction layer for SPI communication.
* Provides handle-based resource management with comprehensive error handling.
*/

#include "tielve_stm32h743_spi.h"

#include <time.h>

#include "tielve_stm32h743_systick.h"

#define NULL ((void *)0)
#define APB2_FREQUENCY_HZ 240000000UL /// APB2 bus frequency (Hz)
#define APB1_FREQUENCY_HZ 120000000UL /// APB1 bus frequency (Hz)
#define APB4_FREQUENCY_HZ 120000000UL /// APB4 bus frequency (Hz)
#define MAX_PRESCALER_POWER 7 ///< Maximum number of prescaler values (2^0 to 2^7)

static spi_handle_t* active_handles[SPI_MAX_HANDLES] = {NULL}; ///< Handle pool to track active SPI instances
static uint8_t active_handle_count = 0; ///< Number of currently active handles

static uint8_t calculate_prescaler(uint32_t bus_freq_hz, uint32_t desired_freq_hz, uint32_t* actual_freq_hz);
static spi_status_t enable_spi_clock(SPI_TypeDef* hardware, volatile uint32_t** rcc_reg, uint32_t* rcc_bit);
static bool is_handle_active(spi_handle_t* handle);
static spi_status_t add_handle_to_pool(spi_handle_t* handle);
static spi_status_t remove_handle_from_pool(spi_handle_t* handle);

// ----------------
// Static Functions
// ----------------
static uint8_t calculate_prescaler(uint32_t bus_freq_hz, uint32_t desired_freq_hz, uint32_t* actual_freq_hz)
{
    if (actual_freq_hz == NULL || desired_freq_hz == 0) {
        return 0xFF;  // Invalid parameters
    }

    // Find the best prescaler (closest to but not exceeding desired frequency)
    uint8_t best_prescaler = 0xFF;
    uint32_t best_frequency = 0;

    for (uint8_t prescaler_power = 0; prescaler_power <= MAX_PRESCALER_POWER; prescaler_power++) {
        uint32_t prescaler_value = 1UL << (prescaler_power + 1);
        uint32_t frequency = bus_freq_hz / prescaler_value;

        if (frequency <= desired_freq_hz && frequency > best_frequency) {
            best_frequency = frequency;
            best_prescaler = prescaler_power;
        }
    }

    if (best_prescaler == 0xFF) {
        return 0xFF;
    }

    *actual_freq_hz = best_frequency;
    return best_prescaler;
}

static spi_status_t enable_spi_clock(SPI_TypeDef* hardware, volatile uint32_t** rcc_reg, uint32_t* rcc_bit)
{
    if (hardware == SPI1) {
        *rcc_reg = &RCC_APB2ENR;
        *rcc_bit = RCC_APB2ENR_SPI1EN;
        RCC_APB2ENR |= RCC_APB2ENR_SPI1EN;
    } else if (hardware == SPI2) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_SPI2EN;
        RCC_APB1LENR |= RCC_APB1LENR_SPI2EN;
    } else if (hardware == SPI3) {
        *rcc_reg = &RCC_APB1LENR;
        *rcc_bit = RCC_APB1LENR_SPI3EN;
        RCC_APB1LENR |= RCC_APB1LENR_SPI3EN;
    } else if (hardware == SPI4) {
        *rcc_reg = &RCC_APB2ENR;
        *rcc_bit = RCC_APB2ENR_SPI4EN;
        RCC_APB2ENR |= RCC_APB2ENR_SPI4EN;
    } else if (hardware == SPI5) {
        *rcc_reg = &RCC_APB2ENR;
        *rcc_bit = RCC_APB2ENR_SPI5EN;
        RCC_APB2ENR |= RCC_APB2ENR_SPI5EN;
    } else if (hardware == SPI6) {
        *rcc_reg = &RCC_APB4ENR;
        *rcc_bit = RCC_APB4ENR_SPI6EN;
        RCC_APB4ENR |= RCC_APB4ENR_SPI6EN;
    } else {
        return SPI_ERROR_UNSUPPORTED_INSTANCE;
    }

    // Small delay to ensure clock is stable
    for (volatile int i = 0; i < 100; i++) {
        __asm("nop");
    }

    return SPI_OK;
}

static bool is_handle_active(spi_handle_t* handle)
{
    for (uint8_t i = 0; i < active_handle_count; i++) {
        if (active_handles[i] == handle) {
            return true;
        }
    }
    return false;
}

static spi_status_t add_handle_to_pool(spi_handle_t* handle)
{
    if (active_handle_count >= SPI_MAX_HANDLES) {
        return SPI_ERROR_NO_AVAILABLE_HANDLES;
    }

    if (is_handle_active(handle)) {
        return SPI_ERROR_ALREADY_INITIALIZED;
    }

    active_handles[active_handle_count] = handle;
    active_handle_count++;

    return SPI_OK;
}

static spi_status_t remove_handle_from_pool(spi_handle_t* handle)
{
    for (uint8_t i = 0; i < active_handle_count; i++) {
        if (active_handles[i] == handle) {
            for (uint8_t j = i; j < active_handle_count - 1; j++) {
                active_handles[j] = active_handles[j + 1];
            }
            active_handles[active_handle_count - 1] = NULL;
            active_handle_count--;
            return SPI_OK;
        }
    }

    return SPI_ERROR_INVALID_HANDLE;
}

spi_status_t spi_init(spi_handle_t* handle, SPI_TypeDef* hardware, const spi_config_t* config)
{
    // Error handling
    if (handle == NULL) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (hardware == NULL || config == NULL) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (handle->is_initialized) {
        return SPI_ERROR_ALREADY_INITIALIZED;
    }
    if (config->data_size_bits < SPI_MIN_DATA_SIZE_BITS || config->data_size_bits > SPI_MAX_DATA_SIZE_BITS) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (config->mode > SPI_MODE_3) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (config->fifo_threshold > 15) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (active_handle_count >= SPI_MAX_HANDLES) {
        return SPI_ERROR_NO_AVAILABLE_HANDLES;
    }

    // Enable RCC clock for this SPI instance
    volatile uint32_t* rcc_register;
    uint32_t rcc_bit_mask;
    spi_status_t status = enable_spi_clock(hardware, &rcc_register, &rcc_bit_mask);
    if (status != SPI_OK) {
        return status;
    }

    uint32_t bus_frequency;
    //SP1,4,5 use APB2 | SPI6 uses APB4, and SPI2,3 use APB1
    if (hardware == SPI1 || hardware == SPI4 || hardware == SPI5) {
        bus_frequency = APB2_FREQUENCY_HZ;
    } else if (hardware == SPI6) {
        bus_frequency = APB4_FREQUENCY_HZ;
    } else {
        bus_frequency = APB1_FREQUENCY_HZ;  // SPI2, SPI3
    }

    uint32_t actual_clock_hz;
    uint8_t mbr_value = calculate_prescaler(bus_frequency, config->clock_speed_hz, &actual_clock_hz);
    if (mbr_value == 0xFF) {
        return SPI_ERROR_INVALID_PARAM;  // Impossible clock speed
    }

    // Disable SPI during configuration
    hardware->CR1 &= ~SPI_CR1_SPE;

    // Configure CFG1 register (data size, FIFO threshold, baud rate)
    uint32_t cfg1_reg = 0;
    cfg1_reg |= ((config->data_size_bits - 1) << SPI_CFG1_DSIZE_Pos) & SPI_CFG1_DSIZE_Msk;
    cfg1_reg |= (config->fifo_threshold << SPI_CFG1_FTHLV_Pos) & SPI_CFG1_FTHLV_Msk;
    cfg1_reg |= (mbr_value << SPI_CFG1_MBR_Pos) & SPI_CFG1_MBR_Msk;
    hardware->CFG1 = cfg1_reg;

    // Configure CFG2 register (mode, master/slave, bit order)
    uint32_t cfg2_reg = 0;
    cfg2_reg |= (0 << SPI_CFG2_COMM_Pos) & SPI_CFG2_COMM_Msk;

    if (config->master_mode) {
        cfg2_reg |= SPI_CFG2_MASTER;
    }
    if (config->lsb_first) {
        cfg2_reg |= SPI_CFG2_LSBFRST;
    }

    // Set CPOL and CPHA based on mode
    if (SPI_MODE_TO_CPOL(config->mode)) {
        cfg2_reg |= SPI_CFG2_CPOL;
    }
    if (SPI_MODE_TO_CPHA(config->mode)) {
        cfg2_reg |= SPI_CFG2_CPHA;
    }

    // Enable software slave management for master mode
    if (config->master_mode) {
        cfg2_reg |= SPI_CFG2_SSM;
        cfg2_reg |= SPI_CFG2_SSIOP;
    }

    hardware->CFG2 = cfg2_reg;

    // Set transfer size to 0 (unlimited/manual control)
    hardware->CR2 = 1;

    // Clear any pending flags
    hardware->IFCR = SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
                     SPI_IFCR_OVRC | SPI_IFCR_CRCEC | SPI_IFCR_TIFEC |
                     SPI_IFCR_MODFC | SPI_IFCR_TSERFC | SPI_IFCR_SUSPC;


    // Enable SPI peripheral
    hardware->CR1 |= SPI_CR1_SPE;

    if (config->master_mode) {
        hardware->CR1 |= SPI_CR1_SSI;
    }

    // Initialize handle structure
    handle->hardware = hardware;
    handle->config = *config;
    handle->actual_clock_hz = actual_clock_hz;
    handle->is_initialized = true;
    handle->transfer_in_progress = false;
    handle->rcc_enable_reg = rcc_register;
    handle->rcc_enable_bit = rcc_bit_mask;

    // Add to active handle pool
    status = add_handle_to_pool(handle);
    if (status != SPI_OK) {
        // Cleanup on failure
        hardware->CR1 &= ~SPI_CR1_SPE;
        handle->is_initialized = false;
        return status;
    }

    return SPI_OK;
}

spi_status_t spi_deinit(spi_handle_t* handle)
{
    if (handle == NULL) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (!handle->is_initialized) {
        return SPI_ERROR_NOT_INITIALIZED;
    }
    if (handle->transfer_in_progress) {
        return SPI_ERROR_BUSY;
    }

    SPI_TypeDef* spi = handle->hardware;

    uint32_t timeout_start = get_system_tick();
    while (!(spi->SR & SPI_SR_TXC) || (spi->SR & SPI_SR_RXWNE)) {
        if (systick_timeout_elapsed(timeout_start, 100)) {
            break;
        }
    }

    // Disable SPI
    spi->CR1 &= ~SPI_CR1_SPE;

    // Clear all flags
    spi->IFCR |= SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_UDRC |
                 SPI_IFCR_OVRC | SPI_IFCR_CRCEC | SPI_IFCR_TIFEC |
                 SPI_IFCR_MODFC | SPI_IFCR_TSERFC | SPI_IFCR_SUSPC;

    // Optionally disable the RCC clock (commented out - might affect other instances)
    // *handle->rcc_enable_reg &= ~handle->rcc_enable_bit;

    spi_status_t pool_status = remove_handle_from_pool(handle);

    // Clear handle
    handle->hardware = NULL;
    handle->actual_clock_hz = 0;
    handle->is_initialized = false;
    handle->transfer_in_progress = false;
    handle->rcc_enable_reg = NULL;
    handle->rcc_enable_bit = 0;

    // Zero out config
    handle->config.clock_speed_hz = 0;
    handle->config.mode = SPI_MODE_0;
    handle->config.data_size_bits = 8;
    handle->config.fifo_threshold = 0;
    handle->config.master_mode = false;
    handle->config.lsb_first = false;

    return pool_status;
}

spi_status_t spi_transfer_byte(spi_handle_t* handle, uint8_t tx_data, uint8_t* rx_data)
{
    // Error handling
    if (!SPI_IS_HANDLE_VALID(handle)) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (handle->transfer_in_progress) {
        return SPI_ERROR_BUSY;
    }
    if (handle->config.data_size_bits != 8) {
        return SPI_ERROR_INVALID_PARAM;
    }

    SPI_TypeDef* spi = handle->hardware;
    uint32_t timeout_start = get_system_tick();

    handle->transfer_in_progress = true;

    // Start Transfer
    spi->CR1 |= SPI_CR1_CSTART;

    // Wait for TXP flag
    while (!(spi->SR & SPI_SR_TXP)) {
        if (systick_timeout_elapsed(timeout_start, SPI_DEFAULT_TIMEOUT_MS)) {
            handle->transfer_in_progress = false;
            return SPI_ERROR_TIMEOUT;
        }
    }

    // Write data to transmit register
    spi->TXDR = tx_data;

    // Wait for RXP flag
    while (!(spi->SR & SPI_SR_RXP)) {
        if (systick_timeout_elapsed(timeout_start, SPI_DEFAULT_TIMEOUT_MS)) {
            handle->transfer_in_progress = false;
            return SPI_ERROR_TIMEOUT;
        }
    }

    uint8_t received_data = (uint8_t)spi->RXDR;

    // Wait for EOT flag
    while (!(spi->SR & SPI_SR_EOT)) {
        if (systick_timeout_elapsed(timeout_start, SPI_DEFAULT_TIMEOUT_MS)) {
            handle->transfer_in_progress = false;
            return SPI_ERROR_TIMEOUT;
        }
    }

    // Clear EOT flag (write 1 to clear)
    spi->IFCR |= SPI_IFCR_EOTC;

    // Check for errors that might have occurred during transfer
    uint32_t status_reg = spi->SR;
    if (status_reg & (SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF)) {
        spi->IFCR |= SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC;
        handle->transfer_in_progress = false;
        return SPI_ERROR_HARDWARE_FAULT;
    }

    // Transfer complete
    handle->transfer_in_progress = false;

    // Return received data if requested
    if (rx_data != NULL) {
        *rx_data = received_data;
    }

    return SPI_OK;
}

spi_status_t spi_transfer_buffer(spi_handle_t* handle, const uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t length)
{
    if (!SPI_IS_HANDLE_VALID(handle)) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (length == 0) {
        return SPI_OK;
    }
    if (tx_buffer == NULL && rx_buffer == NULL) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (handle->transfer_in_progress) {
        return SPI_ERROR_BUSY;
    }
    if (handle->config.data_size_bits != 8) {
        return SPI_ERROR_INVALID_PARAM;
    }

    SPI_TypeDef* spi = handle->hardware;
    uint32_t timeout_start = get_system_tick();

    // Start transfer
    handle->transfer_in_progress = true;
    spi->CR1 |= SPI_CR1_CSTART;

    for (uint16_t i = 0; i < length; i++) {
        if ((i & 0xFF) == 0) {  // Check timeout every 256 bytes
            if (systick_timeout_elapsed(timeout_start, SPI_DEFAULT_TIMEOUT_MS)) {
                handle->transfer_in_progress = false;
                return SPI_ERROR_TIMEOUT;
            }
        }

        // Wait for TXP flag
        uint32_t tx_timeout = get_system_tick();
        while (!(spi->SR & SPI_SR_TXP)) {
            if (systick_timeout_elapsed(tx_timeout, 10)){
                handle->transfer_in_progress = false;
                return SPI_ERROR_TIMEOUT;
            }
        }

        // Send data or dummy byte for RX only
        if (tx_buffer != NULL) {
            spi->TXDR = tx_buffer[i];
        } else {
            spi->TXDR = 0xFF;  // Dummy byte
        }

        // Wait for RXP flag
        uint32_t rx_timeout = get_system_tick();
        while (!(spi->SR & SPI_SR_RXP)) {
            if (systick_timeout_elapsed(rx_timeout, 10)) {
                handle->transfer_in_progress = false;
                return SPI_ERROR_TIMEOUT;
            }
        }

        // Read data
        uint8_t received_byte = (uint8_t)spi->RXDR;
        if (rx_buffer != NULL) {
            rx_buffer[i] = received_byte;
        }

        // Check for errors every 64 bytes
        if ((i & 0x3F) == 0) {
            uint32_t status_reg = spi->SR;
            if (status_reg & (SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF)) {
                spi->IFCR |= SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC;
                handle->transfer_in_progress = false;
                return SPI_ERROR_HARDWARE_FAULT;
            }
        }
    }

    // Wait for EOT
    uint32_t eot_timeout = get_system_tick();
    while (!(spi->SR & SPI_SR_EOT)) {
        if (systick_timeout_elapsed(eot_timeout, 100)) {
            handle->transfer_in_progress = false;
            return SPI_ERROR_TIMEOUT;
        }
    }

    // Clear EOT flag
    spi->IFCR |= SPI_IFCR_EOTC;

    // Error check
    uint32_t final_status = spi->SR;
    if (final_status & (SPI_SR_UDR | SPI_SR_OVR | SPI_SR_MODF)) {
        // Clear error flags
        spi->IFCR |= SPI_IFCR_UDRC | SPI_IFCR_OVRC | SPI_IFCR_MODFC;
        handle->transfer_in_progress = false;
        return SPI_ERROR_HARDWARE_FAULT;
    }

    handle->transfer_in_progress = false;
    return SPI_OK;
}

spi_status_t spi_set_clock_speed(spi_handle_t* handle, uint32_t speed_hz)
{
    // Error handling
    if (!SPI_IS_HANDLE_VALID(handle)) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (speed_hz == 0) {
        return SPI_ERROR_INVALID_PARAM;
    }
    if (handle->transfer_in_progress) {
        return SPI_ERROR_BUSY;
    }

    // If already at desired speed return OK
    uint32_t current_speed = handle->actual_clock_hz;
    uint32_t speed_diff = (current_speed > speed_hz) ?
                         (current_speed - speed_hz) : (speed_hz - current_speed);
    if ((speed_diff * 100) < current_speed) {  // Less than 1% difference
        return SPI_OK;  // Close enough, no need to reconfigure
    }

    SPI_TypeDef* spi = handle->hardware;

    // Determine bus frequency
    uint32_t bus_frequency;
    if (spi == SPI1 || spi == SPI4 || spi == SPI5) {
        bus_frequency = APB2_FREQUENCY_HZ;
    } else if (spi == SPI6) {
        bus_frequency = APB4_FREQUENCY_HZ;
    } else {
        bus_frequency = APB1_FREQUENCY_HZ;
    }

    // Calculate new prescaler value
    uint32_t new_actual_speed;
    uint8_t new_mbr = calculate_prescaler(bus_frequency, speed_hz, &new_actual_speed);
    if (new_mbr == 0xFF) {
        return SPI_ERROR_INVALID_PARAM;
    }

    // Safely disable SPI for reconfiguration
    uint32_t timeout_start = get_system_tick();
    while (spi->SR & (SPI_SR_TXC | SPI_SR_RXWNE)) {
        if (systick_timeout_elapsed(timeout_start, 100)) {
            return SPI_ERROR_TIMEOUT;
        }
    }
    spi->CR1 &= ~SPI_CR1_SPE;

    // Update CFG1 register
    uint32_t cfg1_value = spi->CFG1;

    cfg1_value &= ~SPI_CFG1_MBR_Msk; // Clear MBR value
    cfg1_value |= (new_mbr << SPI_CFG1_MBR_Pos) & SPI_CFG1_MBR_Msk; // Set new MBR
    spi->CFG1 = cfg1_value;

    // Enable SPI
    spi->CR1 |= SPI_CR1_SPE;

    // Update handle
    handle->actual_clock_hz = new_actual_speed;
    handle->config.clock_speed_hz = speed_hz;

    return SPI_OK;
}

spi_status_t spi_get_status(spi_handle_t* handle, bool* is_busy)
{
    if (!SPI_IS_HANDLE_VALID(handle)) {
        return SPI_ERROR_INVALID_HANDLE;
    }
    if (is_busy == NULL) {
        return SPI_ERROR_INVALID_PARAM;
    }

    // Check if transfer is in progress
    *is_busy = handle->transfer_in_progress;
    SPI_TypeDef* spi = handle->hardware;
    uint32_t hw_status = spi->SR;

    // Check if hw is busy
    if (!(hw_status & SPI_SR_TXC) || // if TX FIFO is not empty
        (hw_status & SPI_SR_RXWNE) || // if RX FIFO has data waiting
        (hw_status & SPI_SR_SUSP)) { // if transfer is suspended
        *is_busy = true;
    }

    return SPI_OK;
}

spi_status_t spi_get_default_config(spi_config_t* config)
{
    if (config == NULL) {
        return SPI_ERROR_INVALID_PARAM;
    }

    config->clock_speed_hz = 1000000;
    config->mode = SPI_MODE_0;
    config->data_size_bits = 8;
    config->fifo_threshold = SPI_DEFAULT_FIFO_THRESHOLD;
    config->master_mode = true;
    config->lsb_first = false;

    return SPI_OK;
}