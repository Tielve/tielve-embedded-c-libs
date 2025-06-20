/**
* @file tielve_stm32h743_spi.h
* @brief SPI Driver Implementation for STM32H743
* @author Jonathan Tielve
*
* Implementation of hardware abstraction layer for SPI communication.
* Provides handle-based resource management with comprehensive error handling.
*/

#ifndef TIELVE_STM32H743_SPI_H
#define TIELVE_STM32H743_SPI_H

#include <stdint.h>
#include <stdbool.h>

// -------------------
// Register Definitons
// -------------------
typedef struct {
    volatile uint32_t CR1;  ///< 0x00 Control Register 1
    volatile uint32_t CR2;  ///< 0x04 Control Register 2
    volatile uint32_t CFG1; ///< 0x08 Config Register 1
    volatile uint32_t CFG2; ///< 0x0C Config Register 2
    volatile uint32_t IER;  ///< 0x10 Interrupt Enable Register
    volatile uint32_t SR;   ///< 0x14 Status Register
    volatile uint32_t IFCR; ///< 0x18 Interrupt Flag Clear Register
    volatile uint32_t RESERVED0; ///< 0x1C Reserved
    volatile uint32_t TXDR; ///< 0x20 Transmit Data Register
    volatile uint32_t RESERVED1[3]; ///< 0x24-0x2C Reserved
    volatile uint32_t RXDR; ///< 0x30 Receive Data Register
    volatile uint32_t RESERVED2[3]; ///< 0x34-3C Reserved
} SPI_TypeDef;

/// Base addresses of SPI1-6
#define SPI1_BASE 0x40013000UL
#define SPI2_BASE 0x40003800UL
#define SPI3_BASE 0x40003C00UL
#define SPI4_BASE 0x40013400UL
#define SPI5_BASE 0x40015000UL
#define SPI6_BASE 0x58001400UL

#define SPI1 ((SPI_TypeDef*)SPI1_BASE)
#define SPI2 ((SPI_TypeDef*)SPI2_BASE)
#define SPI3 ((SPI_TypeDef*)SPI3_BASE)
#define SPI4 ((SPI_TypeDef*)SPI4_BASE)
#define SPI5 ((SPI_TypeDef*)SPI5_BASE)
#define SPI6 ((SPI_TypeDef*)SPI6_BASE)

/// RCC Advanced Peripheral Bus
#define RCC_BASE 0x58024400UL ///< RCC Base Address
#define RCC_APB2ENR (*(volatile uint32_t*)(RCC_BASE + 0x0F0)) ///< APB2 Enable
#define RCC_APB1LENR (*(volatile uint32_t*)(RCC_BASE + 0x0E8)) ///< APB1 Low Enable
#define RCC_APB4ENR (*(volatile uint32_t*)(RCC_BASE + 0x0F4)) ///< APB4 Enable

// ---------------
// Bit Definitions
// ---------------
// Control Register 1 Bit Definitions
#define SPI_CR1_SPE (1UL << 0) ///< SPI Enable
#define SPI_CR1_MASRX (1UL << 8) ///< Master auto suspend in receive
#define SPI_CR1_CSTART (1UL << 9) ///< Master transfer start
#define SPI_CR1_CSUSP (1UL << 10) ///< Master suspend
#define SPI_CR1_HDDIR (1UL << 11) ///< RX/TX direction in half duplex
#define SPI_CR1_SSI (1UL << 12) ///< Internal SS signal input level

// Configuration Register 1 Bit Definitions
#define SPI_CFG1_DSIZE_Pos 0 ///< DSIZE position
#define SPI_CFG1_DSIZE_Msk (0x1FUL << SPI_CFG1_DSIZE_Pos) ///< Size of DSIZE 0x1F = 5 bits
#define SPI_CFG1_FTHLV_Pos 5 ///< FIFO threshold level position
#define SPI_CFG1_FTHLV_Msk (0xFUL << SPI_CFG1_FTHLV_Pos) ///< Size of FTHLV 0xF = 4 bits
#define SPI_CFG1_UDRCFG_Pos 9 ///< Behavior of slave transmitter at underrun condition position
#define SPI_CFG1_RXDMAEN (1UL << 14) ///< RX DMA Stream Enable
#define SPI_CFG1_TXDMAEN (1UL << 15) ///< TX DMA Stream Enable
#define SPI_CFG1_CRCSIZE_Pos 16 ///< Length of CRC frame
#define SPI_CFG1_CRCEN (1UL << 22) ///< Hardware CRC enable
#define SPI_CFG1_MBR_Pos 28 ///< Master Baud Rate position
#define SPI_CFG1_MBR_Msk (0x7UL << SPI_CFG1_MBR_Pos) ///< Size of MBR 0x7 = 3 bits

// Configuration Register 2 Bit Definitions
#define SPI_CFG2_MSSI_Pos 0 ///< Master SS idleness position
#define SPI_CFG2_MIDI_Pos 4 ///< Master Inner Data idleness position
#define SPI_CFG2_IOSWP (1UL << 15) ///< Swap Functionality of MOSI and MISO
#define SPI_CFG2_COMM_Pos 17 ///< SPI communication mode position
#define SPI_CFG2_COMM_Msk (0x3UL << SPI_CFG2_COMM_Pos) ///< COMM size 0x3 = 2 bits
#define SPI_CFG2_SP_Pos 19 ///< Serial Protocol position
#define SPI_CFG2_SP_Msk (0x7UL << SPI_CFG2_SP_Pos) ///< SP size 0x7 = 3 bits
#define SPI_CFG2_MASTER (1UL << 22) ///< SPI Master, 0 = Slave, 1 = Master
#define SPI_CFG2_LSBFRST (1UL << 23) ///< Data Frame Format
#define SPI_CFG2_CPHA (1UL << 24) ///< Clock Phase 0 = rising edge, 1 = falling edge
#define SPI_CFG2_CPOL (1UL << 25) ///< Clock Polarity 0 = Idle Low, 1 = Idle High
#define SPI_CFG2_SSM (1UL << 26) ///< Software management of SS signal input
#define SPI_CFG2_SSIOP (1UL << 28) ///< SS io polarity
#define SPI_CFG2_SSOE (1UL << 29) ///< SS Output enable
#define SPI_CFG2_SSOM (1UL << 30) ///< SS Output management in master mode
#define SPI_CFG2_AFCNTR (1UL << 31) ///< Alternate function GPIOs control

// Status Register
#define SPI_SR_RXP (1UL << 0) ///< RX Packet available
#define SPI_SR_TXP (1UL << 1) ///< TX Packet available
#define SPI_SR_DXP (1UL << 2) ///< Duplex packet
#define SPI_SR_EOT (1UL << 3) ///< End of transfer
#define SPI_SR_TXTF (1UL << 4) ///< Transmission transfer filled
#define SPI_SR_UDR (1UL << 5) ///< Underrun
#define SPI_SR_OVR (1UL << 6) ///< Overrun
#define SPI_SR_CRCE (1UL << 7) ///< CRC error
#define SPI_SR_TIFE (1UL << 8) ///< TI frame format error
#define SPI_SR_MODF (1UL << 9) ///< Mode fault
#define SPI_SR_TSERF (1UL << 10) ///< Additional SPI data
#define SPI_SR_SUSP (1UL << 11) ///< Suspend status
#define SPI_SR_TXC (1UL << 12) ///< TxFIFO complete
#define SPI_SR_RXPLVL_Pos 13 ///< RxFIFO packing level
#define SPI_SR_RXPLVL_Msk (0x3UL << SPI_SR_RXPLVL_Pos) ///< 0x3 = 2 bits
#define SPI_SR_RXWNE (1UL << 15) ///< RxFIFO word not empty

// Interrupt flags clear register | bits 0-2 reserved
#define SPI_IFCR_EOTC (1UL << 3) ///< End of transfer flag clear
#define SPI_IFCR_TXTFC (1UL << 4) ///< Transmission transfer filled flag clear
#define SPI_IFCR_UDRC (1UL << 5) ///< Underrun flag clear
#define SPI_IFCR_OVRC (1UL << 6) ///< Overrun flag clear
#define SPI_IFCR_CRCEC (1UL << 7) ///< CRC error flag clear
#define SPI_IFCR_TIFEC (1UL << 8) ///< TI frame format error flag clear
#define SPI_IFCR_MODFC (1UL << 9) ///< Mode fault flag clear
#define SPI_IFCR_TSERFC (1UL << 10) ///< TSERFC flag clear
#define SPI_IFCR_SUSPC (1UL << 11) ///< Suspend flag clear

#define RCC_APB2ENR_SPI1EN (1UL << 12) ///< SPI1 enable
#define RCC_APB1LENR_SPI2EN (1UL << 14) ///< SPI2 enable
#define RCC_APB1LENR_SPI3EN (1UL << 15) ///< SPI3 enable
#define RCC_APB2ENR_SPI4EN (1UL << 13) ///< SPI4 enable
#define RCC_APB2ENR_SPI5EN (1UL << 20) ///< SPI5 enable
#define RCC_APB4ENR_SPI6EN (1UL << 5) ///< SPI6 enable

// ----------------
// Type Definitions
// ----------------
/**
 * @brief SPI driver status codes
 *
 * All SPI functions return one of these status codes to indicate
 * the result of the operation.
 */
typedef enum {
    SPI_OK = 0, ///< Operation completed successfully
    SPI_ERROR_INVALID_HANDLE, ///< Handle pointer is NULL
    SPI_ERROR_INVALID_PARAM, ///< Invalid parameter passed to function
    SPI_ERROR_NOT_INITIALIZED, ///< Handle not properly initialized
    SPI_ERROR_ALREADY_INITIALIZED, ///< Handle already initialized
    SPI_ERROR_BUSY, ///< SPI peripheral is busy
    SPI_ERROR_TIMEOUT, ///< Operation timed out
    SPI_ERROR_HARDWARE_FAULT, ///< Hardware error detected
    SPI_ERROR_UNSUPPORTED_INSTANCE, ///< Invalid SPI instance
    SPI_ERROR_NO_AVAILABLE_HANDLES ///< Maximum number of handles exceeded
} spi_status_t;

/**
 * @brief SPI communication modes
 *
 * Defines the CPOL and CPHA settings.
 *
 */
typedef enum {
    SPI_MODE_0 = 0, ///< CPOL=0 (Idle low), CPHA=0 (Rising edge)
    SPI_MODE_1 = 1, ///< CPOL=0 (Idle low), CPHA=1 (Falling edge)
    SPI_MODE_2 = 2, ///< CPOL=1 (Idle high), CPHA=0 (Rising edge)
    SPI_MODE_3 = 3 ///< CPOL=1 (Idle high), CPHA=1 (Falling edge)
} spi_mode_t;

/**
 * @brief SPI configuration structure
 *
 * Contains all parameters needed to configure an SPI peripheral.
 *
 */
typedef struct {
    uint32_t clock_speed_hz;    ///< SPI clock frequency in Hz
    spi_mode_t mode;           ///< SPI communication mode
    uint8_t data_size_bits;    ///< Data frame size
    uint8_t fifo_threshold;    ///< FIFO threshold level
    bool master_mode;          ///< true=master, false=slave
    bool lsb_first;           ///< true=LSB first, false=MSB first
} spi_config_t;

/**
 * @brief SPI handle structure
 *
 * Contains all state information for an SPI instance.
 *
 */
typedef struct {
    SPI_TypeDef* hardware; ///< Pointer to SPI hardware registers
    spi_config_t config; ///< Configuration
    uint32_t actual_clock_hz; ///< Actual achieved clock frequency
    bool is_initialized; ///< Initialization status
    bool transfer_in_progress; ///< Transfer busy flag
    volatile uint32_t* rcc_enable_reg; ///< RCC enable register
    uint32_t rcc_enable_bit; ///< RCC enable bit mask
} spi_handle_t;

// --------
// Defaults
// --------
#define SPI_MAX_HANDLES 6 ///< Maximum number of SPI handles
#define SPI_DEFAULT_FIFO_THRESHOLD 8 ///< Default FIFO threshold
#define SPI_DEFAULT_TIMEOUT_MS 1000 ///< Default timeout for SPI operations
#define SPI_MAX_DATA_SIZE_BITS 32 ///< Maximum supported data size in bits
#define SPI_MIN_DATA_SIZE_BITS 4 ///< Minimum supported data size in bits

//======================
// Function Declarations
//======================
/**
 * @brief Initialize SPI handle and configure hardware
 *
 * Configures the specified SPI peripheral according to the provided configuration.
 *
 * @param handle Pointer to SPI handle structure
 * @param hardware Pointer to SPI hardware
 * @param config Pointer to configuration structure
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_init(spi_handle_t* handle, SPI_TypeDef* hardware, const spi_config_t* config);

/**
 * @brief Deinitialize SPI handle and release resources
 *
 * Disables the SPI peripheral and marks the handle as uninitialized.
 *
 * @param handle Pointer to initialized handle
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_deinit(spi_handle_t* handle);

/**
 * @brief Transfer a single byte over SPI
 *
 * Performs a full-duplex transfer of one byte. Blocks until transfer completes or timeout occurs.
 *
 * @param handle Pointer to SPI handle
 * @param tx_data Data byte to transmit
 * @param rx_data Pointer to store received byte (can be NULL if not needed)
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_transfer_byte(spi_handle_t* handle, uint8_t tx_data, uint8_t* rx_data);

/**
 * @brief Transfer multiple bytes over SPI
 *
 * Performs full-duplex transfer of multiple bytes. Blocks until all data is transferred or timeout occurs.
 *
 * @param handle Pointer to SPI handle
 * @param tx_buffer Pointer to transmit data (can be NULL for receive-only)
 * @param rx_buffer Pointer to receive buffer (can be NULL for transmit-only)
 * @param length Number of bytes to transfer
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_transfer_buffer(spi_handle_t* handle, const uint8_t* tx_buffer, uint8_t* rx_buffer, uint16_t length);

/**
 * @brief Change SPI clock speed
 *
 * Reconfigures the SPI clock frequency. May adjust to nearest achievable frequency based on system clock and prescaler limitations.
 *
 * @param handle Pointer to SPI handle
 * @param speed_hz Desired clock frequency in Hz
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_set_clock_speed(spi_handle_t* handle, uint32_t speed_hz);

/**
 * @brief Get current SPI status
 *
 * Returns current operational status of the SPI peripheral.
 *
 * @param handle Pointer to SPI handle
 * @param is_busy Pointer to store busy status (true if transfer active)
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_get_status(spi_handle_t* handle, bool* is_busy);

/**
 * @brief Create default SPI configuration
 *
 * Initializes configuration structure with sensible defaults:
 * - 1MHz clock speed
 * - SPI Mode 0
 * - 8-bit data size
 * - Master mode
 * - MSB first
 * - Default FIFO threshold
 *
 * @param config Pointer to configuration structure to initialize
 *
 * @return SPI_OK on success, error code otherwise
 */
spi_status_t spi_get_default_config(spi_config_t* config);

// ------
// Macros
// ------
/**
 * @brief Check if SPI handle is valid and initialized
 */
#define SPI_IS_HANDLE_VALID(handle) ((handle) != NULL && (handle)->is_initialized && (handle)->hardware != NULL)

/**
 * @brief Convert SPI mode enum to CPOL value
 */
#define SPI_MODE_TO_CPOL(mode) (((mode) & 0x02) >> 1)

/**
 * @brief Convert SPI mode enum to CPHA value
 */
#define SPI_MODE_TO_CPHA(mode) ((mode) & 0x01)



#endif