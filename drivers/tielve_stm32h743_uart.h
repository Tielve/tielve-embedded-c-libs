//
// Created by Jonathan Tielve
//

#ifndef TIELVE_STM32H743_UART_H
#define TIELVE_STM32H743_UART_H

#include "tielve_stm32h743_systick.h"
#include "tielve_stm32h743_rcc.h"
#include <stdbool.h>
#include <stdint.h>

// --------------------
// Register Definitions
// --------------------
typedef struct {
    volatile uint32_t CR1;    ///< 0x00 Control register 1
    volatile uint32_t CR2;    ///< 0x04 Control register 2
    volatile uint32_t CR3;    ///< 0x08 Control register 3
    volatile uint32_t BRR;    ///< 0x0C Baud rate register
    volatile uint32_t GTPR;   ///< 0x10 Guard time and prescaler register
    volatile uint32_t RTOR;   ///< 0x14 Receiver timeout register
    volatile uint32_t RQR;    ///< 0x18 Request register
    volatile uint32_t ISR;    ///< 0x1C Interrupt and status register
    volatile uint32_t ICR;    ///< 0x20 Interrupt flag clear register
    volatile uint32_t RDR;    ///< 0x24 Receive data register
    volatile uint32_t TDR;    ///< 0x28 Transmit data register
    volatile uint32_t PRESC;  ///< 0x2C Prescaler register
} USART_TypeDef;

/// Base Addresses
#define USART1_BASE 0x40011000UL
#define USART2_BASE 0x40004400UL
#define USART3_BASE 0x40004800UL
#define UART4_BASE 0x40004C00UL
#define UART5_BASE 0x40005000UL
#define USART6_BASE 0x40011400UL
#define UART7_BASE 0x40007800UL
#define UART8_BASE 0x40007C00UL

#define USART1 ((USART_TypeDef *)USART1_BASE)
#define USART2 ((USART_TypeDef *)USART2_BASE)
#define USART3 ((USART_TypeDef *)USART3_BASE)
#define UART4 ((USART_TypeDef *)UART4_BASE)
#define UART5 ((USART_TypeDef *)UART5_BASE)
#define USART6 ((USART_TypeDef *)USART6_BASE)
#define UART7 ((USART_TypeDef *)UART7_BASE)
#define UART8 ((USART_TypeDef *)UART8_BASE)

// ---------------
// Bit Definitions
// ---------------
#define USART_CR1_UE (1UL << 0) ///< USART enable
#define USART_CR1_UESM (1UL << 1) ///< USART enable in low power
#define USART_CR1_RE (1UL << 2) ///< Receiver enable
#define USART_CR1_TE (1UL << 3) ///< Transmitter enable
#define USART_CR1_IDLEIE (1UL << 4) ///< IDLE interrupt enable
#define USART_CR1_RXFNEIE (1UL << 5) ///< RXFIFO not empty interrupt enable
#define USART_CR1_TCIE (1UL << 6) ///< Transmission complete interrupt enable
#define USART_CR1_TXFNFIE (1UL << 7) ///< TXFIFO not full interrupt enable
#define USART_CR1_PEIE (1UL << 8) ///< PE interrupt enable
#define USART_CR1_PS (1UL << 9) ///< Parity Selection
#define USART_CR1_PCE (1UL << 10) ///< Parity control enable
#define USART_CR1_WAKE (1UL << 11) ///< Receiver wake-up method
#define USART_CR1_M0 (1UL << 12) ///< Word Length
#define USART_CR1_MME (1UL << 13) ///< Mute mode enable
#define USART_CR1_CMIE (1UL << 14) ///< Character match interrupt enable
#define USART_CR1_OVER8 (1UL << 15) ///< Oversampling mode
#define USART_CR1_DEDT (0xFUL << 16) ///< Driver enable deassertion time
#define USART_CR1_DEAT (0xFUL << 21) ///< Driver enable assertion time
#define USART_CR1_RTOIE (1UL << 26) ///< Receiver timeout interrupt enable
#define USART_CR1_EOBIE (1UL << 27) ///< End of block interrupt enable
#define USART_CR1_M1 (1UL << 28) ///< Word Length
#define USART_CR1_FIFOEN (1UL << 29) ///< FIFO mode enable
#define USART_CR1_TXFEIE (1UL << 30) ///< TXFIFO empty interrupt enable
#define USART_CR1_RXFFIE (1UL << 31) ///< RXFIFO full interrupt enable

#define USART_CR2_SLVEN (1UL << 0) ///< Synchronous slave mode enable
#define USART_CR2_DIS_NSS (1UL << 3) ///< NSS pin enable
#define USART_CR2_ADDM7 (1UL << 4) ///< 7/4 bit detection
#define USART_CR2_LBDL (1UL << 5) ///< LIN break detection length
#define USART_CR2_LBDIE (1UL << 6) ///< LIN break detection interrupt enable
#define USART_CR2_LBCL (1UL << 8) ///< Last bit clock pulse
#define USART_CR2_CPHA (1UL << 9) ///< Clock phase
#define USART_CR2_CPOL (1UL << 10) ///< Clock polarity
#define USART_CR2_CLKEN (1UL << 11) ///< Clock enable
#define USART_CR2_STOP_POS 12
#define USART_CR2_STOP (0x3UL << 12) ///< Stop bits
#define USART_CR2_LINEN (1UL << 14) ///< LIN mode enable
#define USART_CR2_SWAP (1UL << 15) ///< Swap TX/RX pins
#define USART_CR2_RXINV (1UL << 16) ///< RX pin active level inversion
#define USART_CR2_TXINV (1UL << 17) ///< TX pin active level inversion
#define USART_CR2_DATAINV (1UL << 18) ///< Binary data inversion
#define USART_CR2_MSBFIRST (1UL << 19) ///< Most significant bit first
#define USART_CR2_ABREN (1UL << 20) ///< Auto baud rate enable
#define USART_CR2_ABRMOD (0x3UL << 21) ///< Auto baud rate mode
#define USART_CR2_RTOEN (1UL << 23) ///< Receiver timeout enable
#define USART_CR2_ADD (0xFFUL << 24) ///< Address of the usart node

#define USART_CR3_EIE (1UL << 0) ///< Error interrupt enable
#define USART_CR3_IREN (1UL << 1) ///< IrDA mode enable
#define USART_CR3_IRLP (1UL << 2) ///< IrDA low power
#define USART_CR3_HDSEL (1UL << 3) ///< Half duplex selection
#define USART_CR3_NACK (1UL << 4) ///< Smartcard NACK enable
#define USART_CR3_SCEN (1UL << 5) ///< Smartcard mode enable
#define USART_CR3_DMAR (1UL << 6) ///< DMA enable receiver
#define USART_CR3_DMAT (1UL << 7) ///< DMA enable transmitter
#define USART_CR3_RTSE (1UL << 8) ///< RTS enable
#define USART_CR3_CTSE (1UL << 9) ///< CTS enable
#define USART_CR3_CTSIE (1UL << 10) ///< CTS interrupt enable
#define USART_CR3_ONEBIT (1UL << 11) ///< One sample bit method enable
#define USART_CR3_OVRDIS (1UL << 12) ///< Overrun disable
#define USART_CR3_DDRE (1UL << 13) ///< DMA disable on reception error
#define USART_CR3_DEM (1UL << 14) ///< Driver enable mode
#define USART_CR3_DEP (1UL << 15) ///< Driver enable polarity selection
#define USART_CR3_SCARCNT (0x7UL << 17) ///< Smartcard auto retry count
#define USART_CR3_WUS (0x3UL << 20) ///< Wake up from low power mode interrupt flag selection
#define USART_CR3_WUFIE (1UL << 22) ///< Wake up from low power mode interrupt enable
#define USART_CR3_TXFTIE (1UL << 23) ///< TXFIFO threshold interrupt enable
#define USART_CR3_TCBGTIE (1UL << 24) ///< Transmission complete before guard time, interrupt enable
#define USART_CR3_RXFTCFG (0x7UL << 25) ///< Receive FIFO threshold configuration
#define USART_CR3_RXFTIE (1UL << 28) ///< RXFIFO threshold interrupt enable
#define USART_CR3_TXFTCFG (0x7UL << 29) ///< TXFIFO threshold config

#define USART_BRR (0xFFFFUL << 0) ///< Baud Rate

#define USART_RTOR_RTO (0xFFFFFFUL << 0) ///< Receiver timeout value
#define USART_RTOR_BLEN (0xFFUL << 24) ///< Block length

#define USART_RQR_ABRRQ (1UL << 0) ///< Auto baud rate request
#define USART_RQR_SBKRQ (1UL << 1) ///< Send break request
#define USART_RQR_MMRQ (1UL << 2) ///< Mute mode request
#define USART_RQR_RXFRQ (1UL << 3) ///< Receive data flush request
#define USART_RQR_TXFRQ (1UL << 4) ///< Transmit data flush request

#define USART_ISR_PE (1UL << 0) ///< Parity Error
#define USART_ISR_FE (1UL << 1) ///< Framing error
#define USART_ISR_NE (1UL << 2) ///< Noise detection flag
#define USART_ISR_ORE (1UL << 3) ///< Overrun error
#define USART_ISR_IDLE (1UL << 4) ///< Idle line detected
#define USART_ISR_RXNE (1UL << 5)  ///< Read data register not empty
#define USART_ISR_TC (1UL << 6)  ///< Transmission complete
#define USART_ISR_TXFNF (1UL << 7)  ///< Transmit data register not full
#define USART_ISR_LBDF (1UL << 8) ///< LIN break detection flag
#define USART_ISR_CTSIF (1UL << 9) ///< CTS interrupt flag
#define USART_ISR_CTS (1UL << 10) ///< CTS flag
#define USART_ISR_RTOF (1UL << 11) ///< Receiver timeout
#define USART_ISR_EOBF (1UL << 12) ///< End of block flag
#define USART_ISR_UDR (1UL << 13) ///< SPI slave underrun error flag
#define USART_ISR_ABRE (1UL << 14) ///< Auto baud rate error
#define USART_ISR_ABRF (1UL << 15) ///< Auto baud rate flag
#define USART_ISR_BUSY (1UL << 16) ///< Busy flag
#define USART_ISR_CMF (1UL << 17) ///< Character match flag
#define USART_ISR_SBKF (1UL << 18) ///< Send break flag
#define USART_ISR_RWU (1UL << 19) ///< Receiver wake up from mute mode
#define USART_ISR_WUF (1UL << 20) ///< Wake up from low power mode flag
#define USART_ISR_TEACK (1UL << 21) ///< Transmit enable ack flag
#define USART_ISR_REACK (1UL << 22) ///< Receive enable ack flag
#define USART_ISR_TXFE (1UL << 23) ///< TXFIFO empty
#define USART_ISR_RXFF (1UL << 24) ///< RXFIFO full
#define USART_ISR_TCBGT (1UL << 25) ///< Transmission complete before guard time flag
#define USART_ISR_RXFT (1UL << 26) ///< RXFIFO threshold flag
#define USART_ISR_TXFT (1UL << 27) ///< TXFIFO threshold flag

#define USART_ICR_PECF (1UL << 0) ///< Parity error clear flag
#define USART_ICR_FECF (1UL << 1) ///< Framing error clear flag
#define USART_ICR_NECF (1UL << 2) ///< Noise detected clear flag
#define USART_ICR_ORECF (1UL << 3) ///< Overrun error clear flag
#define USART_ICR_IDLECF (1UL << 4) ///< Idle line detected clear flag
#define USART_ICR_TXFECF (1UL << 5) ///< TXFIFO empty clear flag
#define USART_ICR_TCCF (1UL << 6) ///< Transmission complete clear flag
#define USART_ICR_TCBGTCF (1UL << 7) ///< Transmission complete before guard time clear flag
#define USART_ICR_LBDCF (1UL << 8) ///< LIN break detection clear flag
#define USART_ICR_CTSCF (1UL << 9) ///< CTS clear flag
#define USART_ICR_RTOCF (1UL << 11) ///< Receiver timeout clear flag
#define USART_ICR_EOBCF (1UL << 12) ///< End of block clear flag
#define USART_ICR_UDRCF (1UL << 13) ///< SPI slave underrun clear flag
#define USART_ICR_CMCF (1UL << 17) ///< Character match clear flag
#define USART_ICR_WUCF (1UL << 20) ///< Wake up from low power mode clear flag

#define USART_RDR (0x1FFUL << 0) ///< Receive data value
#define USART_TDR (0x1FFUL << 0) ///< Transmit data value

#define USART_PRESC (0xFUL << 0) ///< Clock prescaler

// ----------------
// Type definitions
// ----------------
/**
 * @brief USART status codes
 */
typedef enum {
    UART_OK = 0,                     ///< Operation completed successfully
    UART_ERROR_INVALID_HANDLE,       ///< Handle pointer is NULL
    UART_ERROR_INVALID_PARAM,        ///< Invalid parameter passed
    UART_ERROR_NOT_INITIALIZED,      ///< Handle not initialized
    UART_ERROR_ALREADY_INITIALIZED,  ///< Handle already initialized
    UART_ERROR_BUSY,                 ///< UART peripheral is busy
    UART_ERROR_TIMEOUT,              ///< Operation timed out
    UART_ERROR_HARDWARE_FAULT,       ///< Hardware error detected
    UART_ERROR_UNSUPPORTED_INSTANCE, ///< Invalid UART instance
    UART_ERROR_NO_AVAILABLE_HANDLES, ///< Maximum handles exceeded
    UART_ERROR_BUFFER_FULL,          ///< Buffer is full
    UART_ERROR_NO_DATA               ///< No data available
} uart_status_t;

/**
 * @brief USART Parity settings
 */
typedef enum {
    UART_PARITY_NONE = 0,
    UART_PARITY_EVEN,
    UART_PARITY_ODD
} uart_parity_t;

/**
 * @brief UART stop bit configuration
 */
typedef enum {
    UART_STOPBITS_1 = 0, ///< 1 stop bit
    UART_STOPBITS_0_5, ///< 0.5 stop bits
    UART_STOPBITS_2, ///< 2 stop bits
    UART_STOPBITS_1_5 ///< 1.5 stop bits
} uart_stopbits_t;

/**
 * @brief UART word length configuration
 */
typedef enum {
    UART_WORDLENGTH_7B = 0, ///< 7 bits
    UART_WORDLENGTH_8B, ///< 8 bits
    UART_WORDLENGTH_9B ///< 9 bits
} uart_wordlength_t;

/**
 * @brief UART flow control configuration
 */
typedef enum {
    UART_FLOWCONTROL_NONE = 0,    ///< No flow control
    UART_FLOWCONTROL_RTS,     ///< RTS flow control
    UART_FLOWCONTROL_CTS,     ///< CTS flow control
    UART_FLOWCONTROL_RTS_CTS  ///< RTS and CTS flow control
} uart_flowcontrol_t;

/**
 * @brief UART configuration structure
 */
typedef struct {
    uint32_t baudrate; ///< Baud rate
    uart_wordlength_t word_length; ///< Word length
    uart_stopbits_t stop_bits; ///< Stop bits
    uart_parity_t parity; ///< Parity
    uart_flowcontrol_t flow_control; ///< Flow control
    bool oversampling_8; ///< Oversampling
    bool enable_rx; ///< Enable receiver
    bool enable_tx; ///< Enable transmitter
} uart_config_t;

/**
 * @brief UART handle structure
 */
typedef struct {
    USART_TypeDef* hardware; ///< Pointer to UART hardware register
    uart_config_t config; ///< Config struct
    bool is_initialized; ///< Initialization status
    bool tx_busy; ///< Transmit busy flag
    bool rx_busy; ///< Receive busy flag
    volatile uint32_t* rcc_enable_reg; ///< RCC enable register
    uint32_t rcc_enable_bit; ///< RCC enable bit mask
    uint8_t* tx_buffer; ///< TX buffer pointer
    uint16_t tx_buffer_size; ///< TX buffer size
    volatile uint16_t tx_head; ///< TX buffer head
    volatile uint16_t tx_tail; ///< TX buffer tail
    uint8_t* rx_buffer; ///< RX buffer pointer
    uint16_t rx_buffer_size; ///< RX buffer size
    volatile uint16_t rx_head; ///< RX buffer head
    volatile uint16_t rx_tail; ///< RX buffer tail
} uart_handle_t;

// -------------------
// Default Definitions
// -------------------
#define UART_MAX_HANDLES 8               ///< Maximum UART handles
#define UART_DEFAULT_TIMEOUT_MS 1000     ///< Default timeout
#define UART_DEFAULT_TX_BUFFER_SIZE 256  ///< Default TX buffer size
#define UART_DEFAULT_RX_BUFFER_SIZE 256  ///< Default RX buffer size

// ---------------------
// Function Declarations
// ---------------------
/**
 * @brief Initialize UART handle and configure hardware
 *
 * @param handle Pointer to USART handle
 * @param hardware Pointer to USART hardware registers
 * @param config Pointer to config struct
 *
 * @return returns UART_OK on success, error code otherwise.
 */
uart_status_t uart_init(uart_handle_t* handle, USART_TypeDef* hardware, const uart_config_t* config);

/**
 * @brief Initialize UART with buffers for interrupt operation
 *
 * @param handle Pointer to USART handle
 * @param hardware Pointer to USART hardware registers
 * @param config Pointer to config struct
 * @param tx_buffer Transmission "Queue"
 * @param tx_size Size of buffer
 * @param rx_buffer Receptions "Queue"
 * @param rx_size Size of buffer
 *
 * @return returns UART_OK on success, error code otherwise.
 */
uart_status_t uart_init_with_buffers(uart_handle_t* handle, USART_TypeDef* hardware, const uart_config_t* config, uint8_t* tx_buffer, uint16_t tx_size, uint8_t* rx_buffer, uint16_t rx_size);

/**
 * @brief Deinitialize UART handle
 *
 * @param handle Pointer to UART handle to deinitialize
 *
 * @return returns UART_OK on success, error code otherwise.
 */
uart_status_t uart_deinit(uart_handle_t* handle);

/**
 * @brief Send a single byte (blocking)
 */
uart_status_t uart_send_byte(uart_handle_t* handle, uint8_t data);

/**
 * @brief Receive a single byte (blocking)
 */
uart_status_t uart_receive_byte(uart_handle_t* handle, uint8_t* data);

/**
 * @brief Send buffer (blocking)
 */
uart_status_t uart_send_buffer(uart_handle_t* handle, const uint8_t* buffer, uint16_t length);

/**
 * @brief Receive buffer (blocking)
 */
uart_status_t uart_receive_buffer(uart_handle_t* handle, uint8_t* buffer, uint16_t length);

/**
 * @brief Send string (blocking)
 */
uart_status_t uart_send_string(uart_handle_t* handle, const char* str);

/**
 * @brief Send formatted string (printf-style)
 */
uart_status_t uart_printf(uart_handle_t* handle, const char* format, ...);

/**
 * @brief Check if data is available to receive
 */
bool uart_data_available(uart_handle_t* handle);

/**
 * @brief Get current UART status
 */
uart_status_t uart_get_status(uart_handle_t* handle, bool* tx_busy, bool* rx_busy);

/**
 * @brief Create default UART configuration
 */
uart_status_t uart_get_default_config(uart_config_t* config);

/**
 * @brief Enable/disable UART interrupts
 */
uart_status_t uart_enable_interrupts(uart_handle_t* handle, bool enable_tx, bool enable_rx);

/**
 * @brief UART interrupt handler
 */
void uart_irq_handler(uart_handle_t* handle);


#endif //TIELVE_STM32H743_UART_H
