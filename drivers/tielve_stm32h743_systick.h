//
// Created by Jonathan Tielve
//

#ifndef TIELVE_STM32H743_SYSTICK_H
#define TIELVE_STM32H743_SYSTICK_H

#include <stdint.h>
#include <stdbool.h>

#define SYSTICK_BASE 0xE000E010UL // ARM Cortex-M systick address
#define CPU_FREQUENCY_HZ 480000000UL
#define SYSTICK_FREQUENCY_HZ 1000UL ///< 1khz or 1ms
#define SYSTICK_RELOAD_VALUE    (CPU_FREQUENCY_HZ / SYSTICK_FREQUENCY_HZ)
#define SysTick ((SysTick_Type*)SYSTICK_BASE)

typedef struct {
    volatile uint32_t CTRL;   ///< 0x00 Control and Status register
    volatile uint32_t LOAD;   ///< 0x04 Reload value register
    volatile uint32_t VAL;    ///< 0x08 Current value register
    volatile uint32_t CALIB;  ///< 0x0C Calibration value register
} SysTick_Type;

// Bit definitions
#define SYSTICK_CTRL_ENABLE (1UL << 0)   ///< Counter enable
#define SYSTICK_CTRL_TICKINT (1UL << 1)   ///< Enable interrupt
#define SYSTICK_CTRL_CLKSOURCE (1UL << 2)   ///< Clock source (1=CPU clock, 0=external)
#define SYSTICK_CTRL_COUNTFLAG (1UL << 16)  ///< Count flag (cleared on read)
#define SYSTICK_MAX_RELOAD 0x00FFFFFFUL ///< Max reload to restart counter

// --------------------
// Function Definitions
// --------------------
/**
 * @brief Initialize SysTick timer for 1ms interrupts
 *
 * Configures SysTick to generate interrupts every 1ms using the CPU clock.
 *
 * @return true on success, false if configuration is invalid
 *
 * @note This function enables interrupts - ensure interrupt vectors are configured
 */
bool systick_init(void);

/**
 * @brief Get current system tick count in milliseconds
 *
 * Returns the number of milliseconds since systick_init() was called.
 * Safe to call from interrupt and main contexts.
 *
 * @return Tick count in milliseconds
 */
uint64_t get_system_tick(void);

/**
 * @brief Get fractional tick count with microsecond resolution
 *
 * Returns current tick with sub-millisecond precision by reading
 * the SysTick counter value.
 *
 * @return Tick count in microseconds
 */
uint64_t get_system_tick_us(void);


/**
 * @brief Check if specified time has elapsed
 *
 * @param start_tick Tick value when timing period started
 * @param timeout_ms Timeout period in milliseconds
 * @return true if timeout has elapsed, false otherwise
 *
 */
bool systick_timeout_elapsed(uint64_t start_tick, uint32_t timeout_ms);

/**
 * @brief Precise delay using SysTick
 *
 * @param delay_ms Number of milliseconds to delay
 */
void delay_ms(uint32_t delay_ms);

#endif //TIELVE_STM32H743_SYSTICK_H
