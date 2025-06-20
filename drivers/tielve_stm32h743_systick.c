//
// Created by Jonathan Tielve
//
#include "tielve_stm32h743_systick.h"

static volatile uint64_t system_tick_count = 0; ///< Tick counter
static volatile bool systick_initialized = false; ///< Initialization flag

bool systick_init(void) {
    if (SYSTICK_RELOAD_VALUE > SYSTICK_MAX_RELOAD || SYSTICK_RELOAD_VALUE == 0) {
        return false;  // Invalid configuration
    }

    SysTick->CTRL = 0;

    // Set reload value
    SysTick->LOAD = SYSTICK_RELOAD_VALUE - 1;

    // Clear current value register
    SysTick->VAL = 0;

    // Configure and enable SysTick
    SysTick->CTRL = SYSTICK_CTRL_ENABLE |     // Enable counter
                    SYSTICK_CTRL_TICKINT |    // Enable interrupt
                    SYSTICK_CTRL_CLKSOURCE;   // Use CPU clock

    // Reset tick counter
    system_tick_count = 0;
    systick_initialized = true;
    return true;
}

uint64_t get_system_tick(void) {
    if (!systick_initialized) {
        return 0;
    }

    uint64_t tick1, tick2;

    do {
        tick1 = system_tick_count;
        tick2 = system_tick_count;
    } while (tick1 != tick2);  // Retry if interrupt occurred during read

    return tick1;
}

uint64_t get_system_tick_us(void) {
    if (!systick_initialized) {
        return 0;
    }

    // Get current count
    uint64_t ms_ticks = get_system_tick();

    // Get fractional part from SysTick counter
    // SysTick counts DOWN from RELOAD to 0
    uint32_t elapsed_cycles = SYSTICK_RELOAD_VALUE - SysTick->VAL;

    // Convert cycles to microseconds
    uint32_t us_fraction = (elapsed_cycles * 1000) / SYSTICK_RELOAD_VALUE;

    return (ms_ticks * 1000) + us_fraction;
}

bool systick_timeout_elapsed(uint64_t start_tick, uint32_t timeout_ms) {
    uint64_t current_tick = get_system_tick();
    return (current_tick - start_tick) >= timeout_ms;
}

void delay_ms(uint32_t delay_ms) {
    if (!systick_initialized) {
        return;
    }

    uint64_t start_tick = get_system_tick();
    while (!systick_timeout_elapsed(start_tick, delay_ms)) {
        __asm("nop");
    }
}

/// Default ARM interrupt handler
void SysTick_Handler(void) {
    ++system_tick_count;
}