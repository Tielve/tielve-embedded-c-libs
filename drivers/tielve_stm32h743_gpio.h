//
// Created by Jonathan Tielve
//

#ifndef TIELVE_STM32H743_GPIO_H
#define TIELVE_STM32H743_GPIO_H

#include <stdint.h>
#include <stdbool.h>

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

#define RCC_AHB4ENR (*(volatile uint32_t*)(0x58024400UL + 0x0E0))

#endif //TIELVE_STM32H743_GPIO_H
