//
// Created by Jonathan Tielve
//

#ifndef TIELVE_STM32H743_RCC_H
#define TIELVE_STM32H743_RCC_H

// --------------------
// Register definitions
// --------------------
#define RCC_BASE 0x58024400UL ///< Base address for Reset and Clock Control

/// Peripheral Bus
#define RCC_APB1LENR (*(volatile uint32_t*)(RCC_BASE + 0x0E8)) ///< APB1 Low Clock
#define RCC_APB1HENR (*(volatile uint32_t*)(RCC_BASE + 0x0EC)) ///< APB2 High Clock
#define RCC_APB2ENR (*(volatile uint32_t*)(RCC_BASE + 0x0F0)) ///< APB2 Clock
#define RCC_APB3ENR (*(volatile uint32_t*)(RCC_BASE + 0x0E4)) ///< APB3 Clock
#define RCC_APB4ENR (*(volatile uint32_t*)(RCC_BASE + 0x0F4)) ///< APB4 Clock

/// High Power Bus
#define RCC_AHB1ENR (*(volatile uint32_t*)(RCC_BASE + 0x0D8)) ///< AHB1 Clock
#define RCC_AHB2ENR (*(volatile uint32_t*)(RCC_BASE + 0x0DC)) ///< AHB2 Clock
#define RCC_AHB3ENR (*(volatile uint32_t*)(RCC_BASE + 0x0D4)) ///< AHB3 Clock
#define RCC_AHB4ENR (*(volatile uint32_t*)(RCC_BASE + 0x0E0)) ///< AHB4 Clock

#define APB1_FREQUENCY_HZ 64000000UL ///< APB1 bus frequency
#define APB2_FREQUENCY_HZ 64000000UL ///< APB2 bus frequency
#define APB3_FREQUENCY_HZ 64000000UL ///< APB3 bus frequency
#define APB4_FREQUENCY_HZ 64000000UL ///< APB4 bus frequency

// ---------------
// Bit definitions
// ---------------
/// RCC APB1LENR bit definitions
#define RCC_APB1LENR_TIM2EN (1UL << 0) ///< TIM2 clock enable
#define RCC_APB1LENR_TIM3EN (1UL << 1) ///< TIM3 clock enable
#define RCC_APB1LENR_TIM4EN (1UL << 2) ///< TIM4 clock enable
#define RCC_APB1LENR_TIM5EN (1UL << 3) ///< TIM5 clock enable
#define RCC_APB1LENR_TIM6EN (1UL << 4) ///< TIM6 clock enable
#define RCC_APB1LENR_TIM7EN (1UL << 5) ///< TIM7 clock enable
#define RCC_APB1LENR_TIM12EN (1UL << 6) ///< TIM12 clock enable
#define RCC_APB1LENR_TIM13EN (1UL << 7) ///< TIM13 clock enable
#define RCC_APB1LENR_TIM14EN (1UL << 8) ///< TIM14 clock enable
#define RCC_APB1LENR_LPTIM1EN (1UL << 9) ///< LPTIM1 clock enable
#define RCC_APB1LENR_SPI2EN (1UL << 14) ///< SPI2 clock enable
#define RCC_APB1LENR_SPI3EN (1UL << 15) ///< SPI3 clock enable
#define RCC_APB1LENR_SPDIFRXEN (1UL << 16) ///< SPDIFRX clock enable
#define RCC_APB1LENR_USART2EN (1UL << 17) ///< USART2 clock enable
#define RCC_APB1LENR_USART3EN (1UL << 18) ///< USART3 clock enable
#define RCC_APB1LENR_UART4EN (1UL << 19) ///< UART4 clock enable
#define RCC_APB1LENR_UART5EN (1UL << 20) ///< UART5 clock enable
#define RCC_APB1LENR_I2C1EN (1UL << 21) ///< I2C1 clock enable
#define RCC_APB1LENR_I2C2EN (1UL << 22) ///< I2C2 clock enable
#define RCC_APB1LENR_I2C3EN (1UL << 23) ///< I2C3 clock enable
#define RCC_APB1LENR_CECEN (1UL << 27) ///< HDMI-CEC clock enable
#define RCC_APB1LENR_DAC12EN (1UL << 29) ///< DAC1 and 2 clock enable
#define RCC_APB1LENR_UART7EN (1UL << 30) ///< UART7 clock enable
#define RCC_APB1LENR_UART8EN (1UL << 31) ///< UART8 clock enable

/// RCC APB1HENR bit definitions
#define RCC_APB1HENR_CRSEN (1UL << 1) ///< Clock recovery system clock enable
#define RCC_APB1HENR_SWPEN (1UL << 2) ///< SWPMI clock enable
#define RCC_APB1HENR_OPAMPEN (1UL << 4) ///< OPAMP clock enable
#define RCC_APB1HENR_MDIOSEN (1UL << 5) ///< MDIOS clock enable
#define RCC_APB1HENR_FDCANEN (1UL << 8) ///< FDCAN clock enable

/// RCC APB2ENR bit definitions
#define RCC_APB2ENR_TIM1EN (1UL << 0) ///< TIM1 clock enable
#define RCC_APB2ENR_TIM8EN (1UL << 1) ///< TIM8 clock enable
#define RCC_APB2ENR_USART1EN (1UL << 4) ///< USART1 clock enable
#define RCC_APB2ENR_USART6EN (1UL << 5) ///< USART6 clock enable
#define RCC_APB2ENR_SPI1EN (1UL << 12) ///< SPI1 clock enable
#define RCC_APB2ENR_SPI4EN (1UL << 13) ///< SPI4 clock enable
#define RCC_APB2ENR_TIM15EN (1UL << 16) ///< TIM15 clock enable
#define RCC_APB2ENR_TIM16EN (1UL << 17) ///< TIM15 clock enable
#define RCC_APB2ENR_TIM17EN (1UL << 18) ///< TIM15 clock enable
#define RCC_APB2ENR_SPI5EN (1UL << 20) ///< SPI5 clock enable
#define RCC_APB2ENR_SAI1EN (1UL << 22) ///< SAI1 clock enable
#define RCC_APB2ENR_SAI2EN (1UL << 23) ///< SAI2 clock enable
#define RCC_APB2ENR_SAI3EN (1UL << 24) ///< SAI3 clock enable
#define RCC_APB2ENR_DFSDM1EN (1UL << 28) ///< DFSDM1 clock enable
#define RCC_APB2ENR_HRTIMEN (1UL << 29) ///< HRTIM clock enable

/// RCC APB3ENR bit definitions
#define RCC_APB3ENR_LTDCEN (1UL << 3) ///< LTDC clock enable
#define RCC_APB3ENR_WWDG1EN (1UL << 6) ///< WWDG1 clock enable

/// RCC APB4ENR bit definitions
#define RCC_APB4ENR_SYSCFGEN (1UL << 1) ///< SYSCFG clock enable
#define RCC_APB4ENR_LPUART1EN (1UL << 3) ///< LPUART1 clock enable
#define RCC_APB4ENR_SPI6EN (1UL << 5) ///< SPI6 clock enable
#define RCC_APB4ENR_I2C4 (1UL << 7) ///< I2C4 clock enable
#define RCC_APB4ENR_LPTIM2EN (1UL << 9) ///< LPTIM2 clock enable
#define RCC_APB4ENR_LPTIM3EN (1UL << 10) ///< LPTIM3 clock enable
#define RCC_APB4ENR_LPTIM4EN (1UL << 11) ///< LPTIM4 clock enable
#define RCC_APB4ENR_LPTIM5EN (1UL << 12) ///< LPTIM5 clock enable
#define RCC_APB4ENR_COMP12EN (1UL << 14) ///< COMP1 and 2 clock enable
#define RCC_APB4ENR_VREFEN (1UL << 15) ///< VREFBUF clock enable
#define RCC_APB4ENR_RTCAPBEN (1UL << 16) ///< RTC APB clock enable
#define RCC_APB4ENR_SAI4EN (1UL << 21) ///< SAI4 clock enable

/// RCC AHB1ENR bit definitions
#define RCC_AHB1ENR_DMA1EN (1UL << 0) ///< DMA1 clock enable
#define RCC_AHB1ENR_DMA2EN (1UL << 1) ///< DMA2 clock enable
#define RCC_AHB1ENR_ADC12EN (1UL << 5) ///< ADC 1 and 2 clock enable
#define RCC_AHB1ENR_ETH1MACEN (1UL << 15) ///< ETH MAC bus clock enable
#define RCC_AHB1ENR_ETH1TXEN (1UL << 16) ///< ETH transmission clock enable
#define RCC_AHB1ENR_ETH1RXEN (1UL << 17) ///< ETH reception clock enable
#define RCC_AHB1ENR_USB1OTGHSEN (1UL << 25) ///< USB1OTG clock enable
#define RCC_AHB1ENR_USB1OTGHSULPIEN (1UL << 26) ///< USB_PHY1 clock enable
#define RCC_AHB1ENR_USB2OTGHSEN (1UL << 27) ///< USB2OTG clock enable
#define RCC_AHB1ENR_USB2OTGHSULPIEN (1UL << 28) ///< USB_PHY2 clock enable

/// RCC AHB2ENR bit definitions
#define RCC_AHB2ENR_DCMIEN (1UL << 0) ///< DCMI clock enable
#define RCC_AHB2ENR_CRYPTEN (1UL << 4) ///< CRYPT clock enable
#define RCC_AHB2ENR_HASHEN (1UL << 5) ///< HASH clock enable
#define RCC_AHB2ENR_RNGEN (1UL << 6) ///< RNG clock enable
#define RCC_AHB2ENR_SDMMC2EN (1UL << 9) ///< SDMMC2 clock enable
#define RCC_AHB2ENR_SRAM1EN (1UL << 29) ///< SRAM1 block enable
#define RCC_AHB2ENR_SRAM2EN (1UL << 30) ///< SRAM2 block enable
#define RCC_AHB2ENR_SRAM3EN (1UL << 31) ///< SRAM3 block enable

/// RCC AHB3ENR bit definitions
#define RCC_AHB3ENR_MDMAEN (1UL << 0) ///< MDMA clock enable
#define RCC_AHB3ENR_DMA2DEN (1UL << 4) ///< DMA2D clock enable
#define RCC_AHB3ENR_JPGDECEN (1UL << 5) ///< JPGDEC clock enable
#define RCC_AHB3ENR_FMCEN (1UL << 12) ///< FMC clock enable
#define RCC_AHB3ENR_QSPIEN (1UL << 14) ///< QUADSPI clock enable
#define RCC_AHB3ENR_SDMMC1EN (1UL << 16) ///< SDMMC1 clock enable

/// RCC AHB4ENR bit definitions
#define RCC_AHB4ENR_GPIOAEN (1UL << 0) ///< GPIO Port A clock enable
#define RCC_AHB4ENR_GPIOBEN (1UL << 1) ///< GPIO Port B clock enable
#define RCC_AHB4ENR_GPIOCEN (1UL << 2) ///< GPIO Port C clock enable
#define RCC_AHB4ENR_GPIODEN (1UL << 3) ///< GPIO Port D clock enable
#define RCC_AHB4ENR_GPIOEEN (1UL << 4) ///< GPIO Port E clock enable
#define RCC_AHB4ENR_GPIOFEN (1UL << 5) ///< GPIO Port F clock enable
#define RCC_AHB4ENR_GPIOGEN (1UL << 6) ///< GPIO Port G clock enable
#define RCC_AHB4ENR_GPIOHEN (1UL << 7) ///< GPIO Port H clock enable
#define RCC_AHB4ENR_GPIOIEN (1UL << 8) ///< GPIO Port I clock enable
#define RCC_AHB4ENR_GPIOJEN (1UL << 9) ///< GPIO Port J clock enable
#define RCC_AHB4ENR_GPIOKEN (1UL << 10) ///< GPIO Port K clock enable
#define RCC_AHB4ENR_CRCEN (1UL << 19) ///< CRC clock enable
#define RCC_AHB4ENR_BDMAEN (1UL << 21) ///< BDMA and DMAMUX2 clock enable
#define RCC_AHB4ENR_ADC3EN (1UL << 24) ///< ADC3 clock enable
#define RCC_AHB4ENR_HSEMEN (1UL << 25) ///< HSEM clock enable
#define RCC_AHB4ENR_BKPRAMEN (1UL << 28) ///< Backup RAM clock enable

#endif //TIELVE_STM32H743_RCC_H
