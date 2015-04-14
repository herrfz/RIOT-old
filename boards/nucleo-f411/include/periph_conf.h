/*
 * Copyright (C) 2014 Eriza Fazli
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_stm32f411nucleo
 * @{
 *
 * @file
 * @name       Peripheral MCU configuration for the STM32F411Nucleo board
 *
 * @author     Eriza Fazli <erizzaaaaa@gmail.com>
 */

#ifndef __PERIPH_CONF_H
#define __PERIPH_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Clock system configuration
 * @{
 */
#define CLOCK_HSE           (8000000U)          /* external oscillator */
#define CLOCK_CORECLOCK     (100000000U)        /* desired core clock frequency */

/* the actual PLL values are automatically generated */
#define CLOCK_PLL_M         (CLOCK_HSE / 1000000)
#define CLOCK_PLL_N         ((CLOCK_CORECLOCK / 1000000) * 2)
#define CLOCK_PLL_P         (2U)
#define CLOCK_PLL_Q         (CLOCK_PLL_N / 48)
#define CLOCK_AHB_DIV       RCC_CFGR_HPRE_DIV1
#define CLOCK_APB2_DIV      RCC_CFGR_PPRE2_DIV1
#define CLOCK_APB1_DIV      RCC_CFGR_PPRE1_DIV2
#define CLOCK_FLASH_LATENCY FLASH_ACR_LATENCY_5WS
/** @} */

/**
 * @name Timer configuration
 * @{
 */
#define TIMER_NUMOF         (2U)
#define TIMER_0_EN          1
#define TIMER_1_EN          1
#define TIMER_IRQ_PRIO      1

/* Timer 0 configuration */
#define TIMER_0_DEV         TIM2
#define TIMER_0_CHANNELS    4
#define TIMER_0_PRESCALER   (99U) /* 1 MHz for hwtimer, vtimer */
#define TIMER_0_MAX_VALUE   (0xffffffff)
#define TIMER_0_CLKEN()     (RCC->APB1ENR |= RCC_APB1ENR_TIM2EN)
#define TIMER_0_ISR         isr_tim2
#define TIMER_0_IRQ_CHAN    TIM2_IRQn

/* Timer 1 configuration */
#define TIMER_1_DEV         TIM5
#define TIMER_1_CHANNELS    4
#define TIMER_1_PRESCALER   (99U)
#define TIMER_1_MAX_VALUE   (0xffffffff)
#define TIMER_1_CLKEN()     (RCC->APB1ENR |= RCC_APB1ENR_TIM5EN)
#define TIMER_1_ISR         isr_tim5
#define TIMER_1_IRQ_CHAN    TIM5_IRQn
/** @} */

/**
 * @name UART configuration
 * @{
 */
#define UART_NUMOF          (1U)
#define UART_0_EN           1
#define UART_IRQ_PRIO       1
#define UART_CLK            (50000000U) 

/* UART 0 device configuration */
#define UART_0_DEV          USART2
#define UART_0_CLKEN()      (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define UART_0_CLKDIS()     (RCC->APB1ENR &= ~(RCC_APB1ENR_USART2EN))
#define UART_0_CLK          (50000000U)
#define UART_0_IRQ_CHAN     USART2_IRQn
#define UART_0_ISR          isr_usart2
/* UART 0 pin configuration */
#define UART_0_PORT_CLKEN() (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define UART_0_PORT         GPIOA
#define UART_0_TX_PIN       2
#define UART_0_RX_PIN       3
#define UART_0_AF           7
/** @} */

/**
 * @name SPI configuration
 * @{
 */
#define SPI_NUMOF                   (1U)
#define SPI_0_EN                    1
#define SPI_IRQ_PRIO                1

/* SPI 0 device config */
#define SPI_0_DEV                   SPI1
#define SPI_0_CLKEN()               (RCC->APB2ENR |= RCC_APB2ENR_SPI1EN)
#define SPI_0_CLKDIS()              (RCC->APB2ENR &= ~RCC_APB2ENR_SPI1EN)
#define SPI_0_BUS_DIV               1   /* 1 -> SPI runs with half CPU clock, 0 -> quarter CPU clock */
#define SPI_0_IRQ                   SPI1_IRQn
#define SPI_0_IRQ_HANDLER           isr_spi1
/* SPI 0 pin configuration */
#define SPI_0_SCK_PORT              GPIOA
#define SPI_0_SCK_PIN               5
#define SPI_0_SCK_AF                5 /* alternate function SPI */
#define SPI_0_SCK_PORT_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define SPI_0_MISO_PORT             GPIOA
#define SPI_0_MISO_PIN              6
#define SPI_0_MISO_AF               5
#define SPI_0_MISO_PORT_CLKEN()     (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define SPI_0_MOSI_PORT             GPIOA
#define SPI_0_MOSI_PIN              7
#define SPI_0_MOSI_AF               5
#define SPI_0_MOSI_PORT_CLKEN()     (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)

/**
 * @name GPIO configuration
 * @{
 */
#define GPIO_NUMOF          5
#define GPIO_0_EN           1
#define GPIO_1_EN           1
#define GPIO_2_EN           1
#define GPIO_3_EN           1
#define GPIO_4_EN           1
#define GPIO_IRQ_PRIO       1

/* IRQ config */
#define GPIO_IRQ_0          -1
#define GPIO_IRQ_1          -1
#define GPIO_IRQ_2          -1
#define GPIO_IRQ_3          -1
#define GPIO_IRQ_4          -1
#define GPIO_IRQ_5          GPIO_4
#define GPIO_IRQ_6          GPIO_1
#define GPIO_IRQ_7          -1/* not defined */
#define GPIO_IRQ_8          GPIO_3
#define GPIO_IRQ_9          GPIO_2
#define GPIO_IRQ_10         -1
#define GPIO_IRQ_11         -1/* not defined */
#define GPIO_IRQ_12         -1/* not defined */
#define GPIO_IRQ_13         GPIO_0 /* GPIO_IRQ_x -> GPIO_0_PIN pin x */
#define GPIO_IRQ_14         -1/* not defined */
#define GPIO_IRQ_15         -1/* not defined */

/* GPIO channel 0: User button; in */
#define GPIO_0_PORT         GPIOC
#define GPIO_0_PIN          13
#define GPIO_0_CLK          2 /* 0: PORT A, 1: B ... */
#define GPIO_0_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN)
#define GPIO_0_EXTI_CFG()   (SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC) // ref. manual p. 139
#define GPIO_0_IRQ          EXTI15_10_IRQn // http://stm32f4-discovery.com/2014/08/stm32f4-external-interrupts-tutorial/

/* GPIO channel 1: SPI_CS; AT86RF231_CS; as master: out */
#define GPIO_1_PORT         GPIOB
#define GPIO_1_PIN          6
#define GPIO_1_CLK          1
#define GPIO_1_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define GPIO_1_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PB)
#define GPIO_1_IRQ          EXTI9_5_IRQn

/* GPIO channel 2: pin D8 of Arduino connector; AT86RF231_INT; in */
#define GPIO_2_PORT         GPIOA
#define GPIO_2_PIN          9
#define GPIO_2_CLK          0
#define GPIO_2_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define GPIO_2_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI9_PA)
#define GPIO_2_IRQ          EXTI9_5_IRQn

/* GPIO channel 3: pin D7 of Arduino connector; AT86RF231_RESET; out */
#define GPIO_3_PORT         GPIOA
#define GPIO_3_PIN          8
#define GPIO_3_CLK          0
#define GPIO_3_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define GPIO_3_EXTI_CFG()   (SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PA)
#define GPIO_3_IRQ          EXTI9_5_IRQn

/* GPIO channel 4: pin D4 of Arduino connector; AT86RF231_SLEEP; out */
#define GPIO_4_PORT         GPIOB
#define GPIO_4_PIN          5
#define GPIO_4_CLK          1
#define GPIO_4_CLKEN()      (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define GPIO_4_EXTI_CFG()   (SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PB)
#define GPIO_4_IRQ          EXTI9_5_IRQn
/** @} */

#ifdef __cplusplus
}
#endif

#endif /* __PERIPH_CONF_H */
/** @} */
