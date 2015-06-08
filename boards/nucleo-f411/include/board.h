/*
 * Copyright (C) 2014 Eriza Fazli
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_stm32f411nucleo STM32F411Nucleo
 * @ingroup     boards
 * @brief       Board specific files for the STM32F411Nucleo board
 * @{
 *
 * @file
 * @brief       Board specific definitions for the STM32F411Nucleo evaluation board
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
 */

#ifndef __BOARD_H
#define __BOARD_H

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Define the nominal CPU core clock in this board
 */
#define F_CPU               CLOCK_CORECLOCK

/**
 * @name Define UART device and baudrate for stdio
 * @{
 */
#define STDIO               UART_0
#define STDIO_BAUDRATE      (115200U)
#define STDIO_RX_BUFSIZE    (64U)
/** @} */


/**
 * @name Assign the hardware timer
 * @{
 */
#define HW_TIMER            TIMER_0
/** @} */

/**
 * @name Define the interface to the AT86RF231 radio
 * @{
 */
#define AT86RF231_SPI       SPI_0
#define AT86RF231_CS        GPIO_1
#define AT86RF231_INT       GPIO_2
#define AT86RF231_RESET     GPIO_3
#define AT86RF231_SLEEP     GPIO_4
/** @} */

/**
 * @name LED pin definitions
 * @{
 */
#define LED_GREEN_PORT            GPIOA
#define LED_GREEN_PIN             (1 << 5)
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED_GREEN_ON        (LED_GREEN_PORT->BSRRL = LED_GREEN_PIN)
#define LED_GREEN_OFF       (LED_GREEN_PORT->BSRRH = LED_GREEN_PIN)
#define LED_GREEN_TOGGLE    (LED_GREEN_PORT->ODR ^= LED_GREEN_PIN)

#define LED_RED_ON
#define LED_RED_OFF
#define LED_RED_TOGGLE

#define LED_ORANGE_ON
#define LED_ORANGE_OFF
#define LED_ORANGE_TOGGLE
/** @} */

/**
 * software/fake transceiver interrupt macro 
 * here using line 13 (PC13 user button) 
 */
#define TRX_INT()                   EXTI->SWIER |= EXTI_SWIER_SWIER13;

/**
 * serial-line interrupt macros
 */
#define RXNEIE_MASK                 (1 << 5) // RXNEIE is bit 5 of USART CR1 and SR egister
#define TXEIE_MASK                  (1 << 7) // TXEIE is bit 7 of USART CR1 and SR register
#define USART2_ENABLE_RXINTERRUPT   UART_0_DEV->CR1 |= RXNEIE_MASK
#define USART2_ENABLE_TXINTERRUPT   UART_0_DEV->CR1 |= TXEIE_MASK
#define USART2_DISABLE_RXINTERRUPT  UART_0_DEV->CR1 &= ~RXNEIE_MASK
#define USART2_DISABLE_TXINTERRUPT  UART_0_DEV->CR1 &= ~TXEIE_MASK

#define USART2_CLEAR_RXFLAG         UART_0_DEV->SR  &= ~RXNEIE_MASK
#define USART2_CLEAR_TXFLAG         UART_0_DEV->SR  &= ~TXEIE_MASK

/**
 * Define the type for the radio packet length for the transceiver
 */
typedef uint8_t radio_packet_length_t;

/**
 * @brief Initialize board specific hardware, including clock, LEDs and std-IO
 */
void board_init(void);

#ifdef __cplusplus
}
#endif

#endif /** __BOARD_H */
/** @} */
