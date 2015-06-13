/*
 * Copyright (C) 2014 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    board_vesna VESNA Board
 * @ingroup     boards
 * @brief       Board specific files for the VESNA board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the VESNA board.
 *
 * @author      Victor Ariño <victor.arino@triagnosys.com>
 */

#ifndef BOARD_H_
#define BOARD_H_

#include <stdint.h>

#include "cpu.h"
#include "periph_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name Define the nominal CPU core clock in this board
 */
#define F_CPU               CLOCK_CORECLOCK

/**
 * @name Define the UART to be used as stdio and its baudrate
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
#define AT86RF231_CS        GPIO_11
#define AT86RF231_INT       GPIO_12 // xxx probably PC9
#define AT86RF231_RESET     GPIO_13
#define AT86RF231_SLEEP     GPIO_14


//#define AT86RF231_CHANNEL   15
/** @} */

/**
 * @name Define the interface to the CC1101 radio
 * @{
 */
//#define CC1100_SPI       SPI_0
//#define RX_BUF_SIZE      300
//#define CC1100_CS        GPIO_11 // xxx
//#define CC1100_INT       GPIO_12 // xxx probably PC9
//#define CC1100_RESET
//#define CC1100_SLEEP
/** @} */


/**
 * @name LED pin definitions
 * @{
 */
#define LED_GREEN_PORT      (GPIOB)
#define LED_GREEN_PIN       (2)
/** @} */

/**
 * dummy LED macros
 */
#define LED_RED_ON              
#define LED_RED_OFF             
#define LED_RED_TOGGLE          

#define LED_GREEN_TOGGLE        

#define LED_ORANGE_ON              
#define LED_ORANGE_OFF             
#define LED_ORANGE_TOGGLE          


/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */

#define LED_GREEN_ON        (LED_GREEN_PORT->BSRR = (1<<LED_GREEN_PIN))
#define LED_GREEN_OFF       (LED_GREEN_PORT->BRR = (1<<LED_GREEN_PIN))
/** @} */

/**
 * software/fake transceiver interrupt macro 
 * here using line 13
 */
#define TRX_INT()                   EXTI->SWIER |= EXTI_SWIER_SWIER13;

/**
 * serial-line interrupt macros
 */
#define UART_0_ENABLE_RXINTERRUPT   UART_0_DEV->CR1 |= USART_CR1_RXNEIE
#define UART_0_ENABLE_TXINTERRUPT   UART_0_DEV->CR1 |= USART_CR1_TXEIE
#define UART_0_DISABLE_RXINTERRUPT  UART_0_DEV->CR1 &= ~USART_CR1_RXNEIE
#define UART_0_DISABLE_TXINTERRUPT  UART_0_DEV->CR1 &= ~USART_CR1_TXEIE

#define UART_0_CLEAR_RXFLAG         UART_0_DEV->SR  &= ~USART_SR_RXNE
#define UART_0_CLEAR_TXFLAG         UART_0_DEV->SR  &= ~USART_SR_TXE

// UART busy := TXE == 0 after software writes to USART_DR while TXEIE == 1
#define UART_0_TXBUSY               ~(UART_0_DEV->SR & USART_SR_TXE)

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

#endif /* BOARD_H_ */
/** @} */
