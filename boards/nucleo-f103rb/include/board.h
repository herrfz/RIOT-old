/*
 * Copyright (C) 2015 Eriza Fazli
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @defgroup    boards_nucleo_f103rb
 * @ingroup     boards
 * @brief       Board specific files for the nucleo-f103rb board.
 * @{
 *
 * @file
 * @brief       Board specific definitions for the nucleo-f103rb board.
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
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
 * @name Define the UART to be used as stdio, its baudrate, and the size of
 *       receiving ringbuffer
 * @{
 */
#define STDIO               UART_1

#ifndef STDIO_BAUDRATE
#   define STDIO_BAUDRATE   (500000U)
#endif

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
//#define AT86RF231_SPI       SPI_0
//#define AT86RF231_CS        GPIO_11
//#define AT86RF231_INT       GPIO_12
//#define AT86RF231_RESET     GPIO_13
//#define AT86RF231_SLEEP     GPIO_14
/** @} */

/**
 * @name LED pin definitions
 * @{
 */
#define LED_RED_PORT        
#define LED_RED_PIN         
#define LED_GREEN_PORT      (GPIOA)
#define LED_GREEN_PIN       (5)
#define LED_ORANGE_PORT     
#define LED_ORANGE_PIN      
/** @} */

/**
 * @name Macros for controlling the on-board LEDs.
 * @{
 */
#define LED_RED_ON          
#define LED_RED_OFF         
#define LED_RED_TOGGLE      

#define LED_GREEN_ON        (LED_GREEN_PORT->ODR &= ~(1<<LED_GREEN_PIN))
#define LED_GREEN_OFF       (LED_GREEN_PORT->ODR |= (1<<LED_GREEN_PIN))
#define LED_GREEN_TOGGLE    (LED_GREEN_PORT->ODR ^= (1<<LED_GREEN_PIN))

#define LED_ORANGE_ON       
#define LED_ORANGE_OFF      
#define LED_ORANGE_TOGGLE   
/** @} */

/**
 * Define software/fake transceiver interrupt 
 * here using line 13 (PC13 user button) 
 */
#define TRX_INT()           EXTI->SWIER |= EXTI_SWIER_SWIER13;

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
