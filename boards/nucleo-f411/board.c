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
 * @brief       Board specific implementations for the STM32F411Nucleo evaluation board
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
 *
 * @}
 */

#include "board.h"

static void leds_init(void);

void board_init(void)
{
    /* initialize the boards LEDs, this is done first for debugging purposes */
    leds_init();

    /* initialize the CPU */
    cpu_init();
}

/**
 * @brief Initialize the on-board LED (LD2)
 *
 * The LED initialization is hard-coded in this function. As the LEDs are soldered
 * onto the board they are fixed to their CPU pins.
 */
static void leds_init(void)
{
    /* enable clock for port GPIOA */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* configure pins as general outputs; ref. manual p. 153 */
    LED_GREEN_PORT->MODER &= ~(0x00000c00); // reset
    LED_GREEN_PORT->MODER |= 0x00000400; 
    /* set output speed high-speed; ref.manual p. 155 */
    LED_GREEN_PORT->OSPEEDR |= 0x00000c00;
    /* set output type to push-pull; ref. manual p. 154 */
    LED_GREEN_PORT->OTYPER &= ~(0x0020);
    /* disable pull resistors; ref. manual p. 156 */
    LED_GREEN_PORT->PUPDR &= ~(0x00000c00);

    /* turn all LEDs off */
    //LED_GREEN_PORT->BSRRH = 0xf000;
    LED_GREEN_OFF;
}
