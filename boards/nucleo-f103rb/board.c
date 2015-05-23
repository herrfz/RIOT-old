/*
 * Copyright (C) 2015 Eriza Fazli 
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     boards_nucleo_f103rb
 * @{
 *
 * @file        board.c
 * @brief       Board specific implementations for the nucleo-f103rb board
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
 *
 * @}
 */

#include "board.h"
#include "cpu.h"

static void leds_init(void);

void board_init(void)
{
    /* initialize the CPU */
    cpu_init();

    /* initialize the boards LEDs */
    leds_init();
}

/**
 * @brief Initialize the boards on-board LEDs
 *
 * The LEDs initialization is hard-coded in this function. As the LED is soldered
 * onto the board it is fixed to its CPU pins.
 *
 * The LEDs are connected to the following pin:
 * - Green:     PA5
 */
static void leds_init(void)
{
    /* green led */
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    LED_GREEN_PORT->CRL = (0x3 << (LED_GREEN_PIN*4));
}
