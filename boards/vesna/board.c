/*
 * Copyright (C) 2014 TriaGnoSys GmbH
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     board_vesna
 * @{
 *
 * @file        board.c
 * @brief       Board specific implementations for the VESMA board
 *
 * @author      Víctor Ariño <victor.arino@triagnosys.com>
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

    /* initialize the UART */
    uart_init(STDIO, STDIO_BAUDRATE, NULL, NULL, NULL);

    /* forced delay */
    volatile i;
    for(i=0; i< 10000; i++);
}

/**
 * @brief Initialize the boards on-board LEDs
 *
 * The LEDs initialization is hard-coded in this function. As the LED is soldered
 * onto the board it is fixed to its CPU pins.
 *
 * The LEDs are connected to the following pin:
 * - Green:     PB5
 * - Orange:    PC10
 * - Red:       PD2
 */
static void leds_init(void)
{
    /* green pin */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    LED_GREEN_PORT->CRL = (0x3 << (LED_GREEN_PIN*4));
}
