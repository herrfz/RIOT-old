/*
 * Copyright (C) 2014 TriaGnoSys GmbH 
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Blinking Hello World
 *
 * @author      Xavi Perez <xavi@perez.de>
 *
 * @}
 */

#include <stdio.h>
#include "board.h"
#include "vtimer.h"

#define MSEC (1000)
#define SEC (1000 * MSEC)

int main(void)
{
    timex_t now;

    while(1)
    {
        /* Test if leds are working */
        //LED_GREEN_ON;
        //vtimer_usleep(SEC);
        //LED_GREEN_OFF;
        //vtimer_usleep(SEC);

        LED_GREEN_TOGGLE;
        vtimer_usleep(SEC);

        /* Test UART */
        puts("Hello World!");
        printf("You are running RIOT on a(n) %s board.\n", RIOT_BOARD);
        printf("This board features a(n) %s MCU.\n", RIOT_MCU);

        /* TEST CURRENT TIME */
        vtimer_now(&now);
        printf("now is second: %lu\n", now.seconds);
        printf("now is microseconds: %lu\n", now.microseconds);
    }
    return 0;
}
