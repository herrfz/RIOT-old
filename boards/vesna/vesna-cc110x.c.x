/*
 * Copyright (C) 2008, Freie Universitaet Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @file
 * @ingroup     LPC2387
 * @brief       CC1100 LPC2387 dependend functions
 *
 * @author      Heiko Will <hwill@inf.fu-berlin.de>
 * @author      Thomas Hillebrandt <hillebra@inf.fu-berlin.de>
 * @version     $Revision: 1781 $
 *
 * @note        $Id: msba2-cc110x.c 1781 2010-01-26 13:39:36Z hillebra $
 */

#include <stdio.h>
#include <stddef.h>
/* core */
#include "irq.h"
/* board */
#include "include/board.h"
/* drivers  */
#include "cc110x_legacy.h"

#include "gpio.h"
#include "gpioint.h"
#include "spi.h"

int cc110x_get_gdo0(void)
{
	// only csma
    //return  CC1100_GDO0;
	return 0;
}

int cc110x_get_gdo1(void)
{
	// unused
    //return  CC1100_GDO1;
	return 0;
}

int cc110x_get_gdo2(void)
{
	// read register
    //return  CC1100_GDO2;
	return gpio_read(CC1100_INT);
}

void cc110x_spi_init(void)
{
	cc110x_spi_select();
	if(spi_init_master(CC1100_SPI, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ) != 0) {
		// error
	}
	spi_poweron(CC1100_SPI);
}

uint8_t cc110x_txrx(uint8_t c)
{
	char in;
	spi_transfer_byte(CC1100_SPI, c, &in);

	return (uint8_t)in;
}

void cc110x_spi_cs(void)
{
    gpio_init_out(CC1100_CS, GPIO_PULLUP);
}

void
cc110x_spi_select(void)
{
	gpio_init_out(CC1100_CS, GPIO_PULLUP);
}

void
cc110x_spi_unselect(void)
{
	gpio_init_out(CC1100_CS, GPIO_PULLDOWN);
}

void cc110x_before_send(void)
{
    // Disable GDO2 interrupt before sending packet
    cc110x_gdo2_disable();
}

void cc110x_after_send(void)
{
    // Enable GDO2 interrupt after sending packet
    cc110x_gdo2_enable();
}

void cc110x_gdo0_enable(void)
{
    // CCA
}

void cc110x_gdo0_disable(void)
{
    // CCA
}

void cc110x_gdo2_disable(void)
{
	gpio_irq_disable(CC1100_INT);
}

void cc110x_gdo2_enable(void)
{
	gpio_irq_enable(CC1100_INT);
}

void rx_synch_interrupt(void *arg) {
	cc110x_gdo2_irq();
}

void cc110x_init_interrupts(void)
{
	gpio_init_int(CC1100_INT, GPIO_NOPULL, GPIO_RISING, rx_synch_interrupt, NULL);
	cc110x_gdo2_enable();
}
