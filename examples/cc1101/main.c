/*
 * Copyright (C) 2014 Freie Universität Berlin
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
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

//#include <stdio.h>
#include "board.h"
#include "vtimer.h"
#include "uart.h"
#include "gpio.h"
#include "cc110x-interface.h"
#include "spi.h"

#define CC1100_VERSION          (0x31 | 0xc0)


int main(void)
{
	printf("El RCC-CFGR es: %x\n", RCC->CFGR);
    printf("El RCC-CR es: %x\n", RCC->CR);
    printf("El RCC-CSR es: %x\n", RCC->CSR);
    printf("El RCC-AHBENR es: %x\n", RCC->AHBENR);

	//kernel_pid_t pid=KERNEL_PID_UNDEF;
    //cc110x_init(pid);
    //cc110x_print_config();
    /*Active closck GPIO*/
RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;




	uint8_t tmp;

	/* CS */
	gpio_init_out(AT86RF231_CS, GPIO_NOPULL);
    gpio_set(AT86RF231_CS);



    /*Init SPI*/
    spi_init_master(AT86RF231_SPI, SPI_CONF_FIRST_RISING, SPI_SPEED_10MHZ);

    /*Set the chip select*/

    gpio_clear(AT86RF231_CS);


    /*Read serial number at86rf231*/
    spi_transfer_reg(AT86RF231_SPI, AT86RF231_ACCESS_REG | AT86RF231_ACCESS_READ | AT86RF231_REG__VERSION_NUM, 0 ,&tmp );

    /*Read serial number cc1101*/
  //  spi_transfer_reg(AT86RF231_SPI, CC1101_version, 0 ,&tmp );

    /*See whats in there*/
    printf("tmp: %x\n", tmp);


    /*Block the chip select*/
    gpio_set(AT86RF231_CS);


    return 0;
}
