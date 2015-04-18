/*
 * Copyright (C) 2014 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_dummyradio
 * @{
 *
 * @file
 * @brief       Register access functions for the DUMMYRADIO device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 * @author      Joakim Gebart <joakim.gebart@eistec.se>
 *
 * @}
 */

#include <stdio.h>
#include <string.h>

#include "dummyradio_spi.h"
#include "dummyradio.h"

/*                           len   fcf         seq   dstpan      dstaddr     srcpan      srcaddr     payload     lqi  */
static uint8_t fifo_reg[] = {0x0e, 0x01, 0x88, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1c, 0xaa, 0x00, 0x00, 0xca, 0xfe, 0x01};

void dummyradio_reg_write(uint8_t addr, uint8_t value)
{
    printf("dummyradio_reg_write: %02x\n", value);
}

uint8_t dummyradio_reg_read(uint8_t addr)
{
    puts("dummyradio_reg_read\n");
    return 0;
}

void dummyradio_read_fifo(uint8_t *data, radio_packet_length_t length)
{
    memcpy(data, fifo_reg, length);
    printf("dummyradio_read_fifo: %d\n", length);
}

void dummyradio_write_fifo(const uint8_t *data, radio_packet_length_t length)
{
    puts("dummyradio_write_fifo \n");
    for (int i = 0; i < length; i++) {
        printf("%02x ", data[i]);
    }
    puts("\n");
}

uint8_t dummyradio_get_status(void)
{
    puts("dummyradio_get_status\n");
    return 0;
}
