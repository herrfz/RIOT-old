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
#include "board.h"
#include "periph/spi.h"
#include "periph/gpio.h"

void dummyradio_reg_write(uint8_t addr, uint8_t value)
{
    printf("dummyradio_reg_write: %d\n", value);
}

uint8_t dummyradio_reg_read(uint8_t addr)
{
    printf("dummyradio_reg_read\n");
    return 0;
}

void dummyradio_read_fifo(uint8_t *data, radio_packet_length_t length)
{
    memset(data, 0, length);
    printf("dummyradio_read_fifo: %d\n", length);
}

void dummyradio_write_fifo(const uint8_t *data, radio_packet_length_t length)
{
    printf("dummyradio_write_fifo: %d\n", length);
}

uint8_t dummyradio_get_status(void)
{
    printf("dummyradio_get_status\n");
    return 0;
}
