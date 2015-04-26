/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser General
 * Public License v2.1. See the file LICENSE in the top level directory for more
 * details.
 */

/**
 * @ingroup     drivers_dummyradio
 * @{
 *
 * @file
 * @brief       Register access function definitions for the dummyradio device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 */

#ifndef DUMMYRADIO_SPI_H_
#define DUMMYRADIO_SPI_H_

#include <stdint.h>

#include "board.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t dummyradio_reg_read(uint8_t addr);
void dummyradio_reg_write(uint8_t addr, uint8_t value);

void dummyradio_read_fifo(uint8_t *data, radio_packet_length_t length);
void dummyradio_write_fifo(const uint8_t *data, radio_packet_length_t length);

uint8_t dummyradio_get_status(void);

void* dummyradio_faketrx(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* DUMMYRADIO_SPI_H_ */
/** @} */
