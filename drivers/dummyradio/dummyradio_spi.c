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
#include "vtimer.h"
#include "msg.h"

#define SAMPLES 50
#define DEBUG   1
#define VERBOSE 0


/*                           len   fcf         seq   dstpan      dstaddr     srcpan      srcaddr     payload     lqi */
static uint8_t fifo_reg[] = {0x0e, 0x01, 0x88, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1c, 0xaa, 0x00, 0x00, 0xca, 0xfe, 0x01};
static uint8_t beacon[] = {0x3e, 0x40, 0xea, 0x00, 0xfe, 0xca, 0xff, 0xff, 0x30, 0x34, 0x51, 0x10, 0x00, 0x59, 0x00, 0x6c, 0xa3, 0x05, 0x34, 0x06, 0x2f, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x36, 0x23, 0x01, 0x01, 0x65, 0x00, 0x06, 0x01, 0x00, 0x00, 0x00, 0xf0, 0x02, 0x00, 0x00, 0x00, 0xe0, 0x03, 0x00, 0x00, 0x00, 0xe0, 0x04, 0x00, 0x00, 0x00, 0xe0, 0x05, 0x00, 0x00, 0x00, 0xe0, 0x06, 0x00, 0x00, 0x00, 0xe0, 0x01};
static uint8_t irq_status, fifopointer = 0, rxcount = 2;
int calls = 0;
static uint32_t callingtime[SAMPLES];
void capture_time(void);


void dummyradio_reg_write(uint8_t addr, uint8_t value)
{
    if (addr == DUMMYRADIO_REG__IRQ_STATUS) {
        irq_status = value;
        if (value == DUMMYRADIO_IRQ_STATUS_MASK__TRX_END) {
        #if DEBUG
            capture_time();
        #endif
            TRX_INT(); // fake end-of-frame interrupt
        }
    }
}

uint8_t dummyradio_reg_read(uint8_t addr)
{
#if VERBOSE
    printf("dummyradio_reg_read addr: %02x\n", addr);
#endif
    return irq_status;
}

void dummyradio_read_fifo(uint8_t *data, radio_packet_length_t length)
{
    if (rxcount) { // first two calls for node are beacon length and beacon
        if (fifopointer > sizeof(beacon)) fifopointer = 0;
        memcpy(data, &beacon[fifopointer], length);
        fifopointer += length;
        rxcount--;
    } else {
        if (fifopointer > sizeof(fifo_reg)) fifopointer = 0;
        memcpy(data, &fifo_reg[fifopointer], length);
        fifopointer += length;
    }
    irq_status = DUMMYRADIO_IRQ_STATUS_MASK__TRX_END;
#if VERBOSE
    printf("dummyradio_read_fifo: %d\n", length);
#endif
}

void dummyradio_write_fifo(const uint8_t *data, radio_packet_length_t length)
{
    puts("dummyradio_write_fifo");
    for (int i = 0; i < length; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
}

uint8_t dummyradio_get_status(void)
{
    puts("dummyradio_get_status\n");
    return 0;
}

void* dummyradio_faketrx(void *arg)
{
    msg_t msg;
    while(1) {
        msg_receive(&msg);
        //vtimer_usleep(15000); // TODO timeslot width
        //irq_status = msg.content.value;
        TRX_INT(); // interrupt
    }
}


void capture_time(void)
{
    timex_t now;
    vtimer_now(&now);
    if (calls < SAMPLES) {
        callingtime[calls] = SEC_IN_USEC * now.seconds + now.microseconds;
    } else if (calls == SAMPLES) {
        for (int i = 0; i < SAMPLES; i++) {
            printf("%06" PRIu32 "\n", callingtime[i]);
        }
    } else {
        // do nothing
    }
    calls++;
}
