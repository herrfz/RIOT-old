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
 * @brief       RX related functionality for the DUMMYRADIO device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * @}
 */

#include "dummyradio.h"
#include "dummyradio_spi.h"

#include "kernel_types.h"
#include "transceiver.h"
#include "msg.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

dummyradio_packet_t dummyradio_rx_buffer[DUMMYRADIO_RX_BUF_SIZE];
static uint8_t buffer[DUMMYRADIO_RX_BUF_SIZE][DUMMYRADIO_MAX_PKT_LENGTH];
volatile uint8_t rx_buffer_next;
extern netdev_802154_raw_packet_cb_t dummyradio_raw_packet_cb;

void dummyradio_rx_handler(void)
{
    uint8_t lqi, fcs_rssi;
    /* read packet length */
    dummyradio_read_fifo(&dummyradio_rx_buffer[rx_buffer_next].length, 1);

    /* read psdu, read packet with length as first byte and lqi as last byte. */
    uint8_t *buf = buffer[rx_buffer_next];
    dummyradio_read_fifo(buf, dummyradio_rx_buffer[rx_buffer_next].length);

    /* read lqi which is appended after the psdu */
    lqi = buf[dummyradio_rx_buffer[rx_buffer_next].length - 1];

    /* dummy fcs and rssi */
    fcs_rssi = 0xfc;

    /* build package */
    dummyradio_rx_buffer[rx_buffer_next].lqi = lqi;
    /* RSSI has no meaning here, it should be read during packet reception. */
    dummyradio_rx_buffer[rx_buffer_next].rssi = fcs_rssi & 0x1F;  /* bit[4:0] */
    /* bit7, boolean, 1 FCS valid, 0 FCS not valid */
    dummyradio_rx_buffer[rx_buffer_next].crc = (fcs_rssi >> 7) & 0x01;

    if (dummyradio_rx_buffer[rx_buffer_next].crc == 0) {
        DEBUG("dummyradio: Got packet with invalid crc.\n");
        return;
    }

#if ENABLE_DEBUG
    DEBUG("pkg: ");
    for (int i = 1; i < dummyradio_rx_buffer[rx_buffer_next].length; i++) {
        DEBUG("%x ", buf[i]);
    }
    DEBUG("\n");
#endif

    /* read buffer into ieee802154_frame */
    ieee802154_frame_read(&buf[1], &dummyradio_rx_buffer[rx_buffer_next].frame,
                          dummyradio_rx_buffer[rx_buffer_next].length);

    /* if packet is no ACK */
    if (dummyradio_rx_buffer[rx_buffer_next].frame.fcf.frame_type != IEEE_802154_ACK_FRAME) {
#if ENABLE_DEBUG
        ieee802154_frame_print_fcf_frame(&dummyradio_rx_buffer[rx_buffer_next].frame);
#endif
        if (dummyradio_raw_packet_cb != NULL) {
            dummyradio_raw_packet_cb(&dummyradio_netdev, (void*)buf,
                                    dummyradio_rx_buffer[rx_buffer_next].length,
                                    fcs_rssi, lqi, (fcs_rssi >> 7));
        }
#ifdef MODULE_TRANSCEIVER
        /* notify transceiver thread if any */
        if (transceiver_pid != KERNEL_PID_UNDEF) {
            msg_t m;
            m.type = (uint16_t) RCV_PKT_DUMMYRADIO;
            m.content.value = rx_buffer_next;
            msg_send_int(&m, transceiver_pid);
        }
#endif
    }
    else {
        /* This should not happen, ACKs are consumed by hardware */
#if ENABLE_DEBUG
        DEBUG("GOT ACK for SEQ %u\n", dummyradio_rx_buffer[rx_buffer_next].frame.seq_nr);
        ieee802154_frame_print_fcf_frame(&dummyradio_rx_buffer[rx_buffer_next].frame);
#endif
    }

    /* shift to next buffer element */
    if (++rx_buffer_next == DUMMYRADIO_RX_BUF_SIZE) {
        rx_buffer_next = 0;
    }

}
