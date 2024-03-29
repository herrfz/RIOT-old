/*
 * Copyright (C) 2015 Eriza Fazli
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
 * @brief       TX related functionality for the DUMMYRADIO device driver
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
 *
 * @}
 */

#include "dummyradio.h"
#include "dummyradio_spi.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define _MAX_RETRIES   (100)

static int16_t dummyradio_load(dummyradio_packet_t *packet);
static void dummyradio_gen_pkt(uint8_t *buf, dummyradio_packet_t *packet);

static uint8_t sequence_nr;

int16_t dummyradio_send(dummyradio_packet_t *packet)
{
    int16_t result;
    /* radio driver state: sending */
    /* will be freed in dummyradio_rx_irq when TRX_END interrupt occurs */
    driver_state = AT_DRIVER_STATE_SENDING;
    
    result = dummyradio_load(packet);
    if (result < 0) {
        return result;
    }
    dummyradio_transmit_tx_buf(NULL);

    return result;
}

netdev_802154_tx_status_t dummyradio_transmit_tx_buf(netdev_t *dev)
{
    (void)dev;

    dummyradio_reg_write(DUMMYRADIO_REG__TRX_STATE, DUMMYRADIO_TRX_STATE__TX_START);
    DEBUG("dummyradio: Started TX\n");

    return NETDEV_802154_TX_STATUS_OK;
}

int16_t dummyradio_load(dummyradio_packet_t *packet)
{
    // Set missing frame information
    packet->frame.fcf.frame_ver = 0;

    packet->frame.src_pan_id = dummyradio_get_pan();

    if (packet->frame.src_pan_id == packet->frame.dest_pan_id) {
        packet->frame.fcf.panid_comp = 1;
    }
    else {
        packet->frame.fcf.panid_comp = 0;
    }

    if (packet->frame.fcf.src_addr_m == 2) {
        packet->frame.src_addr[0] = (uint8_t)(dummyradio_get_address() >> 8);
        packet->frame.src_addr[1] = (uint8_t)(dummyradio_get_address() & 0xFF);
    }
    else if (packet->frame.fcf.src_addr_m == 3) {
        packet->frame.src_addr[0] = (uint8_t)(dummyradio_get_address_long() >> 56);
        packet->frame.src_addr[1] = (uint8_t)(dummyradio_get_address_long() >> 48);
        packet->frame.src_addr[2] = (uint8_t)(dummyradio_get_address_long() >> 40);
        packet->frame.src_addr[3] = (uint8_t)(dummyradio_get_address_long() >> 32);
        packet->frame.src_addr[4] = (uint8_t)(dummyradio_get_address_long() >> 24);
        packet->frame.src_addr[5] = (uint8_t)(dummyradio_get_address_long() >> 16);
        packet->frame.src_addr[6] = (uint8_t)(dummyradio_get_address_long() >> 8);
        packet->frame.src_addr[7] = (uint8_t)(dummyradio_get_address_long() & 0xFF);
    }

    packet->frame.seq_nr = sequence_nr++;

    /* calculate size of the frame (payload + FCS) */
    packet->length = ieee802154_frame_get_hdr_len(&packet->frame) +
                     packet->frame.payload_len + 1;
    ieee802154_frame_get_hdr_len(&packet->frame);

    if (packet->length > DUMMYRADIO_MAX_PKT_LENGTH) {
        DEBUG("dummyradio: ERROR: packet too long, dropped it.\n");
        return -1;
    }

    /*print mac header*/
#if ENABLE_DEBUG
    printf("\n");
    printf("******NEW PACKET*****\n");
    printf("******MAC HEADER*****\n");
    ieee802154_frame_print_fcf_frame(&packet->frame);
#endif
    /* FCS is added in hardware */
    uint8_t pkt[packet->length];

    /* generate pkt */
    dummyradio_gen_pkt(pkt, packet);

    /* load packet into fifo */
    dummyradio_write_fifo(pkt, packet->length);
    DEBUG("dummyradio: Wrote to FIFO and packet length is:%d\n", packet->length);

    return packet->length;
}

/**
 * @brief Static function to generate byte array from dummyradio packet.
 */
static void dummyradio_gen_pkt(uint8_t *buf, dummyradio_packet_t *packet)
{
    uint8_t index, offset;
    index = ieee802154_frame_init(&packet->frame, &buf[1]);

    // add length for dummyradio
    buf[0] = packet->length + 1;
    index++;
    offset = index;
    DEBUG("******PAYLOAD*****\n");
    while (index < packet->length) {
        buf[index] = packet->frame.payload[index - offset];
        DEBUG("%02x  ", buf[index]);
        index += 1;
    }
    DEBUG("\n");
}



