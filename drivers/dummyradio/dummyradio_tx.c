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
 * @brief       TX related functionality for the DUMMYRADIO device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * @}
 */

#include "dummyradio.h"
#include "dummyradio_spi.h"
#include "hwtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#define _MAX_RETRIES   (100)

static int16_t dummyradio_load(dummyradio_packet_t *packet);
static void dummyradio_gen_pkt(uint8_t *buf, dummyradio_packet_t *packet);

static uint8_t sequence_nr;
static uint8_t wait_for_ack;

int16_t dummyradio_send(dummyradio_packet_t *packet)
{
    int16_t result;
    result = dummyradio_load(packet);
    if (result < 0) {
        return result;
    }
   dummyradio_transmit_tx_buf(NULL);
    return result;
}

netdev_802154_tx_status_t dummyradio_load_tx_buf(netdev_t *dev,
        netdev_802154_pkt_kind_t kind,
        netdev_802154_node_addr_t *dest,
        int use_long_addr,
        int wants_ack,
        netdev_hlist_t *upper_layer_hdrs,
        void *buf,
        unsigned int len)
{
    (void)dev;

    uint8_t mhr[24];
    uint8_t index = 3;

    /* frame type */
    switch (kind) {
        case NETDEV_802154_PKT_KIND_BEACON:
            mhr[0] = 0x00;
            break;
        case NETDEV_802154_PKT_KIND_DATA:
            mhr[0] = 0x01;
            break;
        case NETDEV_802154_PKT_KIND_ACK:
            mhr[0] = 0x02;
            break;
        default:
            return NETDEV_802154_TX_STATUS_INVALID_PARAM;
    }

    if (wants_ack) {
        mhr[0] |= 0x20;
    }

    wait_for_ack = wants_ack;

    uint16_t src_pan = dummyradio_get_pan();
    uint8_t compress_pan = 0;

    if (use_long_addr) {
        mhr[1] = 0xcc;
    }
    else {
        mhr[1] = 0x88;
        if (dest->pan.id == src_pan) {
            compress_pan = 1;
            mhr[0] |= 0x40;
        }
    }

    mhr[2] = sequence_nr++;

    /* First 3 bytes are fixed with FCS and SEQ, resume with index=3 */
    if (use_long_addr) {
        mhr[index++] = (uint8_t)(dest->long_addr & 0xFF);
        mhr[index++] = (uint8_t)(dest->long_addr >> 8);
        mhr[index++] = (uint8_t)(dest->long_addr >> 16);
        mhr[index++] = (uint8_t)(dest->long_addr >> 24);
        mhr[index++] = (uint8_t)(dest->long_addr >> 32);
        mhr[index++] = (uint8_t)(dest->long_addr >> 40);
        mhr[index++] = (uint8_t)(dest->long_addr >> 48);
        mhr[index++] = (uint8_t)(dest->long_addr >> 56);

        uint64_t src_long_addr = dummyradio_get_address_long();
        mhr[index++] = (uint8_t)(src_long_addr & 0xFF);
        mhr[index++] = (uint8_t)(src_long_addr >> 8);
        mhr[index++] = (uint8_t)(src_long_addr >> 16);
        mhr[index++] = (uint8_t)(src_long_addr >> 24);
        mhr[index++] = (uint8_t)(src_long_addr >> 32);
        mhr[index++] = (uint8_t)(src_long_addr >> 40);
        mhr[index++] = (uint8_t)(src_long_addr >> 48);
        mhr[index++] = (uint8_t)(src_long_addr >> 56);
    }
    else {
        mhr[index++] = (uint8_t)(dest->pan.id & 0xFF);
        mhr[index++] = (uint8_t)(dest->pan.id >> 8);

        mhr[index++] = (uint8_t)(dest->pan.addr & 0xFF);
        mhr[index++] = (uint8_t)(dest->pan.addr >> 8);

        if (!compress_pan) {
            mhr[index++] = (uint8_t)(src_pan & 0xFF);
            mhr[index++] = (uint8_t)(src_pan >> 8);
        }

        uint16_t src_addr = dummyradio_get_address();
        mhr[index++] = (uint8_t)(src_addr & 0xFF);
        mhr[index++] = (uint8_t)(src_addr >> 8);
    }

    /* total frame size:
     * index -> MAC header
     * len   -> payload length
     * 2     -> CRC bytes
     * + lengths of upper layers' headers */
    size_t size = index + len + 2 + netdev_get_hlist_len(upper_layer_hdrs);

    if (size > DUMMYRADIO_MAX_PKT_LENGTH) {
        DEBUG("dummyradio: packet too long, dropped it.\n");
        return NETDEV_802154_TX_STATUS_PACKET_TOO_LONG;
    }

    uint8_t size_byte = (uint8_t)size;
    netdev_hlist_t *ptr = upper_layer_hdrs;

    dummyradio_write_fifo(&size_byte, 1);
    dummyradio_write_fifo(mhr, (radio_packet_length_t)index);
    if (upper_layer_hdrs) {
        do {
            dummyradio_write_fifo(ptr->header,
                                (radio_packet_length_t)(ptr->header_len));
            netdev_hlist_advance(&ptr);
        } while (ptr != upper_layer_hdrs);
    }
    dummyradio_write_fifo((uint8_t*)buf, len);
    return NETDEV_802154_TX_STATUS_OK;
}

netdev_802154_tx_status_t dummyradio_transmit_tx_buf(netdev_t *dev)
{
    (void)dev;
    /* radio driver state: sending */
    /* will be freed in dummyradio_rx_irq when TRX_END interrupt occurs */
    driver_state = AT_DRIVER_STATE_SENDING;

    /* Start TX */
    dummyradio_reg_write(DUMMYRADIO_REG__TRX_STATE, DUMMYRADIO_TRX_STATE__TX_START);

    DEBUG("dummyradio: Started TX\n");

    // if (!wait_for_ack) {
    //     DEBUG("dummyradio: Don't wait for ACK, TX done.\n");
    //     return NETDEV_802154_TX_STATUS_OK;
    // }

    // uint8_t trac_status;
    // do {
    //     trac_status = dummyradio_reg_read(DUMMYRADIO_REG__TRX_STATE);
    //     trac_status &= DUMMYRADIO_TRX_STATE_MASK__TRAC;
    // }
    // while (trac_status == DUMMYRADIO_TRX_STATE__TRAC_INVALID);

    // switch (trac_status) {
    //     case DUMMYRADIO_TRX_STATE__TRAC_CHANNEL_ACCESS_FAILURE:
    //         return NETDEV_802154_TX_STATUS_MEDIUM_BUSY;

    //     case DUMMYRADIO_TRX_STATE__TRAC_NO_ACK:
    //         return NETDEV_802154_TX_STATUS_NOACK;

    //     default:
    //         return NETDEV_802154_TX_STATUS_OK;
    // }
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

    /* Go to state PLL_ON */
    dummyradio_reg_write(DUMMYRADIO_REG__TRX_STATE, DUMMYRADIO_TRX_STATE__PLL_ON);

    // /* wait until it is on PLL_ON state */
    // do {
    //     int max_wait = _MAX_RETRIES;
    //     if (!--max_wait) {
    //         DEBUG("dummyradio : ERROR : could not enter PLL_ON mode\n");
    //         break;
    //     }
    // } while ((dummyradio_get_status() & DUMMYRADIO_TRX_STATUS_MASK__TRX_STATUS)
    //          != DUMMYRADIO_TRX_STATUS__PLL_ON);

    /* change into TX_ARET_ON state */
    dummyradio_reg_write(DUMMYRADIO_REG__TRX_STATE, DUMMYRADIO_TRX_STATE__TX_ARET_ON);

    // do {
    //     int max_wait = _MAX_RETRIES;
    //     if (!--max_wait) {
    //         DEBUG("dummyradio : ERROR : could not enter TX_ARET_ON mode\n");
    //         break;
    //     }
    // } while (dummyradio_get_status() != DUMMYRADIO_TRX_STATUS__TX_ARET_ON);

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



