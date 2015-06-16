/*
 * Copyright (C) 2014 Triagnosys
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
 * @brief       pingpong at86rf231
 *
 * @author      Xavier Perez <xaviersimo@triagnosys.com>
 *
 * @}
 */

#include <stdio.h>

#include "vtimer.h"
#include "thread.h"
#include "transceiver.h"
#include "at86rf231.h"

#define MSEC            (1000)
#define SEC             (1000 * MSEC)
#define STARTNODE       (1)

#define ENABLE_DEBUG    (0)
#include "debug.h"

static msg_t msg_q[TRANSCEIVER_BUFFER_SIZE];

int main(void)
{
    int packet_num = 0;
    msg_t m;
    ieee802154_packet_t *p;

    msg_init_queue(msg_q, TRANSCEIVER_BUFFER_SIZE);

    /*Init program*/
    printf("# ********************************************* \n");
    printf("# ************ pingpong AT86RF231 ************* \n");
    printf("# ********************************************* \n");
#if STARTNODE
    printf("# ***************** STARTNODE ***************** \n");
    printf("# ********************************************* \n");
#endif

    uint16_t trx = TRANSCEIVER_DEFAULT;
    transceiver_init(trx);
    transceiver_start();
    transceiver_register(trx, thread_getpid()); //register main to transceiver
    DEBUG("transceiver inizialized and started\n");

    /*Chanel*/
    unsigned int chan = 20;   //Transceiver riot module define channel for default. Use this functions if we want to fix another channel
    at86rf231_set_channel(chan);

    /*adress*/
#if STARTNODE
    radio_address_t addr = 0xaaaa; //define source address start node (Node_A)
#else
    radio_address_t addr = 0xcccc; //define source address destination node (Node_B)
#endif

    /*address*/
    at86rf231_set_address(addr);

    /*PAN*/
    uint16_t pan = 0x0001;
    at86rf231_set_pan(pan);

    /*Define data packet*/
    uint8_t payload[] = {0};

    /*Config MAC header */
    ieee802154_frame_t frame;
    frame.fcf.frame_type = IEEE_802154_DATA_FRAME;
    frame.fcf.sec_enb = 0;
    frame.fcf.frame_pend = 0;
    frame.fcf.ack_req = 1;
    frame.dest_pan_id = pan;
    frame.fcf.src_addr_m = 0x02;
    frame.fcf.dest_addr_m = 0x02;
    frame.payload = payload;
    frame.payload_len = sizeof(payload);

#if STARTNODE
    frame.dest_addr[0] = 0xcc; //destination from start node (Node_A)
    frame.dest_addr[1] = 0xcc;
#else
    frame.dest_addr[0] = 0xaa; //destination from destination node (Node_B)
    frame.dest_addr[1] = 0xaa;
#endif

    /*Build a packet*/
    at86rf231_packet_t packet;
    packet.frame = frame;

#if STARTNODE
    at86rf231_send(&packet); //start node send first packet
    DEBUG("1st packet sent\n");
#endif

    while(1) {
        DEBUG("waiting for receiving msg\n");
        msg_receive(&m);

        switch (m.type) {
        case PKT_PENDING:
            p = (ieee802154_packet_t*) m.content.ptr;            

        #if ENABLE_DEBUG
            DEBUG("Got radio packet:\n");
            DEBUG("\tLength:\t%u\n", p->length);
            DEBUG("\tSrc:\t%u\n", (p->frame.src_addr[0])|(p->frame.src_addr[1]<<8));
            DEBUG("\tDst:\t%u\n", (p->frame.dest_addr[0])|(p->frame.dest_addr[1]<<8));
            DEBUG("\tLQI:\t%u\n", p->lqi);
            DEBUG("\tRSSI:\t%u\n", p->rssi);
            DEBUG("Payload Length:%u\n", p->frame.payload_len);
            DEBUG("Payload:%s \n", p->frame.payload);
        #endif

            packet_num = *(p->frame.payload);
            printf("Payload received: %d\n", packet_num);
            p->processing--; //tell transceiver packet is no longer processed

            packet_num++;
            if (packet_num == 1000) {
                printf("test finished\n"); //stop the test
                while(1);
            } else {
                payload[0] = packet_num;
                frame.payload = payload;
                packet.frame = frame;
                vtimer_usleep(SEC);
                at86rf231_send(&packet);
                DEBUG("packet sent\n");                
            }            
            break;

        case ENOBUFFER:
            puts("Transceiver buffer full");
            break;

        default:
            puts("Unknown packet received");
            break;
        }        
    }

    return 0;
}
