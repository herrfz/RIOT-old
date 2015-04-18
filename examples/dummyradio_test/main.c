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
#include "vtimer.h"
#include "dummyradio.h"

#define MSEC (1000)
#define SEC (1000 * MSEC)

#define ENABLE_DEBUG (1)
#include "debug.h"

int main(void)
{
	dummyradio_initialize(NULL);
	DEBUG("dummyradio initialized\n");

	/* channel */
	unsigned int chan = 6;
	dummyradio_set_channel(chan);
	uint16_t channel = dummyradio_get_channel();
	DEBUG("channel is: %d\n", channel);

	/* address */
	radio_address_t addr = 0xaaaa; //define source address start node
	dummyradio_set_address(addr);
	radio_address_t address = dummyradio_get_address();
	DEBUG("Source address is: %x \n", address);

	/* PAN */
	uint16_t set_pan = 0x0001;
	dummyradio_set_pan(set_pan);
	uint16_t pan = dummyradio_get_pan();
	DEBUG("PAN is: %x \n", pan);

	/* define data packet */
	uint8_t payload[] = {0xca, 0xfe};

	/* MAC header */
	ieee802154_frame_t frame;
	frame.fcf.frame_type = IEEE_802154_DATA_FRAME;
	frame.fcf.sec_enb = 0;
	frame.fcf.frame_pend = 0;
	frame.fcf.ack_req = 0;
	frame.dest_pan_id = pan;
	frame.fcf.src_addr_m = 0x02;
	frame.fcf.dest_addr_m = 0x02;
	frame.payload = payload;
	frame.payload_len = sizeof(payload);
	frame.dest_addr[0] = 0xcc;
	frame.dest_addr[1] = 0xcc;

	/* build a packet */
	dummyradio_packet_t packet;
	packet.frame = frame;
	DEBUG("packet created\n");

	/* get state machine status */
	uint8_t status = dummyradio_get_status();
	DEBUG("status: %x\n", status);

	while(1) {
    	dummyradio_send(&packet);
    	DEBUG("packet sent\n\n");
    	vtimer_usleep(SEC);
	}

	return 0;
}
