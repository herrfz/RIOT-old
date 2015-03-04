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

//#include <stdio.h>
#include "board.h"
#include "vtimer.h"
#include "uart.h"
#include "gpio.h"
#include "thread.h"
#include "transceiver.h"
#include "at86rf231.h"

#define MSEC (1000)
#define SEC (1000 * MSEC)

#define ENABLE_DEBUG (0)
#include "debug.h"

#define STARTNODE    (0) //define pingpong node


int main(void)
{

	/*Init program*/
	printf("# ********************************************* \n");
	printf("# ************ pingpong AT86RF231 ************* \n");
	printf("# ********************************************* \n");
#if STARTNODE
	printf("# ***************** STARTNODE ***************** \n");
	printf("# ********************************************* \n");
#endif

	transceiver_init(TRANSCEIVER_AT86RF231);
	transceiver_start();
	//at86rf231_initialize(NULL); //Use this function to config directly the radio module without use general transceiver functions
	DEBUG("at86rf231 inizialized\n");

	/*Chanel*/
	//unsigned int chan = 6;   //Transceiver riot module define channel for default. Use this functions if we want to fix another channel
	//at86rf231_set_channel(chan);
	uint16_t channel = at86rf231_get_channel();
	DEBUG("channel is: %d\n", channel);

	/*adress*/
#if STARTNODE
	radio_address_t addr = 0xaaaa; //define source address start node (Node_A)
#else
	radio_address_t addr = 0xcccc; //define source address destination node (Node_B)
#endif
	at86rf231_set_address(addr);
	radio_address_t address = at86rf231_get_address();
	DEBUG("Source address is: %x \n", address);

	/*PAN*/
	uint16_t set_pan = 0x0001, newpan;
	newpan = at86rf231_set_pan(set_pan);
	uint16_t pan = at86rf231_get_pan();
	DEBUG("PAN is: %x \n", pan);

	/*Define data packet*/
	//uint8_t payload[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
	//char payload[] = "hello";
	char payload[] = "00";

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
	frame.dest_addr[0] = 0xcc;  //destination from start node (Node_A)
	frame.dest_addr[1] = 0xcc;
#else
	frame.dest_addr[0] = 0xaa; //destination from destination node (Node_B)
	frame.dest_addr[1] = 0xaa;
#endif

	/*Build a packet*/
	at86rf231_packet_t packet;
	packet.frame = frame;
	DEBUG("packet created\n");

	/*Get state machine status*/
	uint8_t status = at86rf231_get_status();
	DEBUG("status: %x\n", status);

	/*Define parameters to receive a packet in main*/
	msg_t m;
	static msg_t msg_q[TRANSCEIVER_BUFFER_SIZE];
	ieee802154_packet_t *p;
    msg_init_queue(msg_q, TRANSCEIVER_BUFFER_SIZE);
    int packet_num = 0;

	transceiver_register(TRANSCEIVER_AT86RF231, thread_getpid()); //Send packets from transceiver function to main

#if STARTNODE

	at86rf231_send(&packet); //star node send first packet
	DEBUG("1st packet sended\n");

#endif

while(1)
{
        DEBUG("waiting for receiving msg\n");
        msg_receive(&m);

	        if (m.type == PKT_PENDING) {

	            p = (ieee802154_packet_t*) m.content.ptr;
	            DEBUG("Got radio packet:\n");
	            DEBUG("\tLength:\t%u\n", p->length);
	            DEBUG("\tSrc:\t%u\n", (p->frame.src_addr[0])|(p->frame.src_addr[1]<<8));
	            DEBUG("\tDst:\t%u\n", (p->frame.dest_addr[0])|(p->frame.dest_addr[1]<<8));
	            DEBUG("\tLQI:\t%u\n", p->lqi);
	            DEBUG("\tRSSI:\t%u\n", p->rssi);

	            DEBUG("Payload Length:%u\n", p->frame.payload_len);
	            DEBUG("Payload:%s \n", p->frame.payload);

	            /*Convert payload from string to integer*/
	            packet_num = atoi (p->frame.payload);

	            p->processing--;

	        }
	        else if (m.type == ENOBUFFER) {
	            puts("Transceiver buffer full");
	        }
	        else {
	            puts("Unknown packet received");
	        }

	        packet_num++;
	        printf("Payload received: %i\n", packet_num);

	        if(packet_num == 1000)
	        {
	        	printf("test finish\n"); //stop the test
	        	while(1)
	        	{

	        	}

	        }
	        /*Convert payload from integer to string and encapsulate again */
	        sprintf(frame.payload,"%d" ,packet_num);
	        packet.frame = frame;
	        vtimer_usleep(SEC);
	    	at86rf231_send(&packet);
	    	DEBUG("packet sended\n");
}
	return 0;
}
