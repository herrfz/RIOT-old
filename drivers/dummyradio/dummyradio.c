/*
 * Copyright (C) 2013 Alaeddine Weslati <alaeddine.weslati@inria.fr>
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
 * @brief       Driver implementation of the DUMMYRADIO device driver
 *
 * @author      Alaeddine Weslati <alaeddine.weslati@inria.fr>
 * @author      Thomas Eichinger <thomas.eichinger@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h> 

#include "dummyradio.h"
#include "dummyradio_spi.h"
#include "board.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include "kernel_types.h"
#include "transceiver.h"
#include "hwtimer.h"
#include "config.h"
#include "thread.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#ifndef DUMMYRADIO_SPI_SPEED
#define SPI_SPEED    SPI_SPEED_5MHZ
#else
#define SPI_SPEED    DUMMYRADIO_SPI_SPEED
#endif

#define _MAX_RETRIES    (100)

#define PRIORITY_OPENWSN            PRIORITY_MAIN-1

static uint16_t radio_pan;
static uint8_t  radio_channel;
static uint16_t radio_address;
static uint64_t radio_address_long;

netdev_802154_raw_packet_cb_t dummyradio_raw_packet_cb;
netdev_rcv_data_cb_t dummyradio_data_packet_cb;

/* default source address length for sending in number of byte */
static size_t _default_src_addr_len = 2;

uint8_t  driver_state;
int      monitor_mode;

static char dummyradio_stack[KERNEL_CONF_STACKSIZE_MAIN];

void dummyradio_gpio_spi_interrupts_init(void);
void dummyradio_reset(void);

#ifdef MODULE_TRANSCEIVER
void dummyradio_init(kernel_pid_t tpid)
{
    transceiver_pid = tpid;
    dummyradio_initialize(NULL);
}
#endif

int dummyradio_initialize(netdev_t *dev)
{
    dummyradio_gpio_spi_interrupts_init();
    dummyradio_reset();
    dummyradio_on();

    /* TODO :
     * and configure security, power
     */
#ifdef MODULE_CONFIG
    dummyradio_set_pan(sysconfig.radio_pan_id);
#else
    dummyradio_set_pan(0x0001);
#endif

    radio_channel = 0;
    radio_address = 0x010F;
    radio_address_long = 0x010203040506010F;

#ifdef MODULE_OPENWSN
    thread_create(dummyradio_stack, KERNEL_CONF_STACKSIZE_MAIN,
                    PRIORITY_OPENWSN, CREATE_STACKTEST,
                    dummyradio_receive_int, NULL, "dummyradio rx thread");
#endif

    return 0;
}



int dummyradio_on(void)
{
    printf("dummyradio_on\n");
    return 1;
}

void dummyradio_off(void)
{

}

int dummyradio_is_on(void)
{
    return ((dummyradio_get_status() & 0x1f) != 0);
}

void dummyradio_switch_to_rx(void)
{

#ifndef MODULE_OPENWSN
    /* Reset IRQ to TRX END only */
    dummyradio_reg_write(DUMMYRADIO_REG__IRQ_MASK, DUMMYRADIO_IRQ_STATUS_MASK__TRX_END);
#else
    /* OpenWSN also needs RX_START IRQ */
    dummyradio_reg_write(DUMMYRADIO_REG__IRQ_MASK, ( DUMMYRADIO_IRQ_STATUS_MASK__RX_START | DUMMYRADIO_IRQ_STATUS_MASK__TRX_END));
#endif

}

void dummyradio_rxoverflow_irq(void)
{
    /* TODO */
}

#ifndef MODULE_OPENWSN
void dummyradio_rx_irq(void)
{
    /* check if we are in sending state */
    if (driver_state == AT_DRIVER_STATE_SENDING) {
        /* tx done, listen again */
        dummyradio_switch_to_rx();
        /* clear internal state */
        driver_state = AT_DRIVER_STATE_DEFAULT;
    }
    else {
        /* handle receive */
        dummyradio_rx_handler();
    }
}
#endif

int dummyradio_add_raw_recv_callback(netdev_t *dev,
                                    netdev_802154_raw_packet_cb_t recv_cb)
{
    (void)dev;

    if (dummyradio_raw_packet_cb == NULL){
        dummyradio_raw_packet_cb = recv_cb;
        return 0;
    }

    return -ENOBUFS;

}

int dummyradio_rem_raw_recv_callback(netdev_t *dev,
                                    netdev_802154_raw_packet_cb_t recv_cb)
{
    (void)dev;

    dummyradio_raw_packet_cb = NULL;
    return 0;
}

int dummyradio_add_data_recv_callback(netdev_t *dev,
                                     netdev_rcv_data_cb_t recv_cb)
{
    (void)dev;

    if (dummyradio_data_packet_cb == NULL){
        dummyradio_data_packet_cb = recv_cb;
        return 0;
    }

    return -ENOBUFS;
}

int dummyradio_rem_data_recv_callback(netdev_t *dev,
                                     netdev_rcv_data_cb_t recv_cb)
{
    (void)dev;

    dummyradio_data_packet_cb = NULL;
    return 0;
}

radio_address_t dummyradio_set_address(radio_address_t address)
{
    radio_address = address;
    return radio_address;
}

radio_address_t dummyradio_get_address(void)
{
    return radio_address;
}

uint64_t dummyradio_set_address_long(uint64_t address)
{
    radio_address_long = address;
    return radio_address_long;
}

uint64_t dummyradio_get_address_long(void)
{
    return radio_address_long;
}

uint16_t dummyradio_set_pan(uint16_t pan)
{
    radio_pan = pan;
    return radio_pan;
}

uint16_t dummyradio_get_pan(void)
{
    return radio_pan;
}

int dummyradio_set_channel(unsigned int channel)
{
    radio_channel = channel;

    if (channel < DUMMYRADIO_MIN_CHANNEL ||
        channel > DUMMYRADIO_MAX_CHANNEL) {
#if DEVELHELP
        puts("[dummyradio] channel out of range!");
#endif
        return -1;
    }

    return radio_channel;
}

unsigned int dummyradio_get_channel(void)
{
    return radio_channel;
}

void dummyradio_set_monitor(int mode)
{
    monitor_mode = mode;
}

int dummyradio_get_monitor(void)
{
    return monitor_mode;
}

void dummyradio_gpio_spi_interrupts_init(void)
{
    /* interrupt from GPIO_0 */
    gpio_init_int(GPIO_0, GPIO_NOPULL, GPIO_RISING, (gpio_cb_t)dummyradio_rx_irq, NULL);
}

void dummyradio_reset(void)
{

}

int dummyradio_get_option(netdev_t *dev, netdev_opt_t opt, void *value,
                         size_t *value_len)
{
    /* XXX: first check only for backwards compatibility with transceiver
     *      (see dummyradio_init) remove when adapter for transceiver exists */
    if (dev != &dummyradio_netdev) {
        return -ENODEV;
    }

    switch (opt) {
        case NETDEV_OPT_CHANNEL:
            if (*value_len < sizeof(unsigned int)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(unsigned int)) {
                *value_len = sizeof(unsigned int);
            }
            *((unsigned int *)value) = dummyradio_get_channel();
            break;

        case NETDEV_OPT_ADDRESS:
            if (*value_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint16_t)) {
                *value_len = sizeof(uint16_t);
            }
            *((uint16_t *)value) = dummyradio_get_address();
            break;

        case NETDEV_OPT_NID:
            if (*value_len < sizeof(uint16_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint16_t)) {
                *value_len = sizeof(uint16_t);
            }
            *((uint16_t *)value) = dummyradio_get_pan();
            break;

        case NETDEV_OPT_ADDRESS_LONG:
            if (*value_len < sizeof(uint64_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint64_t)) {
                *value_len = sizeof(uint64_t);
            }
            *((uint64_t *)value) = dummyradio_get_address_long();
            break;

        case NETDEV_OPT_MAX_PACKET_SIZE:
            if (*value_len == 0) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(uint8_t)) {
                *value_len = sizeof(uint8_t);
            }
            *((uint8_t *)value) = DUMMYRADIO_MAX_PKT_LENGTH;
            break;

        case NETDEV_OPT_PROTO:
            if (*value_len < sizeof(netdev_proto_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(netdev_proto_t)) {
                *value_len = sizeof(netdev_proto_t);
            }
            *((netdev_type_t *)value) = NETDEV_PROTO_802154;
            break;

        case NETDEV_OPT_SRC_LEN:
            if (*value_len < sizeof(size_t)) {
                return -EOVERFLOW;
            }
            if (*value_len > sizeof(size_t)) {
                *value_len = sizeof(size_t);
            }
            *((size_t *)value) = _default_src_addr_len;

        default:
            return -ENOTSUP;
    }

    return 0;
}

static int _type_pun_up_unsigned(void *value_out, size_t desired_len,
                                 void *value_in, size_t given_len)
{
    if (given_len > desired_len) {
        return -EOVERFLOW;
    }

    /* XXX this is ugly, but bear with me */
    switch (given_len) {
        case 8:
            switch (desired_len) {
                case 8:
                    *((uint64_t *)value_out) = (*((uint64_t *)value_in));
                    return 0;
                default:
                    return -EINVAL;
            }

        case 4:
            switch (desired_len) {
                case 8:
                    *((uint64_t *)value_out) = (uint64_t)(*((uint32_t *)value_in));
                    return 0;
                case 4:
                    *((uint32_t *)value_out) = (*((uint32_t *)value_in));
                    return 0;
                default:
                    return -EINVAL;
            }

        case 2:
            switch (desired_len) {
                case 8:
                    *((uint64_t *)value_out) = (uint64_t)(*((uint16_t *)value_in));
                    return 0;
                case 4:
                    *((uint32_t *)value_out) = (uint32_t)(*((uint16_t *)value_in));
                    return 0;
                case 2:
                    *((uint16_t *)value_out) = (*((uint16_t *)value_in));
                    return 0;
                default:
                    return -EINVAL;
            }

        case 1:
            switch (desired_len) {
                case 8:
                    *((uint64_t *)value_out) = (uint64_t)(*((uint8_t *)value_in));
                    return 0;
                case 4:
                    *((uint32_t *)value_out) = (uint32_t)(*((uint8_t *)value_in));
                    return 0;
                case 2:
                    *((uint16_t *)value_out) = (uint16_t)(*((uint8_t *)value_in));
                    return 0;
                case 1:
                    *((uint8_t *)value_out) = (*((uint8_t *)value_in));
                    return 0;
                default:
                    return -EINVAL;
            }

        default:
            return -EINVAL;
    }
}

int dummyradio_set_option(netdev_t *dev, netdev_opt_t opt, void *value,
                         size_t value_len)
{
    uint8_t set_value[sizeof(uint64_t)];
    int res = 0;

    /* XXX: first check only for backwards compatibility with transceiver
     *      (see dummyradio_init) remove when adapter for transceiver exists */
    if (dev != &dummyradio_netdev) {
        return -ENODEV;
    }

    switch (opt) {
        case NETDEV_OPT_CHANNEL:
            if ((res = _type_pun_up_unsigned(set_value, sizeof(unsigned int),
                                             value, value_len)) == 0) {
                unsigned int *v = (unsigned int *)set_value;
                if (*v > 26) {
                    return -EINVAL;
                }
                dummyradio_set_channel(*v);
            }
            break;

        case NETDEV_OPT_ADDRESS:
            if ((res = _type_pun_up_unsigned(set_value, sizeof(uint16_t),
                                             value, value_len)) == 0) {
                uint16_t *v = (uint16_t *)set_value;
                if (*v == 0xffff) {
                    /* Do not allow setting to broadcast */
                    return -EINVAL;
                }
                dummyradio_set_address(*v);
            }
            break;

        case NETDEV_OPT_NID:
            if ((res = _type_pun_up_unsigned(set_value, sizeof(uint16_t),
                                             value, value_len)) == 0) {
                uint16_t *v = (uint16_t *)set_value;
                if (*v == 0xffff) {
                    /* Do not allow setting to broadcast */
                    return -EINVAL;
                }
                dummyradio_set_pan(*v);
            }
            break;

        case NETDEV_OPT_ADDRESS_LONG:
            if ((res = _type_pun_up_unsigned(set_value, sizeof(uint64_t),
                                             value, value_len)) == 0) {
                uint64_t *v = (uint64_t *)set_value;
                /* TODO: error checking? */
                dummyradio_set_address_long(*v);
            }
            break;

        case NETDEV_OPT_SRC_LEN:
            if ((res = _type_pun_up_unsigned(set_value, sizeof(size_t),
                                             value, value_len)) == 0) {
                size_t *v = (size_t *)set_value;

                if (*v != 2 && *v != 8) {
                    return -EINVAL;
                }
                _default_src_addr_len = *v;
            }
            break;

        default:
            return -ENOTSUP;
    }

    return res;
}

int dummyradio_get_state(netdev_t *dev, netdev_state_t *state)
{
    /* XXX: first check only for backwards compatibility with transceiver
     *      (see dummyradio_init) remove when adapter for transceiver exists */
    if (dev != &dummyradio_netdev) {
        return -ENODEV;
    }

    if (!dummyradio_is_on()) {
        *state = NETDEV_STATE_POWER_OFF;
    }
    else if (dummyradio_get_monitor()) {
        *state = NETDEV_STATE_PROMISCUOUS_MODE;
    }
    else {
        *state = NETDEV_STATE_RX_MODE;
    }

    return 0;
}

int dummyradio_set_state(netdev_t *dev, netdev_state_t state)
{
    /* XXX: first check only for backwards compatibility with transceiver
     *      (see dummyradio_init) remove when adapter for transceiver exists */
    if (dev != &dummyradio_netdev) {
        return -ENODEV;
    }

    if (state != NETDEV_STATE_PROMISCUOUS_MODE && dummyradio_get_monitor()) {
        dummyradio_set_monitor(0);
    }

    switch (state) {
        case NETDEV_STATE_POWER_OFF:
            dummyradio_off();
            break;

        case NETDEV_STATE_RX_MODE:
            dummyradio_switch_to_rx();
            break;

        case NETDEV_STATE_PROMISCUOUS_MODE:
            dummyradio_set_monitor(1);
            break;

        default:
            return -ENOTSUP;
    }

    return 0;
}

int dummyradio_channel_is_clear(netdev_t *dev)
{
    (void)dev;
    /* channel is checked by hardware automatically before transmission */
    return 1;
}

void dummyradio_event(netdev_t *dev, uint32_t event_type)
{
    (void)dev;
    (void)event_type;
}

const netdev_802154_driver_t dummyradio_driver = {
    .init = dummyradio_initialize,
    .send_data = netdev_802154_send_data,
    .add_receive_data_callback = dummyradio_add_data_recv_callback,
    .rem_receive_data_callback = dummyradio_rem_data_recv_callback,
    .get_option = dummyradio_get_option,
    .set_option = dummyradio_set_option,
    .get_state = dummyradio_get_state,
    .set_state = dummyradio_set_state,
    .event = dummyradio_event,
    .transmit = dummyradio_transmit_tx_buf,
    .send = netdev_802154_send,
    .add_receive_raw_callback = dummyradio_add_raw_recv_callback,
    .rem_receive_raw_callback = dummyradio_rem_raw_recv_callback,
    .channel_is_clear = dummyradio_channel_is_clear,
};

netdev_t dummyradio_netdev = { NETDEV_TYPE_802154, (netdev_driver_t *) &dummyradio_driver, NULL };
