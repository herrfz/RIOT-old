/*
 * Copyright (C) 2015 Eriza Fazli
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_dummyradio dummyradio
 * @ingroup     drivers
 * @brief       Device driver for the DUMMYRADIO radio
 * @{
 *
 * @file
 * @brief       Interface definition for the DUMMYRADIO device driver
 *
 * @author      Eriza Fazli <erizzaaaaa@gmail.com>
 */

#ifndef DUMMYRADIO_H_
#define DUMMYRADIO_H_

#include <stdio.h>
#include <stdint.h>

#include "kernel_types.h"
#include "board.h"
#include "radio/types.h"
#include "ieee802154_frame.h"
#include "dummyradio/dummyradio_settings.h"
#include "periph/gpio.h"
#include "netdev/802154.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Maximum length of a frame on dummyradio
 */
#define DUMMYRADIO_MAX_PKT_LENGTH    (127)

/**
 * @brief Maximum payload length
 * @details Assuming intra PAN short address mode
 *          results in 2 bytes FCF
 *          + 1 bytes SEQNr
 *          + 2 bytes PAN Id
 *          + 2 bytes destination address
 *          + 2 bytes source address
 */
#define DUMMYRADIO_MAX_DATA_LENGTH   (118)

/**
 * @brief Broadcast address
 */
#define DUMMYRADIO_BROADCAST_ADDRESS (0xFFFF)

/**
 * @brief dummyradio's lowest supported channel
 */
#define DUMMYRADIO_MIN_CHANNEL       (11)

/**
 * @brief dummyradio's highest supported channel
 */
#define DUMMYRADIO_MAX_CHANNEL       (26)

/**
 *  @brief Structure to represent a dummyradio packet.
 */
typedef struct __attribute__((packed))
{
    /** @{ */
    uint8_t length;             /**< the length of the frame of the frame including fcs*/
    ieee802154_frame_t frame;   /**< the ieee802154 frame */
    uint8_t rssi;               /**< the rssi value */
    uint8_t crc;                /**< 1 if crc was successfull, 0 otherwise */
    uint8_t lqi;                /**< the link quality indicator */
    /** @} */
} dummyradio_packet_t;

extern netdev_t dummyradio_netdev;   /**< netdev representation of this driver */

/**
 * @brief States to be assigned to `driver_state`
 * @{
 */
#define AT_DRIVER_STATE_DEFAULT     (0)
#define AT_DRIVER_STATE_SENDING     (1)
/** @} */

/**
 * @brief To keep state inside of dummyradio driver
 * @details This variable is used to determine if a TRX_END IRQ from
 *          the radio transceiver has to be interpreted as end of
 *          sending or reception.
 */
extern uint8_t driver_state;

/**
 * @brief Initialize the dummyradio transceiver
 */
int dummyradio_initialize(netdev_t *dev);

#ifdef MODULE_TRANSCEIVER
/**
 * @brief Init the dummyradio for use with RIOT's transceiver module.
 *
 * @param[in] tpid The PID of the transceiver thread.
 */

void dummyradio_init(kernel_pid_t tpid);
#endif

/**
 * @brief Turn dummyradio on.
 *
 * @return 1 if the radio was correctly turned on; 0 otherwise.
 */
int dummyradio_on(void);

/**
 * @brief Turn dummyradio off.
 */
void dummyradio_off(void);

/**
 * @brief Indicate if the dummyradio is on.
 *
 * @return 1 if the radio transceiver is on (active); 0 otherwise.
 */
int dummyradio_is_on(void);

/**
 * @brief Switches the dummyradio into receive mode.
 */
void dummyradio_switch_to_rx(void);

/**
 * @brief Turns monitor (promiscuous) mode on or off.
 *
 * @param[in] mode The desired mode:
 *                 1 for monitor (promiscuous) mode;
 *                 0 for normal (auto address-decoding) mode.
 */
void dummyradio_set_monitor(int mode);

/**
 * @brief Indicate if the dummyradio is in monitor (promiscuous) mode.
 *
 * @return 1 if the transceiver is in monitor (promiscuous) mode;
 *         0 if it is in normal (auto address-decoding) mode.
 */
int dummyradio_get_monitor(void);

/**
 * @brief Set the channel of the dummyradio.
 *
 * @param[in] chan The desired channel, valid channels are from 11 to 26.
 *
 * @return The tuned channel after calling, or -1 on error.
 */
int dummyradio_set_channel(unsigned int chan);

/**
 * @brief Get the channel of the dummyradio.
 *
 * @return The tuned channel.
 */
unsigned int dummyradio_get_channel(void);

/**
 * @brief Sets the short address of the dummyradio.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint16_t dummyradio_set_address(uint16_t addr);

/**
 * @brief Gets the current short address of the dummyradio.
 *
 * @return The current short address.
 */
uint16_t dummyradio_get_address(void);

/**
 * @brief Sets the IEEE long address of the dummyradio.
 *
 * @param[in] addr The desired address.
 *
 * @return The set address after calling.
 */
uint64_t dummyradio_set_address_long(uint64_t addr);

/**
 * @brief Gets the current IEEE long address of the dummyradio.
 *
 * @return The current IEEE long address.
 */
uint64_t dummyradio_get_address_long(void);

/**
 * @brief Sets the pan ID of the dummyradio.
 *
 * @param[in] pan The desired pan ID.
 *
 * @return The set pan ID after calling.
 */
uint16_t dummyradio_set_pan(uint16_t pan);

/**
 * @brief Gets the current IEEE long address of the dummyradio.
 *
 * @return The current IEEE long address.
 */
uint16_t dummyradio_get_pan(void);

/**
 * @brief Sets the output (TX) power of the dummyradio.
 *
 * @param[in] pow The desired TX (output) power in dBm,
 *                 valid values are -25 to 0; other values
 *                 will be "saturated" into this range.
 *
 * @return The set TX (output) power after calling.
 */
int dummyradio_set_tx_power(int pow);

/**
 * @brief Gets the current output (TX) power of the dummyradio.
 *
 * @return The current TX (output) power.
 */
int dummyradio_get_tx_power(void);

/**
 * @brief Checks if the radio medium is available/clear to send
 *         ("Clear Channel Assessment" a.k.a. CCA).
 *
 * @return a 1 value if radio medium is clear (available),
 *         a 0 value otherwise.
 *
 */
int dummyradio_channel_is_clear(netdev_t *dev);

/**
 * @brief Interrupt handler, gets fired when a RX overflow happens.
 *
 */
void dummyradio_rxoverflow_irq(void);

/**
 * @brief Interrupt handler, gets fired when bytes in the RX FIFO are present.
 *
 */
void dummyradio_rx_irq(void);

/**
 * @brief Sets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function for 802.15.4 packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int dummyradio_add_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Unsets the function called back when a packet is received.
 *        (Low-level mechanism, parallel to the `transceiver` module).
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int dummyradio_rem_raw_recv_callback(netdev_t *dev,
                                 netdev_802154_raw_packet_cb_t recv_cb);

/**
 * @brief Sets a function called back when a data packet is received.
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function for 802.15.4 data packet arrival
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int dummyradio_add_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb);

/**
 * @brief Unsets a function called back when a data packet is received.
 *
 * @param[in] dev     The network device to operate on. (Currently not used)
 * @param[in] recv_cb callback function to unset
 *
 * @return  0 on success
 * @return  -ENODEV if *dev* is not recognized
 * @return  -ENOBUFS, if maximum number of registable callbacks is exceeded
 */
int dummyradio_rem_data_recv_callback(netdev_t *dev,
                                  netdev_rcv_data_cb_t recv_cb);

/**
 * @brief RX handler, process data from the RX FIFO.
 *
 */
void dummyradio_rx_handler(void);

/**
 * @brief Transmit the data loaded into the dummyradio TX buffer.
 *
 * @param[in] dev The network device to operate on. (Currently not used)
 *
 * @return @ref netdev_802154_tx_status_t
 */
netdev_802154_tx_status_t dummyradio_transmit_tx_buf(netdev_t *dev);

/**
 * @brief Send function, sends a dummyradio_packet_t over the air.
 *
 * @param[in] *packet The Packet which will be send.
 *
 * @return The count of bytes which are send or -1 on error
 *
 */
int16_t dummyradio_send(dummyradio_packet_t *packet);

/**
 * RX Packet Buffer, read from the transceiver, filled by the dummyradio_rx_handler.
 */
extern dummyradio_packet_t dummyradio_rx_buffer[DUMMYRADIO_RX_BUF_SIZE];

/**
 * Get dummyradio's status byte
 */
uint8_t dummyradio_get_status(void);

/**
 * Get dummyradio's TRAC status byte
 */
uint8_t dummyradio_get_trac_status(void);

/**
 * dummyradio low-level radio driver definition.
 */
extern const netdev_802154_driver_t dummyradio_driver;

#ifdef __cplusplus
}
#endif

#endif /* DUMMYRADIO_H_ */
/** @} */
