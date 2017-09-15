/*
 * Copyright (c) 2016, Uppsala University, Sweden.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Kasun Hewage
 *          
 */
 
/**
 * \defgroup glossy Glossy API for cc2538 SoC
 * @{
 * \file   glossy.h
 * \file   glossy.c
 */

#ifndef GLOSSY_H_
#define GLOSSY_H_

#include <inttypes.h>
#include "contiki.h"

/**
 * Ratio between the frequencies of the MAC timer and the low-frequency clocks
 */
#define CLOCK_PHI                     (sys_ctrl_get_sys_clock() / RTIMER_SECOND)


enum {
  GLOSSY_UNKNOWN_INITIATOR = 0
};

enum {
  GLOSSY_UNKNOWN_N_TX_MAX = 0
};

enum {
  GLOSSY_UNKNOWN_PAYLOAD_LEN = 0
};

typedef enum {
  GLOSSY_ENC_OFF = 0,
  GLOSSY_ENC_ON = 1
} glossy_enc_t;

typedef enum {
  GLOSSY_AES_128_KEY_SIZE = 1,
  GLOSSY_AES_192_KEY_SIZE = 2,
  GLOSSY_AES_256_KEY_SIZE = 3
} glossy_aes_key_size_t;

typedef enum {
  GLOSSY_UNKNOWN_SYNC = 0x00,
  GLOSSY_WITH_SYNC = 0x10,
  GLOSSY_WITHOUT_SYNC = 0x20,
  GLOSSY_ONLY_RELAY_CNT = 0x30
} glossy_sync_t;

/**
 * Return status of Glossy API
 */
typedef enum {
  GLOSSY_STATUS_FAIL = 0,
  GLOSSY_STATUS_SUCCESS = 1,
} glossy_status_t;

/**
 * List of possible Glossy states.
 */
typedef enum {
    GLOSSY_STATE_OFF,
    GLOSSY_STATE_ACTIVE
} glossy_state_t;

typedef struct {
  uint16_t rx_timeout;
  uint16_t bad_length;
  uint16_t bad_i_header;
  uint16_t bad_crc;
  uint16_t bad_mac;
  uint16_t enc_dec_errs;
  uint16_t bad_g_header;
  uint16_t payload_mismatch;
  uint16_t rf_errs;
  uint16_t rx_cnt;
  uint16_t tx_cnt;
} glossy_stats_t;

extern volatile uint16_t node_id;

/***
 * @brief  Initialize Glossy internals
 * @return GLOSSY_STATUS_SUCCESS if the initialization is successful. Otherwise GLOSSY_STATUS_FAIL.
 */
glossy_status_t glossy_init(void);


/**
 * @brief       start Glossy
 * @param[in]   initiator_id node ID of the initiator, use
 *              GLOSSY_UNKNOWN_INITIATOR if the initiator is unknown
 * @param[in]   payload pointer to the packet data
 * @param[in]   payload_len length of the data, must not exceed PACKET_LEN_MAX
 * @param[in]   n_tx_max maximum number of retransmissions
 * @param[in]   sync synchronization mode
 * @note        n_tx_max must be at most 15!
 *
 * start Glossy, i.e. initiate a flood (if node is initiator) or switch to RX
 * mode (receive/relay packets)
 */
glossy_status_t glossy_start(uint16_t initiator_id,
                             uint8_t* payload,
                             uint8_t  payload_len,
                             uint8_t  n_tx_max,
                             glossy_sync_t sync);

/**
 * @brief            Stop Glossy and resume all other application tasks.
 * @return           Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 * @sa               get_rx_cnt
 */
uint8_t glossy_stop(void);

/**
 * @brief Enable/Disable encryption
 * @param enc Specifies if the encryption should be enabled. Use GLOSSY_ENC_ON to enable and
 *            GLOSSY_ENC_OFF to disable
 */
void glossy_set_enc(glossy_enc_t enc);

/**
 * @brief Set AES key to be used
 * @param key A pointer to the key
 * @param size The size of the key
 * @return GLOSSY_STATUS_SUCCESS if successful. Otherwise GLOSSY_STATUS_FAIL.
 */
glossy_status_t glossy_set_enc_key(uint8_t* key, glossy_aes_key_size_t key_size);

/**
 * @brief  Query activity of glossy
 * @return The number of received bytes since glossy_start was called
 */
uint8_t glossy_is_active(void);

/**
 * @brief Get the number of received packets during the last flood
 * @return Number of receptions during the last flood.
 */
uint8_t glossy_get_n_rx(void);

/**
 * @brief  Get the number of transmitted packets during the last flood
 * @return Number of transmissions during the last flood.
 */
uint8_t glossy_get_n_tx(void);

/**
 * @brief  Get the length of the payload of the received/transmitted packet
 * @return Size of the payload associated with last flood.
 */
uint8_t glossy_get_payload_len(void);

/**
 * @brief  Indicates if the reference time has been updated in the last flood
 * @return non-zero if reference time has been updated. Otherwise zero.
 * 
 */
uint8_t glossy_is_t_ref_updated(void);

/**
 * @brief  Get the reference time (timestamp of the reception of the first byte)
 * @return 64-bit timestamp (type rtimer_clock_t)
 */
rtimer_clock_t glossy_get_t_ref(void);

/**
 * @brief  Get the ID of the initiator
 * @return the ID of the initiator which started the last flood.
 */
uint16_t glossy_get_initiator_id(void);

/**
 * @brief  Get synchronization option of last Glossy flood.
 * @return the synchronization option of last Glossy flood.
 */
glossy_sync_t glossy_get_sync_opt(void);

/**
 * @brief Get relay count of the first reception
 * @return the relay count of the first reception in the last flood.
 */
uint8_t glossy_get_relay_cnt_first_rx(void);

/**
 * @brief Get maximum length of payload
 * @param  enc Encryption state
 * @return The maximum length of payload
 */
uint8_t glossy_get_max_payload_len(glossy_enc_t enc);

/**
 * @brief Get Glossy statistics
 * @param stats A pointer to statistics structure.
 */
void glossy_get_stats(glossy_stats_t* stats);

/**
 * @brief Get the last error associated wit the RF core of CC2538
 * @return The value of the RFCORE_SFR_RFERRF register
 */
uint32_t glossy_get_last_rf_error(void);

/**
 * @brief Print Glossy debug information.
 */
void glossy_debug_print(void);


#endif /* GLOSSY_H_ */

/** @} */
