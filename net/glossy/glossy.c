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
 
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>

#include "glossy.h"
#include "cc2538-rf.h"

#include "contiki.h"
#include "sys/energest.h"
#include "dev/leds.h"
#include "dev/watchdog.h"

#include "lib/memb.h"
#include "lib/list.h"

#include "dev/rfcore.h"
#include "dev/sys-ctrl.h"
#include "dev/udma.h"
#include "reg.h"
#include "dev/cctest.h"
#include "dev/ioc.h"
#include "dev/crypto.h"
#include "dev/ccm.h"

#if GLOSSY_DEBUG_GPIO
#define GLOSSY_DEBUG_GPIO_PORT_RF_ON                 GPIO_A_NUM
#define GLOSSY_DEBUG_GPIO_PIN_RF_ON                  6

#define GLOSSY_DEBUG_GPIO_PORT_SFD_TX                GPIO_A_NUM
#define GLOSSY_DEBUG_GPIO_PIN_SFD_TX                 7

#define GLOSSY_DEBUG_GPIO_PORT_SFD_RX                GPIO_A_NUM
#define GLOSSY_DEBUG_GPIO_PIN_SFD_RX                 7

#define GLOSSY_DEBUG_GPIO_PIN_INIT(port, pin) \
                                GPIO_SOFTWARE_CONTROL(GPIO_PORT_TO_BASE(port), \
                                                      GPIO_PIN_MASK(pin)); \
                                GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(port), \
                                                GPIO_PIN_MASK(pin)); \
                                GLOSSY_DEBUG_GPIO_UNSET_PIN(port, pin);


#define GLOSSY_DEBUG_GPIO_SET_PIN(port, pin) \
                                GPIO_WRITE_PIN(GPIO_PORT_TO_BASE(port), \
                                              GPIO_PIN_MASK(pin), \
                                              GPIO_PIN_MASK(pin))
#define GLOSSY_DEBUG_GPIO_UNSET_PIN(port, pin) \
                                GPIO_WRITE_PIN(GPIO_PORT_TO_BASE(port), \
                                              GPIO_PIN_MASK(pin), 0)


#define GLOSSY_DEBUG_GPIO_PIN_RF_ON_INIT() \
                                GLOSSY_DEBUG_GPIO_PIN_INIT(GLOSSY_DEBUG_GPIO_PORT_RF_ON, \
                                                        GLOSSY_DEBUG_GPIO_PIN_RF_ON);
#define GLOSSY_DEBUG_GPIO_SET_PIN_RF_ON() \
                                GLOSSY_DEBUG_GPIO_SET_PIN(GLOSSY_DEBUG_GPIO_PORT_RF_ON, \
                                                       GLOSSY_DEBUG_GPIO_PIN_RF_ON)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_RF_ON() \
                                GLOSSY_DEBUG_GPIO_UNSET_PIN(GLOSSY_DEBUG_GPIO_PORT_RF_ON, \
                                                         GLOSSY_DEBUG_GPIO_PIN_RF_ON)

#define GLOSSY_DEBUG_GPIO_PIN_SFD_TX_INIT() \
                                GLOSSY_DEBUG_GPIO_PIN_INIT(GLOSSY_DEBUG_GPIO_PORT_SFD_TX, \
                                                        GLOSSY_DEBUG_GPIO_PIN_SFD_TX);
#define GLOSSY_DEBUG_GPIO_SET_PIN_SFD_TX() \
                                GLOSSY_DEBUG_GPIO_SET_PIN(GLOSSY_DEBUG_GPIO_PORT_SFD_TX, \
                                                       GLOSSY_DEBUG_GPIO_PIN_SFD_TX)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_TX() \
                                GLOSSY_DEBUG_GPIO_UNSET_PIN(GLOSSY_DEBUG_GPIO_PORT_SFD_TX, \
                                                         GLOSSY_DEBUG_GPIO_PIN_SFD_TX)
#define GLOSSY_DEBUG_GPIO_PIN_SFD_RX_INIT() \
                                GLOSSY_DEBUG_GPIO_PIN_INIT(GLOSSY_DEBUG_GPIO_PORT_SFD_RX, \
                                                        GLOSSY_DEBUG_GPIO_PIN_SFD_RX);
#define GLOSSY_DEBUG_GPIO_SET_PIN_SFD_RX() \
                                GLOSSY_DEBUG_GPIO_SET_PIN(GLOSSY_DEBUG_GPIO_PORT_SFD_RX, \
                                                       GLOSSY_DEBUG_GPIO_PIN_SFD_RX)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_RX() \
                                GLOSSY_DEBUG_GPIO_UNSET_PIN(GLOSSY_DEBUG_GPIO_PORT_SFD_RX, \
                                                         GLOSSY_DEBUG_GPIO_PIN_SFD_RX)
#else

#define GLOSSY_DEBUG_GPIO_PIN_INIT(...)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN(...)
#define GLOSSY_DEBUG_GPIO_PIN_RF_ON_INIT(...)
#define GLOSSY_DEBUG_GPIO_SET_PIN_RF_ON(...)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_RF_ON(...)
#define GLOSSY_DEBUG_GPIO_PIN_SFD_TX_INIT(...)
#define GLOSSY_DEBUG_GPIO_SET_PIN_SFD_TX(...)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_TX(...)
#define GLOSSY_DEBUG_GPIO_PIN_SFD_RX_INIT(...)
#define GLOSSY_DEBUG_GPIO_SET_PIN_SFD_RX(...)
#define GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_RX(...)

#endif /* GLOSSY_DEBUG_GPIO */

#if GLOSSY_DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#ifdef GLOSSY_CONF_RX_MAJORITY_VOTE
#define GLOSSY_RX_MAJORITY_VOTE   GLOSSY_CONF_RX_MAJORITY_VOTE
#else
#define GLOSSY_RX_MAJORITY_VOTE   0
#endif

/*
 * The Glossy frame format with byte offsets of different fields (without encryption)
 * 0          4     5     6         7                                 2 bytes
 * +--------------------------------------------------------------------------+
 * | PREAMBLE | SFD | LEN | IHEADER | GLOSSY HEADER | GLOSSY PAYLOAD | FOOTER |
 * +--------------------------------------------------------------------------+
 */

#define GLOSSY_MIN_PKT_LEN_PLAIN      6   // ID header (1) + glossy header (>3) + FCS (2) = 6
#define GLOSSY_PROCESSING_TIME        50  // in us
#define GLOSSY_N_TX_MAX_GLOBAL        8   // Absolute maximum number of transmissions
#define GLOSSY_BUFFER_LEN             130

#define USECONDS_TO_MT_TICKS(us)      ((us) * 32) // FIXME: Make this to be configured from MAC timer clock
#define BYTES_TIME_TO_MT_TICKS(len)   (USECONDS_TO_MT_TICKS((len) * 32))
#define BYTES_TO_USECONDS(len)        ((len) * 32)

#define RF_TRUNAROUND_TIME            192 // in us
#define RF_DATA_LEN_FIELD_LEN         1

/*
 * Identification header format with bit offsets
 * 8                 2          1          0
 * +---------------------------------------+
 * | Glossy ID Magic | reserved | enc flag |
 * +---------------------------------------+
 */

#define BUF_TXRX_IHEADER_OFFSET         0
#define BUF_TXRX_IHEADER_FIELD          tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET]
#define BUF_TXRX_G_PKT_OFFSET           1
#define BUF_TXRX_G_PAYLOAD_OFFSET(cfg)  BUF_TXRX_G_PKT_OFFSET + GET_GLOSSY_HEADER_LEN(cfg)

#define IHEADER_MAGIC                   0xa8
#define IHEADER_MAGIC_MASK              0xfc
#define IHEADER_ENC_FLAG                0x01
#define IHEADER_ENC_FLAG_MASK           0x01
#define IHEADER_LEN                     1

#define GET_IHEADER_MAGIC(id)           ((id) & IHEADER_MAGIC_MASK)
#define SET_IHEADER_MAGIC(id)           (id) = ((id) & ~IHEADER_MAGIC_MASK) | IHEADER_MAGIC
#define GET_IHEADER_ENC_FLAG(id)        ((id) & IHEADER_ENC_FLAG_MASK)
#define SET_IHEADER_ENC_FLAG(id)        (id) = ((id) & ~IHEADER_ENC_FLAG_MASK) | IHEADER_ENC_FLAG
#define CLR_IHEADER_ENC_FLAG(id)        (id) &= ~IHEADER_ENC_FLAG_MASK

/*
 * Configuration word format with bit offsets
 * 8          6             4          0
 * +-----------------------------------+
 * | reserved | sync option | n max tx |
 * +-----------------------------------+
 */

typedef struct {
  uint16_t initiator_id;  /**< ID of the initiator */
  uint8_t  config;        /**< Configuration word. See above for the format */
  uint8_t  relay_cnt;     /**< This is optional   */
} glossy_header_t;

#define GLOSSY_HEADER_SYNC_OPT_MASK             0x30
#define GLOSSY_HEADER_N_MAX_TX_MASK             0x0f
#define BUF_PLAIN_HEADER_OFFSET                 0

#define GET_GLOSSY_HEADER_SYNC_OPT(cfg)         ((cfg) & GLOSSY_HEADER_SYNC_OPT_MASK)

#define SET_GLOSSY_HEADER_SYNC_OPT(cfg, opt) \
    (cfg) = ((cfg) & ~GLOSSY_HEADER_SYNC_OPT_MASK) | (opt)

#define GET_GLOSSY_HEADER_N_MAX_TX(cfg)         ((cfg) & GLOSSY_HEADER_N_MAX_TX_MASK)

#define SET_GLOSSY_HEADER_N_MAX_TX(cfg, n_max_tx) \
    (cfg) = ((cfg) & ~GLOSSY_HEADER_N_MAX_TX_MASK) | (n_max_tx)

#define FOOTER_LEN                              2
#define FOOTER1_CRC_OK                          0x80
#define FOOTER1_CORRELATION                     0x7f
#define FOOTER1_RSSI_FIELD                      g_cntxt.tx_rx_buffer[g_cntxt.tx_rx_len - 2]
#define FOOTER1_CRC_FIELD                       g_cntxt.tx_rx_buffer[g_cntxt.tx_rx_len - 1]

#define GET_GLOSSY_HEADER_LEN(cfg)              (GET_GLOSSY_HEADER_SYNC_OPT(cfg) == GLOSSY_WITHOUT_SYNC ? 3 : 4)

#define WITH_SYNC(cfg)                          (GET_GLOSSY_HEADER_SYNC_OPT(cfg) == GLOSSY_WITH_SYNC)

#define WITH_RELAY_CNT(cfg)                     ((WITH_SYNC(cfg)) || (GET_GLOSSY_HEADER_SYNC_OPT(cfg) == GLOSSY_ONLY_RELAY_CNT))

#define IS_INITIATOR()                          (g_cntxt.crr_header.initiator_id == node_id)

#define BUF_PLAIN_DATA_OFFSET(cfg)              (BUF_PLAIN_HEADER_OFFSET + GET_GLOSSY_HEADER_LEN(cfg))


#define IRQ_PRIORITY_GROUPING                   0
#define N_IRQ_PRIORITY_VALS                     12

#define IRQ_PRIORITY_IDX_SysTick_IRQ            0
#define IRQ_PRIORITY_IDX_SMT_IRQ                1
#define IRQ_PRIORITY_IDX_MACT_IRQ               2
#define IRQ_PRIORITY_IDX_RF_TX_RX_IRQ           3
#define IRQ_PRIORITY_IDX_RF_ERR_IRQ             4
#define IRQ_PRIORITY_IDX_UDMA_SW_IRQ            5
#define IRQ_PRIORITY_IDX_UDMA_ERR_IRQ           6
#define IRQ_PRIORITY_IDX_USB_IRQ                7
#define IRQ_PRIORITY_IDX_AES_IRQ                8
#define IRQ_PRIORITY_IDX_PKA_IRQ                9
#define IRQ_PRIORITY_IDX_UART0_IRQ              10
#define IRQ_PRIORITY_IDX_UART1_IRQ              11

#define GLOSSY_SEC_MAC_LEN                      16
#define GLOSSY_SEC_NONCE_LEN                    13
#define GLOSSY_SEC_AES_LEN_LEN                  2
#define GLOSSY_SEC_KEY_AREA                     0

typedef struct {
  uint8_t data[GLOSSY_BUFFER_LEN];
  uint8_t len;
  uint8_t count;
} glossy_payload_t;


typedef struct {

  uint64_t sfd_time;               /**< MAC timer timestamp when the SFD is received or sent. */
  uint64_t t_tx_start;             /**< MAC timer timestamp when the current TX is started. */
  uint64_t t_rx_start;             /**< MAC timer timestamp when the current RX is started. */
  uint64_t t_first_rx;             /**< MAC timer timestamp when first RX is started. */

  uint8_t saved_buffer[GLOSSY_BUFFER_LEN]; /**< Hold plain MPDU for retransmissions */
  uint8_t tx_rx_buffer[GLOSSY_BUFFER_LEN]; /**< Holds TX and RX MPDU */
  uint8_t tx_rx_len;                       /**< The size of MPDU in the TX and RX buffer */

  uint8_t  bytes_read;             /**< Number of bytes read from the RX FIFO of the radio */
  uint8_t  g_pkt_len;              /**< Length of Glossy packet including Glossy header.
                                        doesn't include the size of MAC + NONCE */
  uint8_t  tx_cnt;                 /**< Number of times that the Glossy frame is sent. */
  uint8_t  rx_cnt;                 /**< Number of times that the Glossy frame is received.*/

  uint64_t        t_ref_mtt;       /**< MAC timer timestamp when first RX or TX started. */
  uint8_t         t_ref_updated;   /**< A flag indicates if the reference time is updated. */
  uint8_t         relay_cnt_t_ref; /**< Relay count when the first packet is received. */
  rtimer_clock_t  t_ref_rt;        /**< This is the real-time clock timestamp when first RX or TX started. */

  uint64_t T_slot_estimated;       /**< An estimation of the slot length based on the packet length
                                        after the first transmission/reception */
  uint64_t T_slot_sum;             /**< Summation of slot times. */
  uint8_t  n_T_slots;              /**< Number of slots in the Glossy flood. */

  uint8_t         id_header;       /**< Identification header */
  glossy_header_t crr_header;      /**< Current Glossy header */

  uint8_t         relay_cnt_last_rx;  /**< Last received relay count. */
  uint8_t         relay_cnt_last_tx;  /**< Last sent relay count. */

  uint8_t* payload;               /**< A pointer to the Glossy's payload */
  uint8_t payload_len;            /**< Holds the length of the GLossy's payload */

#if GLOSSY_RX_MAJORITY_VOTE
  glossy_payload_t rx_payload[GLOSSY_N_TX_MAX_GLOBAL];
  uint8_t rx_payload_cnt;
#endif

  glossy_stats_t stats;            /**< Glossy statistics */
  uint32_t rf_err_reg_last;        /**< Content of RF_ERR register (for debugging) */

  uint8_t irq_priority_grouping;
  uint8_t irq_priorities[N_IRQ_PRIORITY_VALS];

  glossy_enc_t enc;                    /**< State if AES encryption is enabled/disabled. */
  uint8_t nonce[GLOSSY_SEC_NONCE_LEN]; /**< Holds the NONCE */
  uint8_t mac[GLOSSY_SEC_MAC_LEN];     /**< Holds the MAC of the encrypted data */

  volatile glossy_state_t state;

} glossy_context_t;

static glossy_context_t g_cntxt;

static void process_received_data();
static inline void mt_disable_cmp_events(void);
static inline void radio_abort_tx(void);

/* ---------------------------------------------------------------------------------------------- */
/**
 * @brief Add the value b to NONCE
 * @param nn A pointer to the starting byte of the NONCE
 * @param b The value to be added to the NONCE
 * @return carry bit after the adding
 */
static uint8_t add_to_nonce(uint8_t* nn, uint8_t b)
{
  uint8_t i;
  uint8_t carry = 0; // This is either 1 or 0
  uint8_t nni;

  carry = ((nni = nn[0] + b) < nn[0]) ? 1 : 0;
  nn[0] = nni;
  for (i = 1; i < GLOSSY_SEC_NONCE_LEN; i++) {
    if ((nni = nn[i] + carry) < carry) {
      carry = 1;
    } else {
      carry = 0;
    }

    nn[i] = nni;
  }
  return carry;
}

/* ---------------------------------------------------------------------------------------------- */
static inline void copy_to_rf_fifo(void)
{
  uint8_t i;

  /* Flush TXFIFO before copying the new packet */
  CC2538_RF_CSP_ISFLUSHTX();
  /* Copy the length of data first */
  REG(RFCORE_SFR_RFDATA) = g_cntxt.tx_rx_len;
  /* Copy data to the TXFIFO. */
  for (i = 0; i < g_cntxt.tx_rx_len - FOOTER_LEN; i++) {
    REG(RFCORE_SFR_RFDATA) = g_cntxt.tx_rx_buffer[i];
  }
}

/* ---------------------------------------------------------------------------------------------- */
static inline void copy_from_rf_fifo(uint8_t nbytes)
{
  uint8_t i;

  nbytes = nbytes <= REG(RFCORE_XREG_RXFIFOCNT) ? nbytes : REG(RFCORE_XREG_RXFIFOCNT);

  for (i = 0; i < nbytes; i++) {
    g_cntxt.tx_rx_buffer[g_cntxt.bytes_read + i] = REG(RFCORE_SFR_RFDATA);
  }
  g_cntxt.bytes_read += nbytes;
}

/* ---------------------------------------------------------------------------------------------- */
static void encryption_done(void)
{
  uint8_t ret;

  crypto_set_isr_callback(NULL);
  /* If successful MAC is just after the encrypted data */
  ret = ccm_auth_encrypt_get_result(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                                  g_cntxt.g_pkt_len],
                                    GLOSSY_SEC_MAC_LEN);
  if(ret != CRYPTO_SUCCESS) {
    PRINTF("encryption_done() ccm_auth_encrypt_get_result(): error %u\n", ret);
    radio_abort_tx();
    g_cntxt.stats.enc_dec_errs++;
    return;
  }
  /* Copy the encrypted data to the RF FIFO */
  copy_to_rf_fifo();

}

/* ---------------------------------------------------------------------------------------------- */
static glossy_status_t encryption_start(void)
{
  uint8_t ret;

  /* Increment NONCE by one to prevent collision attacks */
  add_to_nonce(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                     g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN],
               1);

  if(IS_INITIATOR()) {
    /* We are the initiator and we save the current tx_rx buffer before the encryption since
     * we may need to retransmit if we will not receive a packet in the next slot.
     */
    memcpy(g_cntxt.saved_buffer, g_cntxt.tx_rx_buffer, g_cntxt.tx_rx_len);
  }
  /* Set the callback to be called on AES interrupt */
  crypto_set_isr_callback(encryption_done);
  /* Start encryption */
  ret = ccm_auth_encrypt_start(GLOSSY_SEC_AES_LEN_LEN,
                               GLOSSY_SEC_KEY_AREA,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                                     g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN],
                               NULL,
                               0,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET],
                               g_cntxt.g_pkt_len,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET],
                               GLOSSY_SEC_MAC_LEN,
                               NULL);
  if(ret != CRYPTO_SUCCESS) {
    /* Starting encryption failed */
    PRINTF("encryption_start() ccm_auth_encrypt_start(): error %u\n", ret);
    return GLOSSY_STATUS_FAIL;
  }
  /* Enable AES interrupt */
  NVIC_ClearPendingIRQ(AES_IRQn);
  NVIC_EnableIRQ(AES_IRQn);

  return GLOSSY_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- */
static void decryption_done(void)
{
  uint8_t ret;

  /* Unset AES interrupt callback since we don't need it now */
  crypto_set_isr_callback(NULL);
  /* Retrieve the calculated MAC */
  ret = ccm_auth_decrypt_get_result(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET],
                                    g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN,
                                    g_cntxt.mac, GLOSSY_SEC_MAC_LEN);

  if(ret != CRYPTO_SUCCESS) {
    /* Result retrieving failed */
    PRINTF("decryption_done() ccm_auth_decrypt_get_result(): error %u\n", ret);
    radio_abort_tx();
    g_cntxt.stats.enc_dec_errs++;
    return;
  }

  if (memcmp(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET + g_cntxt.g_pkt_len],
             g_cntxt.mac, GLOSSY_SEC_MAC_LEN)) {
    /* Calculated MAC doesn't match with the received MAC */
    radio_abort_tx();
    g_cntxt.stats.bad_mac++;
    return;
  }
  /* We are good to go. */
  process_received_data();
}

/* ---------------------------------------------------------------------------------------------- */
static glossy_status_t decryption_start(void)
{
  uint8_t ret;

  if (g_cntxt.g_pkt_len < GLOSSY_SEC_MAC_LEN + GLOSSY_SEC_NONCE_LEN) {
    /* Not enough data for decryption */
    return GLOSSY_STATUS_FAIL;
  }
  /* Calculate the real glossy packet length */
  g_cntxt.g_pkt_len -= (GLOSSY_SEC_MAC_LEN + GLOSSY_SEC_NONCE_LEN);
  /* Set the callback to be called on AES interrupt */
  crypto_set_isr_callback(decryption_done);
  /* We use the same buffer to store decrypted data */
  ret = ccm_auth_decrypt_start(GLOSSY_SEC_AES_LEN_LEN,
                               GLOSSY_SEC_KEY_AREA,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                                     g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN],
                               NULL,
                               0,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET],
                               g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN,
                               &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET],
                               GLOSSY_SEC_MAC_LEN,
                               NULL);

  if(ret != CRYPTO_SUCCESS) {
    /* Starting decryption failed */
    PRINTF("decryption_start() ccm_auth_decrypt_start(): error %u\n", ret);
    return GLOSSY_STATUS_FAIL;
  }
  /* Enable AES interrupt */
  NVIC_ClearPendingIRQ(AES_IRQn);
  NVIC_EnableIRQ(AES_IRQn);

  return GLOSSY_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- */
glossy_status_t glossy_set_enc_key(uint8_t* key, glossy_aes_key_size_t key_size)
{
  uint8_t ret;

  ret = aes_load_keys(key, key_size, 1, GLOSSY_SEC_KEY_AREA);
  if(ret != CRYPTO_SUCCESS) {
    PRINTF("glossy_set_enc_key() aes_load_keys(): error %u\n", ret);
    return GLOSSY_STATUS_FAIL;
  }

  return GLOSSY_STATUS_SUCCESS;
}

/* --------------------------- Radio functions -------------------------------------------------- */
static inline void radio_on(void)
{
  CC2538_RF_CSP_ISFLUSHRX();
  CC2538_RF_CSP_ISRXON();

  GLOSSY_DEBUG_GPIO_SET_PIN_RF_ON();

  ENERGEST_ON(ENERGEST_TYPE_LISTEN);

}

/* ---------------------------------------------------------------------------------------------- */
static inline void radio_off(void)
{
  /* Don't turn off if we are off as this will trigger a Strobe Error */
  if(REG(RFCORE_XREG_RXENABLE) != 0) {
    CC2538_RF_CSP_ISRFOFF();
  }

  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);

  GLOSSY_DEBUG_GPIO_UNSET_PIN_RF_ON();

}

/* ---------------------------------------------------------------------------------------------- */
static inline void radio_abort_rx(void)
{
  /* Flush RX FIFO */
  CC2538_RF_CSP_ISFLUSHRX();
}

/* ---------------------------------------------------------------------------------------------- */
static inline void radio_abort_tx(void)
{
  /* Reset CSP */
  cc2538_rf_csp_reset();
  /* Disable CSP events */
  mt_disable_cmp_events();
  /* Flush TX FIFO */
  CC2538_RF_CSP_ISFLUSHTX();
  /* Aborts ongoing transmission and forces an RX calibration. */
  CC2538_RF_CSP_ISRXON();
}

/* ---------------------------------------------------------------------------------------------- */
static inline void radio_start_tx(void)
{
  CC2538_RF_CSP_ISTXON();

  GLOSSY_DEBUG_GPIO_SET_PIN_RF_ON();

  /* NOTE:
   * Since we use CSP to initiate retransmissions, we cannot accurately timestamp when the transmission
   * is actually started. Use of a manual interrupt in CSP can be used as a marking point. But it is
   * not accurate. Therefore, we only look at how long the radio is turned on.
   */
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
}

/* ---------------------------------------------------------------------------------------------- */
/***
 * @brief Disable MAC timer and its events
 */
static inline void mt_disable_cmp_events(void)
{
  /* Clear pending interrupts */
  REG(RFCORE_SFR_MTIRQF) = 0;

  REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
  REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_COMPARE1M;
  REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE2M;
  REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_COMPARE2M;

  /* Disable MAC timer EVENT1 */
  REG(RFCORE_SFR_MTCSPCFG) |= RFCORE_SFR_MTCSPCFG_MACTIMER_EVENT1_CFG;
  /* Disable MAC timer EVENMT */
  REG(RFCORE_SFR_MTCSPCFG) |= RFCORE_SFR_MTCSPCFG_MACTIMER_EVENMT_CFG;

}

/* ---------------------------------------------------------------------------------------------- */
/**
 * @brief Schedule to fire MAC timer interrupt
 *
 *        This function take 96 cycles to execute as measured with debug cycle counter.
 *
 * @param t_start The number of MAC timer ticks
 */
static inline void mt_schedule(uint64_t t_start)
{
  /*
   * NOTE: Interrupt flags are set regardless of the interrupt masks.
   */
  /* Set MTMOVFSEL bits to 011 as we are going to set overflow compare 1 value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000030;
  REG(RFCORE_SFR_MTMOVF0) = ((uint32_t)(t_start >> 16)) & RFCORE_SFR_MTMOVF0_MTMOVF0;
  REG(RFCORE_SFR_MTMOVF1) = ((uint32_t)(t_start >> 24)) & RFCORE_SFR_MTMOVF1_MTMOVF1;
  REG(RFCORE_SFR_MTMOVF2) = ((uint32_t)(t_start >> 32)) & RFCORE_SFR_MTMOVF2_MTMOVF2;

  /* Set MTMSEL bits to 011 as we are going to set counter compare 1 value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000003;
  REG(RFCORE_SFR_MTM0) = ((uint32_t)t_start) & RFCORE_SFR_MTM0_MTM0;
  REG(RFCORE_SFR_MTM1) = ((uint32_t)(t_start >> 8)) & RFCORE_SFR_MTM1_MTM1;

  /* Clear pending interrupts */
  REG(RFCORE_SFR_MTIRQF) = 0;

  /* We check if we need to use overflow compare event.
   * Here, we account how many overflows we need in the near future.
   * We only use the overflow event if we need more than 1 overflows.
   * Otherwise, we just wait until the counter wraps and compare match to happen.
   * NOTE: We do this since there can be situations when we miss overflow compare match event
   * if the overflow happens in very soon (before enabling the overflow compare event).
   */
  uint64_t t_now = cc2538_rf_get_mac_time_now();
  if ((t_start >> 16) - (t_now >> 16) > 1) {
    REG(RFCORE_SFR_MTIRQM) |= RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
    REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_COMPARE1M;
    /* We enable counter compare interrupt when the overflow happens */
  } else {
    REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
    REG(RFCORE_SFR_MTIRQM) |= RFCORE_SFR_MTIRQM_MACTIMER_COMPARE1M;
  }

}

/* ---------------------------------------------------------------------------------------------- */
/**
 * @brief Schedule to start transmission from MAC timer event
 *
 *        This function take xx cycles to execute as measured with debug cycle counter.
 *
 * @param t_start
 */
static inline void mt_schedule_tx_csp(uint64_t t_start)
{
  /* Set MTMOVFSEL bits to 011 as we are going to set overflow compare 1 value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMOVFSEL) | 0x00000030;
  REG(RFCORE_SFR_MTMOVF0) = ((uint32_t)(t_start >> 16)) & RFCORE_SFR_MTMOVF0_MTMOVF0;
  REG(RFCORE_SFR_MTMOVF1) = ((uint32_t)(t_start >> 24)) & RFCORE_SFR_MTMOVF1_MTMOVF1;
  REG(RFCORE_SFR_MTMOVF2) = ((uint32_t)(t_start >> 32)) & RFCORE_SFR_MTMOVF2_MTMOVF2;

  /* Set MTMSEL bits to 011 as we are going to set counter compare 1 value */
  REG(RFCORE_SFR_MTMSEL) = (REG(RFCORE_SFR_MTMSEL) & ~RFCORE_SFR_MTMSEL_MTMSEL) | 0x00000003;
  REG(RFCORE_SFR_MTM0) = ((uint32_t)t_start) & RFCORE_SFR_MTM0_MTM0;
  REG(RFCORE_SFR_MTM1) = ((uint32_t)(t_start >> 8)) & RFCORE_SFR_MTM1_MTM1;

  /* Clear pending interrupts */
  REG(RFCORE_SFR_MTIRQF) = 0;

  /* Reset CSP */
  cc2538_rf_csp_reset();

  /* We check if we need to use overflow compare event.
   * Here, we account how many overflows we need in the near future.
   * We only use the overflow event if we need more than 1 overflows.
   * Otherwise, we just wait until the counter wraps and compare match to happen.
   * NOTE: We do this since there can be situations when we miss overflow compare match event
   * if the overflow happens in very soon (before enabling the overflow compare event).
   */
  uint64_t t_now = cc2538_rf_get_mac_time_now();
  if ((t_start >> 16) - (t_now >> 16) > 1) {
    /* Enable MAC timer event 1 (EVENT1) to be overflow compare match */
    REG(RFCORE_SFR_MTCSPCFG) = (REG(RFCORE_SFR_MTCSPCFG)
                               & ~RFCORE_SFR_MTCSPCFG_MACTIMER_EVENT1_CFG) | 0x00000004;
    /* Write instruction to wait until MAC timer overflow match happens (event 1) */
    REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_WEVENT1;
  }

  /* Enable MAC timer event 2 (EVENMT) to be counter compare match */
  REG(RFCORE_SFR_MTCSPCFG) = (REG(RFCORE_SFR_MTCSPCFG)
                             & ~RFCORE_SFR_MTCSPCFG_MACTIMER_EVENMT_CFG) | 0x00000010;
  /* Write instruction to wait until MAC timer counter match happens (event 2) */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_WEVENT2;
  /* Write STXON instruction to start transmission */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_STXON;
  /* Write SSTOP instruction to stop */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_SSTOP;
  /* Start executing CSP program */
  REG(RFCORE_SFR_RFST) = CC2538_RF_CSP_OP_ISSTART;
}

/* ---------------------------------------------------------------------------------------------- */
/**
 * @brief Updates slot time.
 *        As SFD time is captured by the MAC timer, slot time is simply the difference between
 *        reception start and transmission start
 * @return
 */
static inline void update_T_slot(uint64_t slot_time)
{
  g_cntxt.T_slot_sum += slot_time;
  g_cntxt.n_T_slots++;

}

/* ---------------------------------------------------------------------------------------------- */
static inline void update_t_ref(uint64_t t_ref, uint8_t relay_cnt)
{
  g_cntxt.t_ref_mtt = t_ref;
  g_cntxt.t_ref_updated = 1;
  g_cntxt.relay_cnt_t_ref = relay_cnt;
}


/* ---------------------------------------------------------------------------------------------- */
static glossy_status_t validate_glossy_header(glossy_header_t* rcvd_hdr)
{
  /* Received packet should have sync option specified */
  if ((GET_GLOSSY_HEADER_SYNC_OPT(g_cntxt.crr_header.config) == GLOSSY_UNKNOWN_SYNC)
      && (GET_GLOSSY_HEADER_SYNC_OPT(rcvd_hdr->config) == GLOSSY_UNKNOWN_SYNC)) {

    return GLOSSY_STATUS_FAIL;
  }

  /* If current sync option is known, it should match with the received one */
  if ((GET_GLOSSY_HEADER_SYNC_OPT(g_cntxt.crr_header.config) != GLOSSY_UNKNOWN_SYNC)
      && (GET_GLOSSY_HEADER_SYNC_OPT(g_cntxt.crr_header.config)
          != GET_GLOSSY_HEADER_SYNC_OPT(rcvd_hdr->config))) {

    return GLOSSY_STATUS_FAIL;
  }

  /* Received packet should have maximum number of transmissions specified */
  if ((GET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config) == GLOSSY_UNKNOWN_N_TX_MAX)
      && (GET_GLOSSY_HEADER_N_MAX_TX(rcvd_hdr->config) == GLOSSY_UNKNOWN_N_TX_MAX)) {

    return GLOSSY_STATUS_FAIL;
  }

  /* If current maximum number of transmissions is known, it should match with the received one */
  if ((GET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config) != GLOSSY_UNKNOWN_N_TX_MAX)
      && (GET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config)
          != GET_GLOSSY_HEADER_N_MAX_TX(rcvd_hdr->config))) {

    return GLOSSY_STATUS_FAIL;
  }

  /* Received packet should have initiator ID specified */
  if ((g_cntxt.crr_header.initiator_id == GLOSSY_UNKNOWN_INITIATOR)
      && (rcvd_hdr->initiator_id == GLOSSY_UNKNOWN_INITIATOR)) {

    return GLOSSY_STATUS_FAIL;
  }

  /* If current initiator ID is known, it should match with the received one */
  if ((g_cntxt.crr_header.initiator_id != GLOSSY_UNKNOWN_INITIATOR)
      && (g_cntxt.crr_header.initiator_id != rcvd_hdr->initiator_id)) {

    return GLOSSY_STATUS_FAIL;
  }

  return GLOSSY_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- */
static void process_received_data()
{
  /* Glossy header is at the beginning of decrypted data */
  glossy_header_t * rcvd_header = (glossy_header_t*)(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET]);

  if (validate_glossy_header(rcvd_header) != GLOSSY_STATUS_SUCCESS) {
    /* Glossy header validation failed */
    radio_abort_tx();
    g_cntxt.stats.bad_g_header++;
    return;
  }

  if (g_cntxt.rx_cnt == 0) {
    /* First successful reception. */
    g_cntxt.t_first_rx = g_cntxt.t_rx_start;
    /* Copy the received header to current header */
    memcpy(&g_cntxt.crr_header, rcvd_header, GET_GLOSSY_HEADER_LEN(rcvd_header->config));
  }

  /* Save the current received relay counter value */
  if (WITH_RELAY_CNT(g_cntxt.crr_header.config)) {
    g_cntxt.relay_cnt_last_rx = rcvd_header->relay_cnt;
  }

  if (WITH_SYNC(g_cntxt.crr_header.config)) {
    /* Glossy time synchronization enabled */
    if (!g_cntxt.t_ref_updated) {
      /* reference time has not been updated yet. So update it */
      update_t_ref(g_cntxt.t_rx_start, g_cntxt.relay_cnt_last_rx);
    }

    if ((g_cntxt.relay_cnt_last_rx == g_cntxt.relay_cnt_last_tx + 1) && g_cntxt.tx_cnt > 0) {
      /* This reception is just after a transmission. So we update the slot time */
      update_T_slot(g_cntxt.t_rx_start - g_cntxt.t_tx_start);
    }
  }

  g_cntxt.rx_cnt++;

#if GLOSSY_RX_MAJORITY_VOTE
  if(!IS_INITIATOR() && g_cntxt.rx_cnt < GLOSSY_N_TX_MAX_GLOBAL) {
    /* Glossy payload is just after the Glossy header */
    uint8_t app_data_len = g_cntxt.g_pkt_len - GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config);
    uint8_t* rx_app_data = &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                                 GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config)];
    uint8_t i;
    uint8_t found = 0;
    for (i = 0; i < g_cntxt.rx_payload_cnt; i++) {
      if (g_cntxt.rx_payload[i].len == app_data_len) {
        if (memcmp(g_cntxt.rx_payload[i].data, rx_app_data, app_data_len) == 0) {
          found = 1;
          g_cntxt.rx_payload[i].count++;
          break;
        }
      }
      g_cntxt.stats.payload_mismatch++;
    }
    if (!found) {
      /* Add a new entry */
      memcpy(g_cntxt.rx_payload[i].data, rx_app_data, app_data_len);
      g_cntxt.rx_payload[i].len = app_data_len;
      g_cntxt.rx_payload[i].count = 1;
      g_cntxt.rx_payload_cnt++;
    }
  }
#else
  if((!IS_INITIATOR()) && (g_cntxt.rx_cnt == 1)) {
    uint8_t app_data_len = g_cntxt.g_pkt_len - GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config);
    uint8_t* rx_app_data = &g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET +
                                                 GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config)];
    memcpy(g_cntxt.payload, rx_app_data, app_data_len);
    g_cntxt.payload_len = app_data_len;
  }
#endif /* GLOSSY_RX_MAJORITY_VOTE */


  if (g_cntxt.tx_cnt == GET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config)) {
    /* We don't need more transmissions */
    radio_abort_tx();
    /* Stop Glossy */
    glossy_stop();
    return;
  }

  if (WITH_RELAY_CNT(g_cntxt.crr_header.config)) {
    /* we need to increase the relay counter by one */
    rcvd_header->relay_cnt++;
    g_cntxt.relay_cnt_last_tx = rcvd_header->relay_cnt;
  }

  if (GET_IHEADER_ENC_FLAG(g_cntxt.id_header) == IHEADER_ENC_FLAG) {
    /* Need to encrypt the payload */
    if (encryption_start() != GLOSSY_STATUS_SUCCESS) {
      /* Starting encryption failed */
      radio_abort_tx();
      g_cntxt.stats.enc_dec_errs++;
    }
  } else {
    /* Just copy the data to the RF FIFO */
    copy_to_rf_fifo();
  }
}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_tx_started(void)
{
  g_cntxt.t_tx_start = g_cntxt.sfd_time;

  if (g_cntxt.tx_cnt == 0) {
    /* This is the first transmission.
     * We estimate slot length based on the packet length
     * Note that FCS length is also included in tx_rx_len
     */
    g_cntxt.T_slot_estimated = BYTES_TIME_TO_MT_TICKS(RF_DATA_LEN_FIELD_LEN + g_cntxt.tx_rx_len)
                               + USECONDS_TO_MT_TICKS(GLOSSY_PROCESSING_TIME)
                               + USECONDS_TO_MT_TICKS(RF_TRUNAROUND_TIME) /* RF turn around time */
                               + BYTES_TIME_TO_MT_TICKS(5); /* Time for preamble and SFD */

  }

}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_tx_ended(void)
{
  g_cntxt.tx_cnt++;

  if (WITH_SYNC(g_cntxt.crr_header.config)) {

    if (!g_cntxt.t_ref_updated) {
      update_t_ref(g_cntxt.t_tx_start, g_cntxt.relay_cnt_last_tx);
    }

    if ((g_cntxt.relay_cnt_last_tx == g_cntxt.relay_cnt_last_rx + 1) && g_cntxt.rx_cnt > 0) {
      /* This transmission is just after a reception. So we update the slot time */
      update_T_slot(g_cntxt.t_tx_start - g_cntxt.t_rx_start);
    }
  }

  /* Stop Glossy if tx_cnt reached tx_max */
  if ((g_cntxt.tx_cnt == GET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config))
      || (g_cntxt.tx_cnt == GLOSSY_N_TX_MAX_GLOBAL)) {
    /* Stop glossy */
    glossy_stop();

  } else {
    /* We need more transmissions */
    if (IS_INITIATOR() && g_cntxt.rx_cnt == 0) {
      /* Initiator hasn't received any packet yet.
       * Therefore, we are going to see if we will receive a packet in the next time slot.
       * We add 10 us to compensate any jitter.
       */
      uint64_t t_rx_start_expected = g_cntxt.t_tx_start + g_cntxt.T_slot_estimated
                                     + USECONDS_TO_MT_TICKS(10);
      mt_schedule(t_rx_start_expected);

    } else {
      /* This is not the initiator or the initiator has received packets previously */
      /* Reset CSP */
      cc2538_rf_csp_reset();
      CC2538_RF_CSP_ISFLUSHTX();
    }

  }

}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_rx_ended(void)
{
  uint64_t t_tx_start_new;

  /* Disable MAC timer events as we've received a complete packet */
  //mt_disable_cmp_events();

  /* Schedule the next transmission first.
   * We use the time when SFD received as the reference to schedule the next packet transmission.
   * If we don't need retransmissions further, we have to reset and disable CSP events later
   */
  t_tx_start_new = g_cntxt.t_rx_start;
  t_tx_start_new += BYTES_TIME_TO_MT_TICKS(RF_DATA_LEN_FIELD_LEN + g_cntxt.tx_rx_len);
  t_tx_start_new += USECONDS_TO_MT_TICKS(GLOSSY_PROCESSING_TIME);
  /* Schedule next transmission using MAC timer events and CSP */
  mt_schedule_tx_csp(t_tx_start_new);

  /* Read rest of the data from RXFIFO */
  copy_from_rf_fifo(g_cntxt.tx_rx_len - g_cntxt.bytes_read);

  /* We accept only one frame. So flush the RXFIFO */
  CC2538_RF_CSP_ISFLUSHRX();
  /* Check if the packet is not corrupted */
  if (!(FOOTER1_CRC_FIELD & FOOTER1_CRC_OK)) {
    radio_abort_tx();
    g_cntxt.stats.bad_crc++;
    return;
  }

  /* Calculate the new Glossy packet length */
  g_cntxt.g_pkt_len = g_cntxt.tx_rx_len - IHEADER_LEN - FOOTER_LEN;

  if (GET_IHEADER_ENC_FLAG(g_cntxt.id_header) == IHEADER_ENC_FLAG) {
    /* We have to decrypt the data */
    if (decryption_start() != GLOSSY_STATUS_SUCCESS) {
      /* Starting decryption failed */
      radio_abort_tx();
      g_cntxt.stats.enc_dec_errs++;
      return;
    }
    /* process_received_data() is called in decryption_done() */
  } else {
    /* Data is not encrypted */
    process_received_data();
  }
}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_rx_started(void)
{
  uint64_t t_rx_timeout;
  uint8_t tx_rx_len_tmp;

  g_cntxt.t_rx_start = g_cntxt.sfd_time;

  if(IS_INITIATOR()) {
    cc2538_rf_csp_reset();
    mt_disable_cmp_events();
  }
  /* 32 us to receive one byte. We should wait until at least 2 bytes are in the RXFIFO */
  t_rx_timeout = g_cntxt.t_rx_start + BYTES_TIME_TO_MT_TICKS(GLOSSY_MIN_PKT_LEN_PLAIN);
  /* Wait until the at least two byte time in order to proceed */
  while(!CC2538_RF_RXFIFO_HAS_DATA()) {
    if (cc2538_rf_get_mac_time_now() > t_rx_timeout) {
      /* We reached receive timeout. So abort the reception */
      radio_abort_rx();
      g_cntxt.stats.rx_timeout++;
      return;
    }
    watchdog_periodic();
  }

  /* Read the length byte */
  tx_rx_len_tmp = REG(RFCORE_SFR_RFDATA);
  /* Check for out of sync packets and minimum length */
  if( (tx_rx_len_tmp > CC2538_RF_MAX_PACKET_LEN) || (tx_rx_len_tmp < GLOSSY_MIN_PKT_LEN_PLAIN) ) {
    radio_abort_rx();
    g_cntxt.stats.bad_length++;
    return;
  }

#if GLOSSY_RX_MAJORITY_VOTE
  /* We receive a mismatched payload and take majority vote later when glossy is stopped.
   * Only the initiator expects the correct length all the time.
   */
  if (IS_INITIATOR() && g_cntxt.tx_rx_len != tx_rx_len_tmp) {
    radio_abort_rx();
    g_cntxt.stats.payload_mismatch++;
    return;
  }
#else
  /* Check if all packets we receive are in the same length regardless of who started the flood */
  if (g_cntxt.rx_cnt > 0 && g_cntxt.tx_rx_len != tx_rx_len_tmp) {
    radio_abort_rx();
    g_cntxt.stats.payload_mismatch++;
    return;
  }
#endif

  g_cntxt.tx_rx_len = tx_rx_len_tmp;

  g_cntxt.bytes_read = 0;
  /* Wait until the at least one more byte time in order to proceed */
  while(!CC2538_RF_RXFIFO_HAS_DATA()) {
    if (cc2538_rf_get_mac_time_now() > t_rx_timeout) {
      /* We reached receive timeout. So abort the reception */
      radio_abort_rx();
      g_cntxt.stats.rx_timeout++;
      return;
    }
    watchdog_periodic();
  }

  /* Read a byte to check Glossy identification header */
  g_cntxt.tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET] = REG(RFCORE_SFR_RFDATA);
  g_cntxt.bytes_read++;
  /* We keep receiving only if it has the right header */
  if ((GET_IHEADER_MAGIC(g_cntxt.tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET])) != IHEADER_MAGIC) {
    /* Wrong header: abort packet reception */
    radio_abort_rx();
    g_cntxt.stats.bad_i_header++;
    return;
  }

  /* Check if all packets we receive are with the same ID header */
  if (g_cntxt.rx_cnt > 0 && g_cntxt.id_header != g_cntxt.tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET]) {
    /* Wrong header: abort packet reception */
    radio_abort_rx();
    g_cntxt.stats.bad_i_header++;
    return;
  }

  g_cntxt.id_header = g_cntxt.tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET];

  /* We expect rest of the reception will continue without any problem even if we will receive a
   * corrupted packet. Therefore, we don't use any timeout for receiving of rest of the packet.
   * FIXME: Check if this would be enough. If it is not, use a MAC timer to schedule a reception
   *        timeout.
   */
}

/* ---------------------------------------------------------------------------------------------- */
/**
 * @brief The MAC Timer ISR
 *
 *        This is the interrupt service routine for MAC timer of CC2538
 *
 *        As the counter is 16-bit, first, we have to detect when the needed number of overflows happen.
 *        Then, we have to detect when the counter reaches to the needed value.
 */
void cc2538_rf_mt_isr(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);
  /* Interrupt flags are set regardless of the interrupt masks.
   */
  if((REG(RFCORE_SFR_MTIRQF) & RFCORE_SFR_MTIRQF_MACTIMER_OVF_COMPARE1F)) {
    REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_OVF_COMPARE1M;
    REG(RFCORE_SFR_MTIRQM) |= RFCORE_SFR_MTIRQM_MACTIMER_COMPARE1M;

  } else if((REG(RFCORE_SFR_MTIRQF) & RFCORE_SFR_MTIRQF_MACTIMER_COMPARE1F)) {
    REG(RFCORE_SFR_MTIRQM) &= ~RFCORE_SFR_MTIRQM_MACTIMER_COMPARE1M;
    /* This is where we should add what to do when the time elapsed */

    if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_SFD) {
     /* We are receiving something. So we avoid scheduling initiator retransmission */
      REG(RFCORE_SFR_MTIRQF) = 0;
      return;
    }
    /* Initiator hasn't received anything in the previous slot after the last transmission.
     * Therefore, we are going to re-transmit in next slot.
     * We use the time when SFD is sent in the previous transmission as the reference to
     * schedule the next packet retransmission.
     */

    /* We need to copy the saved buffer again to the tx_rx_buffer since we are going to modify
     * the relay counter
     */
    memcpy(g_cntxt.tx_rx_buffer, g_cntxt.saved_buffer, g_cntxt.tx_rx_len);

    glossy_header_t *rcvd_header = (glossy_header_t*)(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET]);

    /* If we need to use relay counter, we need to increase the relay counter by two */
    if (WITH_RELAY_CNT(g_cntxt.crr_header.config)) {
      rcvd_header->relay_cnt += 2;
      g_cntxt.relay_cnt_last_tx = rcvd_header->relay_cnt;
    }

    uint64_t t_tx_start_new = g_cntxt.t_tx_start;
    t_tx_start_new += g_cntxt.T_slot_estimated;
    t_tx_start_new += BYTES_TIME_TO_MT_TICKS(RF_DATA_LEN_FIELD_LEN + g_cntxt.tx_rx_len);
    t_tx_start_new += USECONDS_TO_MT_TICKS(GLOSSY_PROCESSING_TIME);

    mt_schedule_tx_csp(t_tx_start_new);

    if (GET_IHEADER_ENC_FLAG(g_cntxt.id_header) == IHEADER_ENC_FLAG) {
      /* Need to encrypt the payload */
      if (encryption_start() != GLOSSY_STATUS_SUCCESS) {
        /* Starting encryption failed */
        radio_abort_tx();
        g_cntxt.stats.enc_dec_errs++;
      }
    } else {
      /* Just copy to the RF FIFO */
      copy_to_rf_fifo();
    }

  }

  REG(RFCORE_SFR_MTIRQF) = 0;

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

/* ---------------------------------------------------------------------------------------------- */
/**
 * \brief The cc2538 RF RX/TX ISR
 *
 *        This is the interrupt service routine for all RF interrupts relating
 *        to RX and TX.
 */

void cc2538_rf_rx_tx_isr(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  /* Check for SFD to see if SFD is sent or received. Note that this interrupt is not fired
   * when SFD goes low.
   */
  if(REG(RFCORE_SFR_RFIRQF0) & RFCORE_SFR_RFIRQF0_SFD) {
#if GLOSSY_DEBUG_GPIO
    if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE) {
      GLOSSY_DEBUG_GPIO_SET_PIN_SFD_RX();
    } else if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE) {
      GLOSSY_DEBUG_GPIO_SET_PIN_SFD_TX();
    } else {
      /* ERROR */
    }
#endif /* GLOSSY_DEBUG_GPIO */

    g_cntxt.sfd_time = cc2538_rf_get_sfd_timestamp();

    if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE) {
      glossy_rx_started();
    } else if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_TX_ACTIVE) {
      glossy_tx_started();
    } else {
      /* ERROR */
    }
  }

  /* We have to copy rest of the frame before we are handling glossy_rx_ended() event */
  if(REG(RFCORE_SFR_RFIRQF0) & RFCORE_SFR_RFIRQF0_FIFOP) {
    copy_from_rf_fifo(CC2538_RF_RXFIFO_THRES);
  }

  /* There is no interrupt when SFD goes low. So we have to check for TXDONE/RXPKTDONE flags
   * to see if complete frame transmitted/received
   */

  if(REG(RFCORE_SFR_RFIRQF0) & RFCORE_SFR_RFIRQF0_RXPKTDONE) {
    if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE) {

      GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_RX();

      glossy_rx_ended();
    } else {
      /* ERROR */
    }
  }

  if(REG(RFCORE_SFR_RFIRQF1) & RFCORE_SFR_RFIRQF1_TXDONE) {
    if (REG(RFCORE_XREG_FSMSTAT1) & RFCORE_XREG_FSMSTAT1_RX_ACTIVE) {
#if GLOSSY_DEBUG_GPIO
      GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_TX();
#endif /* GLOSSY_DEBUG_GPIO */
      glossy_tx_ended();
    } else {
      /* ERROR */
    }
  }

  /* Clear pending interrupts */
  REG(RFCORE_SFR_RFIRQF0) = 0;
  REG(RFCORE_SFR_RFIRQF1) = 0;

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

/* ---------------------------------------------------------------------------------------------- */
/**
 * \brief The cc2538 RF Error ISR
 *
 *        This is the interrupt service routine for all RF errors.
 */
void
cc2538_rf_err_isr(void)
{
  ENERGEST_ON(ENERGEST_TYPE_IRQ);

  GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_RX();
  GLOSSY_DEBUG_GPIO_UNSET_PIN_SFD_TX();

  g_cntxt.stats.rf_errs++;
  g_cntxt.rf_err_reg_last = REG(RFCORE_SFR_RFERRF);

  /* Clear pending interrupts */
  REG(RFCORE_SFR_RFERRF) = 0;

  ENERGEST_OFF(ENERGEST_TYPE_IRQ);
}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_set_irq_priorities(void)
{
  g_cntxt.irq_priority_grouping = NVIC_GetPriorityGrouping();

  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_SysTick_IRQ] = NVIC_GetPriority(SysTick_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_SMT_IRQ] = NVIC_GetPriority(SMT_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_MACT_IRQ] = NVIC_GetPriority(MACT_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_RF_TX_RX_IRQ] = NVIC_GetPriority(RF_TX_RX_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_RF_ERR_IRQ] = NVIC_GetPriority(RF_ERR_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_AES_IRQ] = NVIC_GetPriority(AES_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_PKA_IRQ] = NVIC_GetPriority(PKA_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UDMA_SW_IRQ] = NVIC_GetPriority(UDMA_SW_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UDMA_ERR_IRQ] = NVIC_GetPriority(UDMA_ERR_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_USB_IRQ] = NVIC_GetPriority(USB_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UART0_IRQ] = NVIC_GetPriority(UART0_IRQn);
  g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UART1_IRQ] = NVIC_GetPriority(UART1_IRQn);


  NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);

  NVIC_SetPriority(UART0_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 0, 0));
  NVIC_SetPriority(UART1_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 0, 0));
  NVIC_SetPriority(RF_TX_RX_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 0, 0));
  NVIC_SetPriority(MACT_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 0, 1));

  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(SMT_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(RF_ERR_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(AES_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(PKA_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(UDMA_SW_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(UDMA_ERR_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));
  NVIC_SetPriority(USB_IRQn, NVIC_EncodePriority(IRQ_PRIORITY_GROUPING, 1, 0));


}

/* ---------------------------------------------------------------------------------------------- */
static inline void glossy_restore_irq_priorities(void)
{
  NVIC_SetPriorityGrouping(g_cntxt.irq_priority_grouping);

  NVIC_SetPriority(SysTick_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_SysTick_IRQ]);
  NVIC_SetPriority(SMT_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_SMT_IRQ]);
  NVIC_SetPriority(MACT_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_MACT_IRQ]);
  NVIC_SetPriority(RF_TX_RX_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_RF_TX_RX_IRQ]);
  NVIC_SetPriority(RF_ERR_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_RF_ERR_IRQ]);
  NVIC_SetPriority(AES_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_AES_IRQ]);
  NVIC_SetPriority(PKA_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_PKA_IRQ]);
  NVIC_SetPriority(UDMA_SW_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UDMA_SW_IRQ]);
  NVIC_SetPriority(UDMA_ERR_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UDMA_ERR_IRQ]);
  NVIC_SetPriority(USB_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_USB_IRQ]);
  NVIC_SetPriority(UART0_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UART0_IRQ]);
  NVIC_SetPriority(UART1_IRQn, g_cntxt.irq_priorities[IRQ_PRIORITY_IDX_UART1_IRQ]);
}

/* ---------------------------------------------------------------------------------------------- */
glossy_status_t glossy_init(void)
{
  cc2538_rf_init();

  cc2538_rf_set_channel(CC2538_RF_CHANNEL);
  /* Initialize id_header */
  g_cntxt.id_header = IHEADER_MAGIC;
  /* Disable encryption by default */
  CLR_IHEADER_ENC_FLAG(g_cntxt.id_header);

  /* FIXEME: Generate random nonce */
  memset(g_cntxt.nonce, 0, GLOSSY_SEC_NONCE_LEN);

  crypto_init();

  GLOSSY_DEBUG_GPIO_PIN_RF_ON_INIT();
  GLOSSY_DEBUG_GPIO_PIN_SFD_RX_INIT();
  GLOSSY_DEBUG_GPIO_PIN_SFD_TX_INIT();

#if 0
  /* FIXME: Need to investigate why SFD signal is not available on PC6
   */
  //GPIO_SET_OUTPUT(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(2));
  GPIO_PERIPHERAL_CONTROL(GPIO_PORT_TO_BASE(GPIO_C_NUM), GPIO_PIN_MASK(2));
  ioc_set_over(GPIO_C_NUM, 2, IOC_OVERRIDE_OE);
  REG(RFCORE_XREG_RFC_OBS_CTRL0) = 0x0000000F;
  REG(CCTEST_OBSSEL2) = CCTEST_OBSSEL_EN;
#endif

  return GLOSSY_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- */
glossy_status_t glossy_start(uint16_t initiator_id, uint8_t* payload, uint8_t payload_len,
		                     uint8_t n_tx_max, glossy_sync_t sync)
{
  g_cntxt.payload = payload;

  g_cntxt.sfd_time = 0;
  g_cntxt.t_tx_start = 0;
  g_cntxt.t_rx_start = 0;
  g_cntxt.t_first_rx = 0;

  g_cntxt.tx_rx_len = 0;
  g_cntxt.bytes_read = 0;
  g_cntxt.g_pkt_len = 0;

  g_cntxt.tx_cnt = 0;
  g_cntxt.rx_cnt = 0;

  g_cntxt.t_ref_mtt = 0;
  g_cntxt.t_ref_updated = 0;
  g_cntxt.relay_cnt_t_ref = 0;

  g_cntxt.T_slot_estimated = 0;
  g_cntxt.T_slot_sum = 0;
  g_cntxt.n_T_slots = 0;

#if GLOSSY_RX_MAJORITY_VOTE
  g_cntxt.rx_payload_cnt = 0;
#endif

  g_cntxt.relay_cnt_last_rx = 0;
  g_cntxt.relay_cnt_last_tx = 0;

  g_cntxt.rf_err_reg_last = 0;

  g_cntxt.crr_header.initiator_id = initiator_id;
  SET_GLOSSY_HEADER_SYNC_OPT(g_cntxt.crr_header.config, sync);
  SET_GLOSSY_HEADER_N_MAX_TX(g_cntxt.crr_header.config, n_tx_max);
  g_cntxt.crr_header.relay_cnt = 0;

  glossy_set_irq_priorities();

  if (IS_INITIATOR()) {
    /* If it is the initiator it has to know whether to use time synchronization or not */
    if (sync == GLOSSY_UNKNOWN_SYNC) {
      PRINTF("Invalid sync option\n");
      return GLOSSY_STATUS_FAIL;
    }

    /* Calculate Glossy packet length */
    g_cntxt.g_pkt_len = payload_len + GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config);
    /* Calculate TX RX length */
    if (GET_IHEADER_ENC_FLAG(g_cntxt.id_header) == IHEADER_ENC_FLAG) {
      /* Encryption enabled */
      g_cntxt.tx_rx_len = IHEADER_LEN + g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN
                          + GLOSSY_SEC_NONCE_LEN + FOOTER_LEN;
    } else {
      /* Encryption is disabled */
      g_cntxt.tx_rx_len = IHEADER_LEN + g_cntxt.g_pkt_len + FOOTER_LEN;
    }

    if (g_cntxt.tx_rx_len > CC2538_RF_MAX_PACKET_LEN) {
      /* Payload is too big to be sent in one packet */
      PRINTF("Too large payload\n");
      return GLOSSY_STATUS_FAIL;
    }

    /* Copy identification header to tx_rx_buffer */
    g_cntxt.tx_rx_buffer[BUF_TXRX_IHEADER_OFFSET] = g_cntxt.id_header;
    /* Copy Glossy header to tx_rx_buffer */
    memcpy(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET], &g_cntxt.crr_header,
           GET_GLOSSY_HEADER_LEN(g_cntxt.crr_header.config));
    /* Copy payload to tx_rx_buffer buffer */
    memcpy(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PAYLOAD_OFFSET(g_cntxt.crr_header.config)], payload,
           payload_len);

    if (GET_IHEADER_ENC_FLAG(g_cntxt.id_header) == IHEADER_ENC_FLAG) {
      /* Increment NONCE */
      add_to_nonce(g_cntxt.nonce, 4);
      /* Copy the NONCE to tx_rx_buffer */
      memcpy(&g_cntxt.tx_rx_buffer[BUF_TXRX_G_PKT_OFFSET + g_cntxt.g_pkt_len + GLOSSY_SEC_MAC_LEN],
             g_cntxt.nonce,
             GLOSSY_SEC_NONCE_LEN);

      if (encryption_start() != GLOSSY_STATUS_SUCCESS) {
        /* Starting encryption failed */
        g_cntxt.stats.enc_dec_errs++;
        PRINTF("Encryption failed\n");
        return GLOSSY_STATUS_FAIL;
      }

    } else {
      /* If we are the initiator we save the tx_rx buffer before the encryption since we may need to
       * retransmit again if we will not receive in the next slot.
       */
      memcpy(g_cntxt.saved_buffer, g_cntxt.tx_rx_buffer, g_cntxt.tx_rx_len);
      /* Just copy to the RF FIFO */
      copy_to_rf_fifo();
    }

    g_cntxt.state = GLOSSY_STATE_ACTIVE;
    /* Start transmission. Actual transmission starts after 192 us */
    radio_start_tx();

  } else {
    /* Not the initiator */
    g_cntxt.state = GLOSSY_STATE_ACTIVE;
    /* Receiver nodes just turn on the radio and listen */
    radio_on();
  }

  return GLOSSY_STATUS_SUCCESS;
}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_stop(void)
{
  rtimer_clock_t t_now_rt, t_next_rt;
  uint64_t t_now_mtt;

  if (g_cntxt.state == GLOSSY_STATE_OFF) {
    return g_cntxt.rx_cnt;
  }

  g_cntxt.state = GLOSSY_STATE_OFF;
  radio_off();

  /* Wait until rtimer captures the next tick */
  t_now_rt = RTIMER_NOW();
  do {
    watchdog_periodic();
  } while (t_now_rt == (t_next_rt = RTIMER_NOW()));
  t_now_mtt = cc2538_rf_get_mac_time_now();

  CC2538_RF_CSP_ISFLUSHRX();
  CC2538_RF_CSP_ISFLUSHTX();

  if (g_cntxt.t_ref_updated) {

    if (g_cntxt.n_T_slots > 0) {
      g_cntxt.t_ref_mtt -= (g_cntxt.relay_cnt_t_ref * g_cntxt.T_slot_sum) / g_cntxt.n_T_slots;
    } else {
      g_cntxt.t_ref_mtt -= g_cntxt.relay_cnt_t_ref * g_cntxt.T_slot_estimated;
    }

    uint64_t t_ref_to_now_mtt = t_now_mtt - g_cntxt.t_ref_mtt;
    rtimer_clock_t t_ref_to_mt_now_rt = 1 + (rtimer_clock_t)(t_ref_to_now_mtt / CLOCK_PHI);
    g_cntxt.t_ref_rt = t_next_rt - t_ref_to_mt_now_rt;
  }

  glossy_restore_irq_priorities();

  NVIC_DisableIRQ(AES_IRQn);
  NVIC_ClearPendingIRQ(AES_IRQn);
  crypto_set_isr_callback(NULL);
  if(REG(AES_CTRL_ALG_SEL) != 0x00000000) {
    /* glossy_stop() is called before AES is done. So we cancel all DMA transfers and
     * reset the algorithm selection of the cryptoprocessor
     */
    REG(AES_DMAC_SWRES) = 0x00000001;
    REG(AES_CTRL_ALG_SEL) = 0x00000000;
  }

#if GLOSSY_RX_MAJORITY_VOTE
  if (!IS_INITIATOR() && g_cntxt.rx_cnt > 0) {
    /* Take majority vote for received payloads */
    uint8_t i;
    uint8_t max_count = 0, max_idx = 0;
    for (i = 0; i < g_cntxt.rx_payload_cnt; i++) {
      if (g_cntxt.rx_payload[i].count > max_count) {
        max_count = g_cntxt.rx_payload[i].count;
        max_idx = i;
      }
    }
    memcpy(g_cntxt.payload, g_cntxt.rx_payload[max_idx].data, g_cntxt.rx_payload[max_idx].len);
    g_cntxt.payload_len = g_cntxt.rx_payload[max_idx].len;
  }
#endif

  g_cntxt.stats.rx_cnt += g_cntxt.rx_cnt;
  g_cntxt.stats.tx_cnt += g_cntxt.tx_cnt;

  return g_cntxt.rx_cnt;
}

/* ---------------------------------------------------------------------------------------------- */
void glossy_debug_print(void) {

#if GLOSSY_DEBUG
  PRINTF("GSTATS| %"PRIu16", %"PRIu16", %"PRIu16", %"PRIu16
         ", %"PRIu16", %"PRIu16", %"PRIu16", %"PRIu16
         ", %"PRIu16"\n",
          g_cntxt.stats.rx_timeout,
          g_cntxt.stats.bad_length,
          g_cntxt.stats.bad_i_header,
          g_cntxt.stats.bad_crc,
          g_cntxt.stats.bad_mac,
          g_cntxt.stats.enc_dec_errs,
          g_cntxt.stats.bad_g_header,
          g_cntxt.stats.payload_mismatch,
          g_cntxt.stats.rf_errs);

  if(g_cntxt.t_ref_updated) {
    PRINTF("n_T_slots %"PRIu8", relay_cnt_t_ref %"PRIu8", T_slot %"PRIu64", t_ref_mtt %"PRIu64
           ", T_slot_estimated %"PRIu64"\n",
            g_cntxt.n_T_slots,
            g_cntxt.relay_cnt_t_ref,
            (g_cntxt.n_T_slots > 0) ? (g_cntxt.T_slot_sum / g_cntxt.n_T_slots) : 0,
            g_cntxt.t_ref_mtt,
            g_cntxt.T_slot_estimated);

  }
#endif /* GLOSSY_DEBUG */

}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_is_t_ref_updated(void)
{
  return g_cntxt.t_ref_updated;
}

/* ---------------------------------------------------------------------------------------------- */
rtimer_clock_t glossy_get_t_ref(void)
{
  return g_cntxt.t_ref_rt;
}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_get_n_rx(void)
{
  return g_cntxt.rx_cnt;
}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_get_payload_len(void)
{
  return g_cntxt.payload_len;
}

/* ---------------------------------------------------------------------------------------------- */
void glossy_set_enc(glossy_enc_t enc) {
  if (enc == GLOSSY_ENC_ON) {
    SET_IHEADER_ENC_FLAG(g_cntxt.id_header);
  } else {
    CLR_IHEADER_ENC_FLAG(g_cntxt.id_header);
  }
}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_get_relay_cnt_first_rx(void)
{
  return g_cntxt.relay_cnt_t_ref;
}

/* ---------------------------------------------------------------------------------------------- */
uint16_t glossy_get_initiator_id(void)
{
  return g_cntxt.crr_header.initiator_id;
}

/* ---------------------------------------------------------------------------------------------- */
glossy_sync_t glossy_get_sync_opt(void) {
  return GET_GLOSSY_HEADER_SYNC_OPT(g_cntxt.crr_header.config);
}

/* ---------------------------------------------------------------------------------------------- */
uint8_t glossy_get_max_payload_len(glossy_enc_t enc)
{
  /* Calculate TX RX length */
  if (enc == GLOSSY_ENC_ON) {
    /* Encryption enabled */
    return CC2538_RF_MAX_PACKET_LEN
        - (IHEADER_LEN + sizeof(glossy_header_t) + GLOSSY_SEC_MAC_LEN + GLOSSY_SEC_NONCE_LEN
           + FOOTER_LEN);
  } else {
    /* Encryption is disabled */
    return CC2538_RF_MAX_PACKET_LEN - (IHEADER_LEN + sizeof(glossy_header_t) + FOOTER_LEN);
  }

  return 0;
}

/* ---------------------------------------------------------------------------------------------- */
void glossy_get_stats(glossy_stats_t* stats)
{
  memcpy(stats, &g_cntxt.stats, sizeof(glossy_stats_t));
}

/* ---------------------------------------------------------------------------------------------- */
uint32_t glossy_get_last_rf_error(void)
{
  return g_cntxt.rf_err_reg_last;
}

/* ---------------------------------------------------------------------------------------------- */
