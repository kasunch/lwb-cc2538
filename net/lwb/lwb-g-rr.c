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

#include <string.h>
#include <stdio.h>
#include <inttypes.h>

#include "contiki.h"
#include "dev/leds.h"
#include "lib/memb.h"
#include "lib/list.h"
#include "lib/random.h"

#include "glossy.h"
#include "lwb.h"
#include "lwb-common.h"
#include "lwb-macros.h"
#include "lwb-g-sync.h"
#include "lwb-g-rr.h"
#include "lwb-scheduler.h"
#include "lwb-sched-compressor.h"

#if LWB_DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

extern lwb_context_t lwb_context;

PROCESS_NAME(lwb_main_process);

/// @brief Iterator for slot index
static uint8_t slot_idx;

/** \brief Buffers for TX and RX */
MEMB(mmb_data_buf, data_buf_lst_item_t, LWB_MAX_DATA_BUF_ELEMENTS);
/** \brief Transmit data buffer element list */
LIST(lst_tx_buf_queue);
/** \brief Receive data buffer element list */
LIST(lst_rx_buf_queue);
/** \brief Number of data packets to be sent over LWB */
static uint8_t tx_buf_q_size;
/** \brief Number of data packets to be delivered to the application layer */
static uint8_t rx_buf_q_size;

/** \brief Memory block allocation space for stream requests */
MEMB(mmb_stream_req, stream_req_lst_item_t, LWB_MAX_STREAM_REQ_ELEMENTS);
/** \brief List for stream requests to be sent */
LIST(lst_stream_req);
/** @brief Number of stream requests to be sent */
static uint8_t stream_reqs_lst_size;

static uint8_t n_rounds_to_wait;

static uint8_t n_trials;

static uint8_t stream_id_next;


/*------------------------------------------------------------------------------------------------*/
void lwb_g_rr_init()
{

  memb_init(&mmb_data_buf);
  list_init(lst_tx_buf_queue);
  list_init(lst_rx_buf_queue);
  tx_buf_q_size = 0;
  rx_buf_q_size = 0;
  memb_init(&mmb_stream_req);
  list_init(lst_stream_req);
  stream_reqs_lst_size = 0;
  n_rounds_to_wait = 0;
  n_trials = 0;
  stream_id_next = 1;

}

/*------------------------------------------------------------------------------------------------*/
static void prepare_stream_acks()
{
  SET_LWB_PKT_TYPE(LWB_PKT_TYPE_STREAM_ACK);

  lwb_stream_ack_header_t* ack_header = (lwb_stream_ack_header_t*) LWB_PKT_DATA_PTR();
  ack_header->n_acks = lwb_context.n_stream_acks;

  memcpy(LWB_PKT_DATA_PTR() + sizeof(lwb_stream_ack_header_t), lwb_context.stream_akcs,
         2 * lwb_context.n_stream_acks);

  lwb_context.txrx_buf_len = sizeof(lwb_pkt_header_t) + sizeof(lwb_stream_ack_header_t)
                             + 2 * lwb_context.n_stream_acks;

  LWB_STATS_STREAM_REQ_ACK(n_ack_tx) += lwb_context.n_stream_acks;

  lwb_context.n_stream_acks = 0;
}

/*------------------------------------------------------------------------------------------------*/
static void prepare_data_packet()
{
  data_buf_lst_item_t* buf_item = list_head(lst_tx_buf_queue);
  stream_req_lst_item_t* req_item;
  uint8_t n_possible;
  uint8_t n_available;
  uint8_t i;

  SET_LWB_PKT_TYPE(LWB_PKT_TYPE_DATA);
  /* Copy only the data now and we will copy the header at the end.
   * This is to avoid possible alignment issues
   */
  memcpy(LWB_PKT_DATA_PTR() + sizeof(data_header_t), &buf_item->buf.data, buf_item->buf.header.data_len);
  lwb_context.txrx_buf_len = sizeof(lwb_pkt_header_t) + sizeof(data_header_t)
                             + buf_item->buf.header.data_len;

  /* Calculate possible number of stream requests that can be piggybacked with the application
   * data
   */
  n_possible = (LWB_PKT_APP_DATA_LEN_MAX() - buf_item->buf.header.data_len
                - sizeof(lwb_stream_req_header_t))
               / sizeof(lwb_stream_req_t);
  n_available = MIN(n_possible, stream_reqs_lst_size);

  if (lwb_context.lwb_mode == LWB_MODE_SOURCE && stream_reqs_lst_size > 0 && n_available > 0) {

    lwb_stream_req_header_t str_req_hdr;

    str_req_hdr.n_reqs = n_available;
    memcpy(lwb_context.txrx_buf + lwb_context.txrx_buf_len, &str_req_hdr,
           sizeof(lwb_stream_req_header_t));
    lwb_context.txrx_buf_len += sizeof(lwb_stream_req_header_t);

    /* Iterate through all stream requests and try to include them into one packet.
     * Here, we do not remove any of the stream requests from the list as they may need to be resent
     * in a later time if no acknowledgements are received.
     */
    for (req_item = list_head(lst_stream_req), i = 0;
         req_item != NULL && i < n_available;
         req_item = req_item->next, i++) {

      memcpy(lwb_context.txrx_buf + lwb_context.txrx_buf_len, &req_item->req,
             sizeof(lwb_stream_req_t));

      lwb_context.txrx_buf_len += sizeof(lwb_stream_req_t);
    }
    LWB_PKT_APP_DATA_HDR_OPT_SET_PKT_TYPE(&(buf_item->buf.header), LWB_PKT_TYPE_STREAM_REQ);
  }

  /* Set data header and copy to the buffer */
  buf_item->buf.header.in_queue = tx_buf_q_size - 1;
  memcpy(LWB_PKT_DATA_PTR(), &(buf_item->buf.header), sizeof(data_header_t));

  list_remove(lst_tx_buf_queue, buf_item);
  memb_free(&mmb_data_buf, buf_item);
  tx_buf_q_size--;

  LWB_STATS_DATA(n_tx)++;
}

/*------------------------------------------------------------------------------------------------*/
static void prepare_stream_reqs()
{

  uint8_t n_possible = (LWB_PKT_DATA_LEN_MAX() - sizeof(lwb_stream_req_header_t))
                       / sizeof(lwb_stream_req_t);
  uint8_t n_available = MIN(n_possible, stream_reqs_lst_size);
  uint8_t i;
  stream_req_lst_item_t* req_item;

  SET_LWB_PKT_TYPE(LWB_PKT_TYPE_STREAM_REQ);

  lwb_stream_req_header_t* req_hdr = (lwb_stream_req_header_t*) LWB_PKT_DATA_PTR();
  req_hdr->n_reqs = n_available;

  lwb_context.txrx_buf_len = sizeof(lwb_pkt_header_t) + sizeof(lwb_stream_req_header_t);
  /* Iterate through all stream requests and try to include them into one packet.
   * Here, we do not remove any of the stream requests from the list as they may need to be resent
   * in a later time if no acknowledgements are received.
   */
  for (req_item = list_head(lst_stream_req), i = 0;
       req_item != NULL && i < n_available;
       req_item = req_item->next, i++) {

    memcpy(lwb_context.txrx_buf + lwb_context.txrx_buf_len, &req_item->req,
           sizeof(lwb_stream_req_t));

    lwb_context.txrx_buf_len += sizeof(lwb_stream_req_t);
  }

  LWB_STATS_STREAM_REQ_ACK(n_req_tx) += i;

}

/*------------------------------------------------------------------------------------------------*/
static uint8_t prepare_packets_from_host(void)
{
  if (lwb_context.n_stream_acks > 0) {
    prepare_stream_acks();
    return 1;
  }
  return 0;
}

/*------------------------------------------------------------------------------------------------*/
static void iterate_stream_reqs(lwb_stream_req_header_t* str_req_hdr)
{
  uint8_t i;
  lwb_stream_req_t *stream_req = (lwb_stream_req_t*) ((uint8_t*) str_req_hdr
                                                      + sizeof(lwb_stream_req_header_t));
  lwb_stream_req_t stream_req_tmp;
  for (i = 0; i < str_req_hdr->n_reqs; i++) {
    memcpy(&stream_req_tmp, &stream_req[i], sizeof(lwb_stream_req_t));
    lwb_sched_process_stream_req(glossy_get_initiator_id(), &stream_req_tmp);
  }
}

/*------------------------------------------------------------------------------------------------*/
static void process_data_packet(uint8_t slot_idx)
{
  if (GET_LWB_PKT_TYPE() != LWB_PKT_TYPE_DATA) {
    return;
  }

  if (lwb_context.txrx_buf_len < sizeof(lwb_pkt_header_t) + sizeof(data_header_t)) {
    return;
  }

  lwb_sched_update_data_slot_usage(slot_idx, 1);

  data_header_t data_hdr;
  memcpy(&data_hdr, LWB_PKT_DATA_PTR(), sizeof(data_header_t));
  if (data_hdr.to_id != node_id && data_hdr.to_id != 0) {
    // We drop this packet
    LWB_STATS_DATA(n_rx_dropped)++;
    return;
  }

  data_buf_lst_item_t* buf_item = memb_alloc(&mmb_data_buf);
  if (!buf_item) {
    LWB_STATS_DATA(n_rx_nospace)++;
    return;
  }

  buf_item->from_id = glossy_get_initiator_id();
  /* Copy the data including the header into the buffer and add to the queue */
  memcpy(&(buf_item->buf), LWB_PKT_DATA_PTR(), sizeof(data_header_t) + data_hdr.data_len);
  list_add(lst_rx_buf_queue, buf_item);
  rx_buf_q_size++;

  LWB_STATS_DATA(n_rx)++;

  /* Poll the LWB main process to deliver data to APP layer */
  LWB_SET_POLL_FLAG(LWB_POLL_FLAGS_DATA);
  process_poll(&lwb_main_process);

  /* Only the host processes piggybacked stream requests */
  if (lwb_context.lwb_mode == LWB_MODE_HOST
      && LWB_PKT_APP_DATA_HDR_OPT_GET_PKT_TYPE(&data_hdr) == LWB_PKT_TYPE_STREAM_REQ) {

    lwb_stream_req_header_t* str_req_hdr = (lwb_stream_req_header_t*)(LWB_PKT_APP_DATA_PTR()
                                                                      + data_hdr.data_len);
    iterate_stream_reqs(str_req_hdr);
  }

}

/*------------------------------------------------------------------------------------------------*/
static void process_stream_acks()
{
  if (GET_LWB_PKT_TYPE() != LWB_PKT_TYPE_STREAM_ACK) {
    return;
  }

  lwb_stream_ack_header_t* ack_hdr = (lwb_stream_ack_header_t*) LWB_PKT_DATA_PTR();

  if (lwb_context.txrx_buf_len < sizeof(lwb_pkt_header_t) + sizeof(lwb_stream_ack_header_t)
                                 + ack_hdr->n_acks * sizeof(uint16_t)) {
    return;
  }

  LWB_STATS_STREAM_REQ_ACK(n_ack_rx) += ack_hdr->n_acks;

  uint8_t* acks_ptr = LWB_PKT_DATA_PTR() + sizeof(lwb_stream_ack_header_t);
  uint8_t i;
  uint16_t ack_node_id;
  stream_req_lst_item_t* req_item;
  for (i = 0; i < ack_hdr->n_acks; i++) {
    ack_node_id = acks_ptr[i * 2] | acks_ptr[i * 2 + 1] << 8;
    if (ack_node_id == node_id && (req_item = list_head(lst_stream_req))) {
      list_remove(lst_stream_req, req_item);
      memb_free(&mmb_stream_req, req_item);
      stream_reqs_lst_size--;
    }
  }

  if(stream_reqs_lst_size == 0) {
    /* Hooray..! we are joined */
    lwb_context.joining_state = LWB_JOINING_STATE_JOINED;
  }
}

/*------------------------------------------------------------------------------------------------*/
static void process_packets_from_host(void)
{
  if (GET_LWB_PKT_TYPE() == LWB_PKT_TYPE_STREAM_ACK) {
    process_stream_acks();
  }
}

/*------------------------------------------------------------------------------------------------*/
static void process_stream_reqs(uint8_t slot_idx)
{
  if (GET_LWB_PKT_TYPE() != LWB_PKT_TYPE_STREAM_REQ) {
    return;
  }

  if (lwb_context.txrx_buf_len < sizeof(lwb_pkt_header_t) + sizeof(lwb_stream_req_header_t)) {
    return;
  }

  lwb_stream_req_header_t* str_req_hdr = (lwb_stream_req_header_t*)LWB_PKT_DATA_PTR();
  iterate_stream_reqs(str_req_hdr);

}
/*------------------------------------------------------------------------------------------------*/
PT_THREAD(lwb_g_rr_host(struct rtimer *rt, pt_state_t* pt_state, uint8_t idx_start))
{
  PT_BEGIN(pt_state->pt);

  lwb_reset_slot_energest();
  /* Loop for data and ACK slots */
  for (slot_idx = 0; slot_idx < N_CURRENT_DATA_SLOTS(); slot_idx++) {

    lwb_save_energest();

    if (CURRENT_SCHEDULE().slots[slot_idx] == 0) {
      /* We have stream acknowledgement(s) to be sent. */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP));

      if (prepare_packets_from_host()) {
        glossy_start(node_id, lwb_context.txrx_buf, lwb_context.txrx_buf_len, N_RR,
                     GLOSSY_ONLY_RELAY_CNT);
        LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                       + T_SYNC_ON
                       + T_S_R_GAP
                       + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                       + T_RR_ON);
        glossy_stop();
        lwb_update_ctrl_energest();
      } else {
        /* No stream AKCs */
      }

    } else if (CURRENT_SCHEDULE().slots[slot_idx] == node_id) {
      /* This is our slot. Send data if we have */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP));

      if (tx_buf_q_size > 0) {
        prepare_data_packet();
        glossy_start(node_id, lwb_context.txrx_buf, lwb_context.txrx_buf_len, N_RR,
                     GLOSSY_ONLY_RELAY_CNT);
        LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                       + T_SYNC_ON
                       + T_S_R_GAP
                       + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                       + T_RR_ON);
        glossy_stop();
      } else {
        /* We have nothing to send. Stay silent */
      }

    } else {
      /* Not our slot. Just participate to the flooding. Wake up early */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     - T_GUARD);
      glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                   GLOSSY_ONLY_RELAY_CNT);
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     + T_RR_ON
                     + T_GUARD);
      glossy_stop();

      if (glossy_get_n_rx() > 0) {
        lwb_context.txrx_buf_len = glossy_get_payload_len();
        process_data_packet(slot_idx);
      } else {
        /* Nothing received */
        lwb_sched_update_data_slot_usage(slot_idx, 0);
      }

    }

    lwb_update_slot_energest();
  }

  lwb_save_energest();
  /* Loop for contention slots */
  for (;slot_idx < N_CURRENT_DATA_SLOTS() + N_CURRENT_FREE_SLOTS(); slot_idx++) {

    /* Wake up early */
    LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                   + T_SYNC_ON
                   + T_S_R_GAP
                   + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                   - T_GUARD);

    glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                 GLOSSY_ONLY_RELAY_CNT);
    LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                   + T_SYNC_ON
                   + T_S_R_GAP
                   + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                   + T_RR_ON
                   + T_GUARD);
    glossy_stop();

    if (glossy_get_n_rx() > 0) {
      lwb_context.txrx_buf_len = glossy_get_payload_len();
      process_stream_reqs(slot_idx);
    } else {
      /* Nothing received */
    }
  }
  lwb_update_ctrl_energest();

  PT_END(pt_state->pt);

  return PT_ENDED;
}

/*------------------------------------------------------------------------------------------------*/
PT_THREAD(lwb_g_rr_source(struct rtimer *rt, pt_state_t* pt_state, uint8_t idx_start))
{
  PT_BEGIN(pt_state->pt);

  lwb_reset_slot_energest();
  /* Loop for data and ACK slots */
  for (slot_idx = 0; slot_idx < N_CURRENT_DATA_SLOTS(); slot_idx++) {

    lwb_save_energest();

    if (CURRENT_SCHEDULE().slots[slot_idx] == 0) {
      /* We have stream acknowledgement(s) to be received. Wake up early */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     - T_GUARD);
      glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                   GLOSSY_ONLY_RELAY_CNT);
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     + T_RR_ON
                     + T_GUARD);
      glossy_stop();
      lwb_update_ctrl_energest();

      if (glossy_get_n_rx() > 0) {
        lwb_context.txrx_buf_len = glossy_get_payload_len();
        process_packets_from_host();
      }

    } else if (CURRENT_SCHEDULE().slots[slot_idx] == node_id) {
      /* This is our slot. Send data if we have */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP));

      if (tx_buf_q_size > 0) {
        prepare_data_packet();
        glossy_start(node_id, lwb_context.txrx_buf, lwb_context.txrx_buf_len, N_RR,
                     GLOSSY_ONLY_RELAY_CNT);
        LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                       + T_SYNC_ON
                       + T_S_R_GAP
                       + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                       + T_RR_ON);
        glossy_stop();
      } else {
        /* We have nothing to send. Stay silent */
      }

    } else {
      /* Not our slot. Just participate to the flooding. Wake up early. */
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     - T_GUARD);
      glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                   GLOSSY_ONLY_RELAY_CNT);
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     + T_RR_ON
                     + T_GUARD);
      glossy_stop();

      if (glossy_get_n_rx() > 0) {
        lwb_context.txrx_buf_len = glossy_get_payload_len();
        process_data_packet(slot_idx);
      } else {
        /* Nothing received */
      }
    }
    lwb_update_slot_energest();
  }

  lwb_save_energest();
  /* Loop for contention slots */
  for (;slot_idx < N_CURRENT_DATA_SLOTS() + N_CURRENT_FREE_SLOTS(); slot_idx++) {

    LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                   + T_SYNC_ON
                   + T_S_R_GAP
                   + (idx_start + slot_idx) * (T_RR_ON + T_GAP));

    if (stream_reqs_lst_size > 0) {
      if (n_rounds_to_wait == 0) {
        prepare_stream_reqs();
        glossy_start(node_id, lwb_context.txrx_buf, lwb_context.txrx_buf_len, N_RR,
                     GLOSSY_ONLY_RELAY_CNT);
        LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                       + T_SYNC_ON
                       + T_S_R_GAP
                       + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                       + T_RR_ON);
        glossy_stop();
        n_trials++;
        /* Calculate the number of trials to wait before trying again */
        n_rounds_to_wait = (uint8_t)random_rand() % (1 << (n_trials % 4));

      } else {
        /* We just participate to the flooding */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                     GLOSSY_ONLY_RELAY_CNT);
        LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                       + T_SYNC_ON
                       + T_S_R_GAP
                       + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                       + T_RR_ON);
        glossy_stop();
        n_rounds_to_wait--;
      }

    } else {
      /* We just participate to the flooding */
      glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN, N_RR,
                   GLOSSY_ONLY_RELAY_CNT);
      LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                     + T_SYNC_ON
                     + T_S_R_GAP
                     + (idx_start + slot_idx) * (T_RR_ON + T_GAP)
                     + T_RR_ON);
      glossy_stop();
    }
  }
  lwb_update_ctrl_energest();

  PT_END(pt_state->pt);
  return PT_ENDED;
}

/*------------------------------------------------------------------------------------------------*/
lwb_status_t lwb_g_rr_queue_packet(uint8_t* data, uint8_t data_len, uint16_t to_id)
{
  if (LWB_PKT_APP_DATA_LEN_MAX() < data_len) {
    return LWB_STATUS_FAIL;
  }

  data_buf_lst_item_t* p_item = memb_alloc(&mmb_data_buf);

  if (!p_item) {
    LWB_STATS_DATA(n_tx_nospace)++;
    return LWB_STATUS_FAIL;
  }

  p_item->from_id = node_id;
  p_item->buf.header.to_id = to_id;
  p_item->buf.header.data_len = data_len;
  p_item->buf.header.options = 0;
  memcpy(p_item->buf.data, data, data_len);
  list_add(lst_tx_buf_queue, p_item);
  tx_buf_q_size++;

  return LWB_STATUS_SUCCESS;
}

/*------------------------------------------------------------------------------------------------*/
uint8_t lwb_g_rr_stream_add(uint16_t ipi, uint16_t time_offset)
{
  stream_req_lst_item_t* p_req_item = memb_alloc(&mmb_stream_req);

  if (!p_req_item) {
    return 0;
  }

  p_req_item->req.ipi = ipi;
  p_req_item->req.time_info = time_offset;
  LWB_SET_STREAM_TYPE(p_req_item->req.req_type, LWB_STREAM_TYPE_ADD);
  LWB_SET_STREAM_ID(p_req_item->req.req_type, stream_id_next);
  list_add(lst_stream_req, p_req_item);
  stream_reqs_lst_size++;

  switch (lwb_context.joining_state) {
    case LWB_JOINING_STATE_NOT_JOINED:
    case LWB_JOINING_STATE_JOINING:
      lwb_context.joining_state = LWB_JOINING_STATE_JOINING;
      break;
    case LWB_JOINING_STATE_JOINED:
    case LWB_JOINING_STATE_PARTLY_JOINED:
      lwb_context.joining_state = LWB_JOINING_STATE_PARTLY_JOINED;
      break;
  }

  return stream_id_next++;
}

/*------------------------------------------------------------------------------------------------*/
void lwb_g_rr_stream_del(uint8_t id)
{
  stream_req_lst_item_t* p_req_item = memb_alloc(&mmb_stream_req);

  if (!p_req_item) {
      return;
  }

  p_req_item->req.ipi = 0;
  p_req_item->req.time_info = 0;
  LWB_SET_STREAM_TYPE(p_req_item->req.req_type, LWB_STREAM_TYPE_DEL);
  LWB_SET_STREAM_ID(p_req_item->req.req_type, id);
  list_add(lst_stream_req, p_req_item);
  stream_reqs_lst_size++;
  /* we don't care about the joining state in here */
}

/*------------------------------------------------------------------------------------------------*/
void lwb_g_rr_stream_mod(uint8_t id, uint16_t ipi) {
  stream_req_lst_item_t* p_req_item = memb_alloc(&mmb_stream_req);

  if (!p_req_item) {
      return;
  }

  PRINTF("stream mod\n");

  p_req_item->req.ipi = ipi;
  p_req_item->req.time_info = 0;
  LWB_SET_STREAM_TYPE(p_req_item->req.req_type, LWB_STREAM_TYPE_MOD);
  LWB_SET_STREAM_ID(p_req_item->req.req_type, id);
  list_add(lst_stream_req, p_req_item);
  stream_reqs_lst_size++;

  // Set the joining state
  switch (lwb_context.joining_state) {
    case LWB_JOINING_STATE_NOT_JOINED:
    case LWB_JOINING_STATE_JOINING:
      lwb_context.joining_state = LWB_JOINING_STATE_JOINING;
      break;
    case LWB_JOINING_STATE_JOINED:
    case LWB_JOINING_STATE_PARTLY_JOINED:
      lwb_context.joining_state = LWB_JOINING_STATE_PARTLY_JOINED;
      break;
  }
}

/*------------------------------------------------------------------------------------------------*/
void lwb_g_rr_data_output()
{
  data_buf_lst_item_t* item = NULL;
  while ((item = list_head(lst_rx_buf_queue))) {
    if (lwb_context.p_callbacks && lwb_context.p_callbacks->p_on_data) {
      lwb_context.p_callbacks->p_on_data(item->buf.data, item->buf.header.data_len,
                                         item->from_id);
    }

    list_remove(lst_rx_buf_queue, item);
    memb_free(&mmb_data_buf, item);
    rx_buf_q_size--;
  }
}
