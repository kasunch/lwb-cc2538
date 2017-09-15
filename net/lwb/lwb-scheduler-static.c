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
#include "lib/memb.h"
#include "lib/list.h"
#include "lib/random.h"

#include "glossy.h"
#include "lwb-common.h"
#include "lwb-scheduler.h"
#include "lwb-macros.h"

#if LWB_DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define MAX_BANDWIDTH       LWB_SCHED_MAX_SLOTS
#define MAX_N_FREE_SLOTS    1

#define LWB_SCHED_PERIOD_START    5
#define LWB_SCHED_PERIOD_STEADY   5

#define LWB_SCHED_WAIT_TIME       300
#define LWB_SCHED_WAIT_N_STREAMS  24

extern lwb_context_t lwb_context;

MEMB(streams_memb, lwb_stream_info_t, LWB_MAX_N_STREAMS);
LIST(streams_list);
static uint16_t n_streams;

static lwb_stream_info_t* crr_sched_strms[LWB_SCHED_MAX_SLOTS];
static uint8_t n_crr_sched_strms;

static lwb_stream_info_t* elgble_strms[LWB_MAX_N_STREAMS];
static uint8_t n_elgble_strms;

static uint16_t period;
static uint16_t used_bw;   /* used bandwidth: # packets per period */
static uint16_t max_bw;

/*------------------------------------------------------------------------------------------------*/
static void inline add_stream(uint16_t from_node_id, lwb_stream_req_t *p_req)
{
  /* We have a stream add request with an existing stream ID.
   * This could happen due to the node has not received the stream acknowledgement.
   * So we add an acknowledgement from it. Otherwise it will keep sending stream requests.
   */
  if (lwb_context.n_stream_acks < LWB_SCHED_MAX_SLOTS) {
    lwb_context.stream_akcs[lwb_context.n_stream_acks++] = from_node_id;
  }

  lwb_stream_info_t *crr_stream;
  for (crr_stream = list_head(streams_list); crr_stream != NULL; crr_stream = crr_stream->next) {
    if (from_node_id == crr_stream->node_id
        && LWB_GET_STREAM_ID(p_req->req_type) == crr_stream->stream_id) {
      /* duplicate stream add request */
      PRINTF("SREQ duplicate: node %"PRIu16", id %"PRIu8", ipi %"PRIu16"\n",
             from_node_id, LWB_GET_STREAM_ID(p_req->req_type), p_req->ipi);
      LWB_STATS_SCHED(n_duplicates)++;
      return;
    }
  }

  if (used_bw + MAX(1, period / p_req->ipi) > max_bw) {
    /* Cannot support stream due to bandwidth limit. Drop it */
    PRINTF("SREQ BW limit: used %"PRIu16", max %"PRIu16", node %"PRIu16", id %"PRIu8", ipi %"PRIu16"\n",
           used_bw, max_bw, from_node_id, LWB_GET_STREAM_ID(p_req->req_type), p_req->ipi);
    return;
  }

  crr_stream = memb_alloc(&streams_memb);
  if (!crr_stream) {
    LWB_STATS_SCHED(n_no_space)++;
    return;
  }

  memset(crr_stream, 0, sizeof(lwb_stream_info_t));
  crr_stream->node_id = from_node_id;
  crr_stream->ipi = p_req->ipi;
  crr_stream->last_assigned = lwb_context.time;
  crr_stream->next_ready = p_req->time_info;
  crr_stream->stream_id = LWB_GET_STREAM_ID(p_req->req_type);
  crr_stream->avg_max_qlen = LWB_SCHED_DEFAULT_AVG_MAX_QLEN;

  list_add(streams_list, crr_stream);
  n_streams++;

  used_bw += MAX(1, period / p_req->ipi);

  LWB_STATS_SCHED(n_added)++;

  PRINTF("SREQ added: used %"PRIu16", max %"PRIu16", node %"PRIu16", id %"PRIu8", ipi %"PRIu16"\n",
         used_bw, max_bw, from_node_id, LWB_GET_STREAM_ID(p_req->req_type), p_req->ipi);

}

/*------------------------------------------------------------------------------------------------*/
static void del_stream_ex(lwb_stream_info_t *stream)
{
  if (stream == NULL) {
    return;
  }

  used_bw -= MAX(1, period / stream->ipi);
  PRINTF("SREQ deleted: used %"PRIu16", max %"PRIu16", node %"PRIu16", id %"PRIu8", ipi %"PRIu16"\n",
         used_bw, max_bw, stream->node_id, stream->stream_id, stream->ipi);

  list_remove(streams_list, stream);
  memb_free(&streams_memb, stream);
  n_streams--;
}

/*------------------------------------------------------------------------------------------------*/
static inline void del_stream(uint16_t id, lwb_stream_req_t *p_req)
{
  lwb_stream_info_t *prev_stream;
  for (prev_stream = list_head(streams_list); prev_stream != NULL;
       prev_stream = prev_stream->next) {

    if ((id == prev_stream->node_id)
        && (LWB_GET_STREAM_ID(p_req->req_type) == prev_stream->stream_id)) {
      del_stream_ex(prev_stream);
      return;
    }
  }
}
/*------------------------------------------------------------------------------------------------*/
void lwb_sched_init(void)
{
  memb_init(&streams_memb);
  list_init(streams_list);
  n_streams = 0;

  period = LWB_SCHED_PERIOD_START;
  max_bw = MIN(LWB_SCHED_GET_MAX_BW(period, MAX_N_FREE_SLOTS), LWB_SCHED_MAX_SLOTS) ;
  used_bw = 0;
}

/*------------------------------------------------------------------------------------------------*/
void lwb_sched_compute_schedule(lwb_schedule_t* p_sched)
{
  uint8_t n_free_slots;
  uint8_t n_assigned_slots = 0;
  lwb_stream_info_t *crr_strm;
  lwb_stream_info_t *strm_to_remove;
  uint8_t i;

  for (crr_strm = list_head(streams_list); crr_strm != NULL;) {

    LWB_STATS_SCHED(n_unused_slots) += crr_strm->n_allocated - crr_strm->n_used;

    crr_strm->n_allocated = 0;
    crr_strm->n_used = 0;

    /* Recycle unused data slots based on activity */
    if (crr_strm->n_cons_missed > LWB_SCHED_N_CONS_MISSED_MAX) {
      strm_to_remove = crr_strm;
      crr_strm = crr_strm->next;
      del_stream_ex(strm_to_remove);
    } else {
      crr_strm = crr_strm->next;
    }
  }


  memset(crr_sched_strms, 0, sizeof(lwb_stream_info_t*) * LWB_SCHED_MAX_SLOTS);
  n_crr_sched_strms = 0;

  /* Always have a contention slot */
  n_free_slots = MAX_N_FREE_SLOTS;

  if (lwb_context.n_stream_acks > 0) {
    /* We have stream ACKs to be sent.
     * There is no stream associated for stream AKCs
     */
    crr_sched_strms[n_crr_sched_strms++] = NULL;
    p_sched->slots[n_assigned_slots++] = 0;
  }

  lwb_context.time += period;

  /* Find eligible streams */
  uint8_t tot_in_this_round = 0;
  n_elgble_strms = 0;
  for (crr_strm = list_head(streams_list); crr_strm != NULL; crr_strm = crr_strm->next) {
    if (lwb_context.time >= crr_strm->ipi + crr_strm->last_assigned) {
      tot_in_this_round += (uint8_t)((lwb_context.time - crr_strm->last_assigned) / crr_strm->ipi);
      elgble_strms[n_elgble_strms++] = crr_strm;
    }
  }
  /* Calculate the maximum number of slots we can accommodate */
  tot_in_this_round = MIN(max_bw, (tot_in_this_round + n_assigned_slots));
  /* Allocate slots for all eligible streams in round-robin manner */
  while (n_assigned_slots < tot_in_this_round) {
    for (i = 0; i < n_elgble_strms && n_assigned_slots < tot_in_this_round; i++) {
        p_sched->slots[n_assigned_slots++] = elgble_strms[i]->node_id;
        elgble_strms[i]->n_allocated++;
        elgble_strms[i]->last_assigned = lwb_context.time;
        crr_sched_strms[n_crr_sched_strms++] = elgble_strms[i];
      }
  }

  if (lwb_context.time > LWB_SCHED_WAIT_TIME || n_streams == LWB_SCHED_WAIT_N_STREAMS) {
    period = LWB_SCHED_PERIOD_STEADY;
    max_bw = MIN(LWB_SCHED_GET_MAX_BW(period, MAX_N_FREE_SLOTS), LWB_SCHED_MAX_SLOTS) ;
  }

  LWB_SET_N_FREE_SLOTS(p_sched->sched_info.n_slots, n_free_slots);
  LWB_SET_N_DATA_SLOTS(p_sched->sched_info.n_slots, n_assigned_slots);
  p_sched->sched_info.time = lwb_context.time;
  p_sched->sched_info.round_period = period;

  PRINTF("MAX_BW %"PRIu16"\n", max_bw);
}

/*------------------------------------------------------------------------------------------------*/
void lwb_sched_process_stream_req(uint16_t from_node_id, lwb_stream_req_t *req)
{
  switch (LWB_GET_STREAM_TYPE(req->req_type)) {
    case LWB_STREAM_TYPE_ADD:
      add_stream(from_node_id, req);
      break;
    case LWB_STREAM_TYPE_DEL:
      del_stream(from_node_id, req);
      break;
    case LWB_STREAM_TYPE_MOD:
      del_stream(from_node_id, req);
      add_stream(from_node_id, req);
      break;
    default:
      break;
  }
}

/*------------------------------------------------------------------------------------------------*/
void lwb_sched_update_data_slot_usage(uint8_t slot_index, uint8_t used)
{
  if (slot_index > 0 && slot_index < n_crr_sched_strms && crr_sched_strms[slot_index]) {
    if (used) {
      crr_sched_strms[slot_index]->n_used++;
      crr_sched_strms[slot_index]->n_cons_missed = 0;
    } else {
      crr_sched_strms[slot_index]->n_cons_missed++;
    }
  }
}

/*------------------------------------------------------------------------------------------------*/
void lwb_sched_update_qlen(uint8_t slot_index, uint8_t qlen)
{
  if (slot_index < n_crr_sched_strms && crr_sched_strms[slot_index]) {
    if (crr_sched_strms[slot_index]->max_qlen < qlen) {
      crr_sched_strms[slot_index]->max_qlen = qlen;
    }
  }
}

/*------------------------------------------------------------------------------------------------*/
void lwb_sched_print()
{
#if LWB_DEBUG
  lwb_stream_info_t *crr_stream;
  uint8_t i = 0;
  // Recycle unused data slots based on activity
  PRINTF("ST|");
  for (crr_stream = list_head(streams_list); crr_stream != NULL;) {
    PRINTF("%u-%u ", i++, crr_stream->avg_max_qlen);
    crr_stream = crr_stream->next;
  } PRINTF("\n");
#endif
}
