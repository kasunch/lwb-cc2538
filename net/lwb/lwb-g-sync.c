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

#include "glossy.h"
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

static struct pt pt_sync;
static struct pt pt_rr;

static pt_state_t pt_state_sync;
static pt_state_t pt_state_rr;

static lwb_status_t ret_status;

static volatile uint8_t is_active;

PROCESS_NAME(lwb_main_process);


/*------------------------------------------------------------------------------------------------*/
void lwb_g_sync_init()
{

  is_active = 1;
  lwb_context.run_state = LWB_RUN_STATE_ACTIVE;
  pt_state_sync.pt = &pt_sync;
  pt_state_rr.pt = &pt_rr;

  PT_INIT(pt_state_sync.pt);

  if (lwb_context.lwb_mode == LWB_MODE_HOST) {
    lwb_sched_init();
    lwb_sched_compute_schedule(&lwb_context.current_sched);

    pt_state_sync.cb = lwb_g_sync_host;
    pt_state_rr.cb = lwb_g_sync_host;
    rtimer_set(&lwb_context.rt, RTIMER_NOW() + RTIMER_SECOND * 2, 0,
               (rtimer_callback_t) pt_state_sync.cb, &pt_state_sync);
  } else {
    lwb_context.joining_state = LWB_JOINING_STATE_NOT_JOINED;
    pt_state_sync.cb = lwb_g_sync_source;
    pt_state_rr.cb = lwb_g_sync_source;
    rtimer_set(&lwb_context.rt, RTIMER_NOW() + RTIMER_SECOND, 0,
               (rtimer_callback_t) pt_state_sync.cb, &pt_state_sync);
  }
}

void lwb_g_sync_stop()
{
  is_active = 0;
}

/*------------------------------------------------------------------------------------------------*/
void prepare_schedule(void)
{
  lwb_pkt_header_t* header = (lwb_pkt_header_t*) lwb_context.txrx_buf;
  header->pkt_type = LWB_PKT_TYPE_SCHED;
  lwb_context.txrx_buf_len = sizeof(lwb_pkt_header_t);
  /* Compress and copy the schedule to buffer */
  memcpy(lwb_context.txrx_buf + sizeof(lwb_pkt_header_t), &CURRENT_SCHEDULE_INFO(),
         sizeof(lwb_sched_info_t));
  lwb_context.txrx_buf_len += sizeof(lwb_sched_info_t);
  lwb_context.txrx_buf_len += lwb_sched_compress(&CURRENT_SCHEDULE(),
                                                 lwb_context.txrx_buf + lwb_context.txrx_buf_len,
                                                 LWB_MAX_TXRX_BUF_LEN - lwb_context.txrx_buf_len);
}

/*------------------------------------------------------------------------------------------------*/
PT_THREAD(lwb_g_sync_host(struct rtimer *rt, pt_state_t* pt_state))
{

  pt_state = &pt_state_sync;

  PT_BEGIN(pt_state->pt);

  PRINTF("Starting LWB host thread\r\n");

  prepare_schedule();

  while (is_active) {

    leds_on(LEDS_GREEN);
#if LWB_DEBUG_GPIO
    LWB_DEBUG_GPIO_SET_PIN_1();
#endif

    lwb_context.t_start = RTIMER_TIME(rt);

    lwb_save_energest();
    /* Start glossy and keep it on for T_SYNC_ON time*/
    glossy_start(node_id, lwb_context.txrx_buf, lwb_context.txrx_buf_len, N_SYNC, GLOSSY_WITH_SYNC);
    LWB_WAIT_UNTIL(lwb_context.t_start + T_SYNC_ON);
    glossy_stop();
    lwb_update_ctrl_energest();

    lwb_context.sync_stats.n_rx = glossy_get_n_rx();
    lwb_context.sync_stats.relay_cnt_first_rx = glossy_get_relay_cnt_first_rx();
    lwb_context.t_sync_ref = GLOSSY_T_REF;

    /* Glossy scheduling for data and contention slots  */
    PT_SPAWN(pt_state->pt, pt_state_rr.pt, lwb_g_rr_host(rt, &pt_state_rr, 0));

    memcpy(&OLD_SCHEDULE(), &CURRENT_SCHEDULE(), sizeof(lwb_schedule_t));
    /* Compute new schedule. The current schedule becomes the old one */
    lwb_sched_compute_schedule(&CURRENT_SCHEDULE());
    /* Compress and copy the schedule to buffer */

    /* Compress and copy the schedule to buffer */
    prepare_schedule();

    LWB_SET_POLL_FLAG(LWB_POLL_FLAGS_SCHED_END);
    process_poll(&lwb_main_process);

    leds_off(LEDS_GREEN);
#if LWB_DEBUG_GPIO
    LWB_DEBUG_GPIO_UNSET_PIN_1();
#endif

    /* Wait until start of the next LWB round */
    LWB_WAIT_UNTIL(lwb_context.t_start + OLD_SCHEDULE_INFO().round_period * RTIMER_SECOND);

  }

  is_active = 0;
  lwb_context.run_state = LWB_RUN_STATE_STOPPED;

  PT_END(pt_state->pt);
  return PT_ENDED;
}

/*------------------------------------------------------------------------------------------------*/
static inline void lwb_set_n_my_slots()
{
  uint8_t i;
  lwb_context.n_my_slots = 0;
  for (i = 0; i < N_CURRENT_DATA_SLOTS(); i++) {
    if (node_id == CURRENT_SCHEDULE().slots[i]) {
      lwb_context.n_my_slots++;
    }
  }
}

/*------------------------------------------------------------------------------------------------*/
static inline void lwb_estimate_skew()
{
  /* We estimate the clock skew if we've received more than two consecutive schedules */
  uint32_t t_diff = CURRENT_SCHEDULE_INFO().time - OLD_SCHEDULE_INFO().time;
  int32_t skew_tmp = (int32_t)(lwb_context.t_sync_ref - lwb_context.t_last_sync_ref)
                     - (int32_t)(RTIMER_SECOND * t_diff);

  if (t_diff != 0) {
    lwb_context.skew = skew_tmp / (int32_t)t_diff; /* Calculate skew per second */
  }
}

/*------------------------------------------------------------------------------------------------*/
static lwb_status_t validate_sched_header()
{
  if (glossy_get_payload_len() < sizeof(lwb_pkt_header_t)) {
    return LWB_STATUS_FAIL;
  }

  lwb_pkt_header_t* pkt_header = (lwb_pkt_header_t*) lwb_context.txrx_buf;

  if (glossy_get_payload_len() < sizeof(lwb_sched_info_t)
      || pkt_header->pkt_type != LWB_PKT_TYPE_SCHED) {
    return LWB_STATUS_FAIL;
  }

  return LWB_STATUS_SUCCESS;
}


/*------------------------------------------------------------------------------------------------*/
static inline void update_sync_state()
{
  if (GLOSSY_IS_SYNCED() && validate_sched_header() == LWB_STATUS_SUCCESS) {

    /* Copy only the schedule information header since we need this for clock skew calculation */
    memcpy(&CURRENT_SCHEDULE_INFO(), lwb_context.txrx_buf + sizeof(lwb_pkt_header_t),
           sizeof(lwb_sched_info_t));

    LWB_STATS_SYNC(n_synced)++;

    switch (lwb_context.sync_state) {
      case LWB_SYNC_STATE_BOOTSTRAP: {
        lwb_context.sync_state = LWB_SYNC_STATE_QUASI_SYNCED;
        lwb_context.t_sync_guard = T_GUARD_3;
        lwb_context.t_sync_ref = GLOSSY_T_REF;
        break;
      }
      default: {
        /*
         * If LWB get synchronized while in any other states, we consider LWB is synchronized.
         */
        lwb_context.sync_state = LWB_SYNC_STATE_SYNCED;
        lwb_context.t_sync_guard = T_GUARD;
        lwb_context.t_sync_ref = GLOSSY_T_REF;
        lwb_estimate_skew();
        break;
      }
    }

  } else {
    /* Missed a schedule or received an invalid schedule */

    LWB_STATS_SYNC(n_sync_missed)++;

    switch (lwb_context.sync_state) {
      case LWB_SYNC_STATE_SYNCED: {
        lwb_context.sync_state = LWB_SYNC_STATE_UNSYNCED_1;
        lwb_context.t_sync_guard = T_GUARD_1;
        break;
      }
      case LWB_SYNC_STATE_UNSYNCED_1: {
        lwb_context.sync_state = LWB_SYNC_STATE_UNSYNCED_2;
        lwb_context.t_sync_guard = T_GUARD_2;
        break;
      }
      case LWB_SYNC_STATE_UNSYNCED_2: {
        lwb_context.sync_state = LWB_SYNC_STATE_UNSYNCED_3;
        lwb_context.t_sync_guard = T_GUARD_3;
        break;
      }
      case LWB_SYNC_STATE_UNSYNCED_3:
      case LWB_SYNC_STATE_QUASI_SYNCED: {
        // go back to bootstrap
        lwb_context.sync_state = LWB_SYNC_STATE_BOOTSTRAP;
        lwb_context.t_sync_guard = T_GUARD_3;
        lwb_context.skew = 0;
        break;
      }
      default:
        break;
    }

    if (lwb_context.sync_state != LWB_SYNC_STATE_BOOTSTRAP) {
      /* We may have missed one or few consecutive schedules. So we use the round period of the
       * previous schedule to estimate new reference time
       */
      uint32_t new_t_ref = lwb_context.t_last_sync_ref
                            + OLD_SCHEDULE_INFO().round_period * RTIMER_SECOND
                            + (int32_t) (OLD_SCHEDULE_INFO().round_period * lwb_context.skew);
      lwb_context.t_sync_ref = new_t_ref;
    }

  }
}

/*------------------------------------------------------------------------------------------------*/
static void prepare_for_bootstrap(void)
{

  lwb_context.sync_state = LWB_SYNC_STATE_BOOTSTRAP;

#if LWB_SCHED_SIG_VERIFICATION
  lwb_context.n_con_sig_missed = 0;

  set_random_v_fields(&CURRENT_SCHEDULE());
  glossy_set_v_mode(GLOSSY_V_MODE_ENABLE);

  lwb_sched_sig_reset();

#endif /* LWB_SCHED_SIG_VERIFICATION */
}

/*------------------------------------------------------------------------------------------------*/

PT_THREAD(lwb_g_sync_source(struct rtimer *rt, pt_state_t* pt_state))
{
  pt_state = &pt_state_sync;

  PT_BEGIN(pt_state->pt);

  PRINTF("Starting LWB source thread\r\n");

  prepare_for_bootstrap();

  while (is_active) {

    leds_on(LEDS_GREEN);
#if LWB_DEBUG_GPIO
    LWB_DEBUG_GPIO_SET_PIN_1();
#endif

    lwb_context.t_start = RTIMER_TIME(rt);

    if (lwb_context.sync_state == LWB_SYNC_STATE_BOOTSTRAP) {
      PRINTF("BOOTSTRAP\r\n");
      do {
        lwb_save_energest();
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN,
                     N_SYNC, GLOSSY_WITH_SYNC);
        LWB_WAIT_UNTIL(RTIMER_TIME(rt) + T_SYNC_ON);
        glossy_stop();
        lwb_update_ctrl_energest();
        /* FIXME: Got to sleep if we don't receive for a schedule for long time */
      } while (!GLOSSY_IS_SYNCED());

    } else {
      lwb_save_energest();
      glossy_start(GLOSSY_UNKNOWN_INITIATOR, lwb_context.txrx_buf, GLOSSY_UNKNOWN_PAYLOAD_LEN,
                   N_SYNC, GLOSSY_WITH_SYNC);
      LWB_WAIT_UNTIL(lwb_context.t_start + T_SYNC_ON + lwb_context.t_sync_guard);
      glossy_stop();
      lwb_update_ctrl_energest();
    }

    lwb_context.sync_stats.n_rx = glossy_get_n_rx();
    lwb_context.sync_stats.relay_cnt_first_rx = glossy_get_relay_cnt_first_rx();

    update_sync_state();

    if (lwb_context.sync_state == LWB_SYNC_STATE_BOOTSTRAP) {
      /* We've missed too many schedules. So going to bootstrap */
      continue;
    }

    if (lwb_context.sync_state == LWB_SYNC_STATE_QUASI_SYNCED
        || lwb_context.sync_state == LWB_SYNC_STATE_SYNCED) {
      /* We are good to go */
      lwb_context.txrx_buf_len = glossy_get_payload_len();

      uint8_t* sched = LWB_PKT_DATA_PTR() + sizeof(lwb_sched_info_t);
      uint8_t len = lwb_context.txrx_buf_len - sizeof(lwb_pkt_header_t) - sizeof(lwb_sched_info_t);
      ret_status = lwb_sched_decompress(&CURRENT_SCHEDULE(), sched, len);

      lwb_set_n_my_slots();

      if (lwb_context.sync_state == LWB_SYNC_STATE_SYNCED) {
        PT_SPAWN(pt_state->pt, pt_state_rr.pt, lwb_g_rr_source(rt, &pt_state_rr, 0));
      }

    } else {
      /* We have missed one or more schedules */
      memcpy(&CURRENT_SCHEDULE(), &OLD_SCHEDULE(), sizeof(lwb_schedule_t));
      /* Set the new time based on the round period */
      CURRENT_SCHEDULE_INFO().time = OLD_SCHEDULE_INFO().time + OLD_SCHEDULE_INFO().round_period;

      if (lwb_context.sync_state == LWB_SYNC_STATE_UNSYNCED_1) {
        PT_SPAWN(pt_state->pt, pt_state_rr.pt, lwb_g_rr_source(rt, &pt_state_rr, 0));
      }
    }

    memcpy(&OLD_SCHEDULE(), &CURRENT_SCHEDULE(), sizeof(lwb_schedule_t));
    lwb_context.t_last_sync_ref = lwb_context.t_sync_ref;
    lwb_context.time = OLD_SCHEDULE_INFO().time;

    LWB_SET_POLL_FLAG(LWB_POLL_FLAGS_SCHED_END);
    process_poll(&lwb_main_process);

    leds_off(LEDS_GREEN);
#if LWB_DEBUG_GPIO
    LWB_DEBUG_GPIO_UNSET_PIN_1();
#endif
    /* Wait until start of the next LWB round */
    LWB_WAIT_UNTIL(lwb_context.t_sync_ref
                   + (OLD_SCHEDULE_INFO().round_period * (uint32_t)RTIMER_SECOND)
                   + ((int32_t)OLD_SCHEDULE_INFO().round_period * lwb_context.skew)
                   - lwb_context.t_sync_guard);
  }

  is_active = 0;
  lwb_context.run_state = LWB_RUN_STATE_STOPPED;
  lwb_context.joining_state = LWB_JOINING_STATE_NOT_JOINED;

  PT_END(pt_state->pt);
  return PT_ENDED;
}
