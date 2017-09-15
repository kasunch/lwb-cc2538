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

#ifndef __LWB_MACROS_H___
#define __LWB_MACROS_H___

/// @file lwb-macros.h
/// @brief LWB context specific macros

#include "contiki.h"

#include "lwb-common.h"

#define N_CURRENT_DATA_SLOTS()          LWB_GET_N_DATA_SLOTS(lwb_context.current_sched.sched_info.n_slots)
#define N_CURRENT_FREE_SLOTS()          LWB_GET_N_FREE_SLOTS(lwb_context.current_sched.sched_info.n_slots)
#define CURRENT_SCHEDULE()              (lwb_context.current_sched)
#define CURRENT_SCHEDULE_INFO()         (lwb_context.current_sched.sched_info)

#define OLD_SCHEDULE()                  (lwb_context.old_sched)
#define OLD_SCHEDULE_INFO()             (lwb_context.old_sched.sched_info)

#define LWB_STATS_SYNC(statitem)        (lwb_context.sync_stats.statitem)
#define LWB_STATS_DATA(statitem)        (lwb_context.data_stats.statitem)
#define LWB_STATS_STREAM_REQ_ACK(statitem)  (lwb_context.stream_req_ack_stats.statitem)
#define LWB_STATS_SCHED(statitem)       (lwb_context.sched_stats.statitem)

#define LWB_SET_POLL_FLAG(flag)         (lwb_context.poll_flags |= 1 << flag)
#define LWB_UNSET_POLL_FLAG(flag)       (lwb_context.poll_flags &= ~(1 << flag))
#define LWB_IS_SET_POLL_FLAG(flag)      (lwb_context.poll_flags && (1 << flag))

/// @defgroup TX RX buffer macros
/// @{
#define GET_LWB_PKT_TYPE(type)          (lwb_context.txrx_buf[0])
#define SET_LWB_PKT_TYPE(type)          (lwb_context.txrx_buf[0] = type)
#define LWB_PKT_DATA_PTR()              (lwb_context.txrx_buf + sizeof(lwb_pkt_header_t))
#define LWB_PKT_DATA_LEN_MAX()          (glossy_get_max_payload_len(lwb_context.enc) - sizeof(lwb_pkt_header_t))
#define LWB_PKT_APP_DATA_PTR()          (lwb_context.txrx_buf + sizeof(lwb_pkt_header_t) + sizeof(data_header_t))
#define LWB_PKT_APP_DATA_LEN_MAX()      (LWB_PKT_DATA_LEN_MAX() - sizeof(data_header_t))
#define LWB_PKT_APP_DATA_HDR_OPT_SET_PKT_TYPE(hdr, type)  (hdr)->options |= (type) & 0x0f
#define LWB_PKT_APP_DATA_HDR_OPT_GET_PKT_TYPE(hdr)        ((hdr)->options & 0x0f)
/// @}


#define LWB_SCHED_GET_MAX_BW(period, n_free)  ((((period) * RTIMER_SECOND) \
                                                 - T_SYNC_ON \
                                                 - T_COMP \
                                                 - ((n_free) * T_FREE_ON) \
                                                 - ((n_free) * T_GAP)) \
                                               / (T_GAP + T_RR_ON))

/// @addtogroup rtimer scheduling
///             macro for rtimer based scheduling
/// @{
#define SCHEDULE(ref, offset, cb)   rtimer_set(&lwb_context.rt, ref + offset, 1, (rtimer_callback_t)cb, &lwb_context)
#define SCHEDULE_L(ref, offset, cb) rtimer_set_long(&lwb_context.rt, ref, offset, (rtimer_callback_t)cb, &lwb_context)

#define LWB_WAIT_UNTIL(time) \
{\
  rtimer_set(rt, (time), 0, (rtimer_callback_t)pt_state->cb, pt_state);\
  PT_YIELD(pt_state->pt);\
}

///  @}

#endif // __LWB_MACROS_H___
