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

#ifndef __LWB_COMMON_H__
#define __LWB_COMMON_H__

/// @file lwb-common.h
/// @brief Common header file

#include <stdint.h>

#include "contiki.h"
#include "glossy.h"

#ifdef LWB_CUSTOM_CONF_H
#include LWB_CUSTOM_CONF_H
#endif

#include "lwb-default-conf.h"

#define LWB_MAX_SLOTS_UNIT_TIME ((uint8_t)((RTIMER_SECOND - T_SYNC_ON - T_HSLP_SCHED)/(T_GAP + T_RR_ON)) - 1)

/// @brief LWB mode.
typedef enum {
    LWB_STATUS_SUCCESS,  ///< Indicates successful status.
    LWB_STATUS_FAIL      ///< Indicates failed status.
} lwb_status_t;

/// @brief LWB mode.
typedef enum {
    LWB_MODE_HOST,  ///< LWB in host mode.
    LWB_MODE_SOURCE     ///< LWB in source mode.
} lwb_mode_t;

/// @brief Stream request types
typedef enum {
    LWB_STREAM_TYPE_ADD = 1,     ///< Stream add request.
    LWB_STREAM_TYPE_DEL,         ///< Stream delete request.
    LWB_STREAM_TYPE_MOD          ///< Stream modify request.
} stream_req_types_t;

/// @brief LWB packet types
typedef enum {
    LWB_PKT_TYPE_NO_DATA,       ///< No Type
    LWB_PKT_TYPE_STREAM_REQ,    ///< Stream requests
    LWB_PKT_TYPE_STREAM_ACK,    ///< Stream acknowledgements
    LWB_PKT_TYPE_DATA,          ///< Data packets
    LWB_PKT_TYPE_SCHED          ///< Schedule packets
} pkt_types_t;

/// @brief Synchronization states
typedef enum {
    LWB_SYNC_STATE_BOOTSTRAP,      ///< Initial state. Not synchronized
    LWB_SYNC_STATE_QUASI_SYNCED,       ///< Received schedule when in BOOTSTRAP state. Clock drift is not estimated.
    LWB_SYNC_STATE_SYNCED,             ///< Received schedule and clock drift is estimated.
    LWB_SYNC_STATE_UNSYNCED_1,         ///< Missed schedule one time.
    LWB_SYNC_STATE_UNSYNCED_2,         ///< Missed schedule two times.
    LWB_SYNC_STATE_UNSYNCED_3,         ///< Missed schedule three times.
} lwb_sync_state_t;

typedef enum {
    LWB_JOINING_STATE_NOT_JOINED,
    LWB_JOINING_STATE_JOINING,
    LWB_JOINING_STATE_PARTLY_JOINED,
    LWB_JOINING_STATE_JOINED
} lwb_joining_state_t;

typedef enum {
  LWB_RUN_STATE_STOPPED,
  LWB_RUN_STATE_ACTIVE
} lwb_run_state_t;

typedef enum {
    LWB_POLL_FLAGS_DATA,
    LWB_POLL_FLAGS_SCHED_END,
    LWB_POLL_FLAGS_SLOT_END
} lwb_poll_flags_t;

/// @brief Header to be used when sending LWB packets
typedef struct __attribute__ ((__packed__)) {
  uint8_t  pkt_type;  ///< Data options
} lwb_pkt_header_t;


/// @brief Header to be used when sending data
typedef struct __attribute__ ((__packed__)) {
  uint16_t to_id;    ///< To node ID
  uint8_t  data_len;  ///< Data options
  uint8_t  in_queue;  ///< Number of packets in queue that are ready to be sent
  uint8_t  options;   ///< Data options
} data_header_t;


/// @brief This header is used when sending stream requests as separate messages.
typedef struct __attribute__ ((__packed__)) {
  uint8_t     n_reqs;       ///< Number of stream requests
} lwb_stream_req_header_t;

/// @brief Header to be used when sending stream acknowledgement
typedef struct __attribute__ ((__packed__)) {
  uint8_t  n_acks;  ///< NUmber of acknowledgement
} lwb_stream_ack_header_t;

/// @brief Structure for stream requests
typedef struct __attribute__ ((__packed__)) lwb_stream_req {
  uint16_t ipi;           ///< Inter-packet interval in seconds.
  uint16_t time_info;     ///< Starting time of the stream.
  uint8_t  req_type;      ///< Request type. Least significant 2 bits represent stream request type.
                         ///  Most significant 6 bits represent request ID.
                         ///  @see stream_req_types_t and GET_STREAM_ID and SET_STREAM_ID
} lwb_stream_req_t;

/// @brief Structure for the header of a schedule
typedef struct __attribute__ ((__packed__)) {
  uint32_t time;          ///< The current time at the host.
  uint16_t round_period; ///< Round period (duration between beginning of two rounds) in seconds
  uint8_t  n_slots;       ///< Number of slots in the round.
                          ///  Most significant 2 bits represent free slots and least significant 6 bits represent data slots.
} lwb_sched_info_t;

/// @brief LWB schedule
typedef struct __attribute__ ((__packed__)) {
  lwb_sched_info_t    sched_info;                    ///< schedule information
  uint16_t            slots[LWB_SCHED_MAX_SLOTS];    ///< slots. The node ID will be stored.
} lwb_schedule_t;

typedef struct lwb_stream_info {
  struct lwb_stream_info *next;
  uint16_t node_id;             ///< Node ID
  uint16_t ipi;                 ///< Inter-packet interval in seconds
  uint32_t last_assigned;
  uint32_t next_ready;
  uint8_t  stream_id;           ///< Stream ID
  uint8_t  n_cons_missed;       ///< Number of consecutive slot misses for the stream
  uint8_t  n_used;        ///< Number of slots used in a round
  uint8_t  n_allocated;   ///< Number of allocated slots for the stream in a round

  uint8_t  avg_max_qlen;
  uint8_t  max_qlen;            ///< Maximum length of the queue at the source node
  uint8_t  max_qlen_tmp;
  uint8_t  qlens[LWB_SCHED_MAX_QLEN_WNIDOW_SIZE];
  uint8_t  n_qlens;

} lwb_stream_info_t;

/// @brief LWB callbacks
typedef struct lwb_callbacks {
  void (*p_on_data)(uint8_t*, uint8_t, uint16_t);
  void (*p_on_sched_end)(void);
} lwb_callbacks_t;


/// @brief Scheduler related statistics
typedef struct lwb_sched_stats {
  uint16_t n_added;          ///< Number of streams added so far
  uint16_t n_deleted;        ///< Number of streams deleted so far
  uint16_t n_no_space;       ///< Number of streams that are unable to add due to space unavailability
  uint16_t n_modified;       ///< Number of streams modified
  uint16_t n_duplicates;     ///< Number of duplicated stream requests
  uint16_t n_unused_slots;
} lwb_sched_stats_t;

/// @brief Glossy synchronization related statistics
typedef struct lwb_sync_stats {
  uint16_t n_synced;           ///< Number of instances that the schedule is received
  uint16_t n_sync_missed;      ///< Number of instances that the schedule is not received
  uint8_t n_rx;
  uint8_t relay_cnt_first_rx;
} lwb_sync_stats_t;

/// @brief Statistics related to data packets
typedef struct lwb_data_stats {
  uint16_t n_tx_nospace;     ///< Number of instances in which unable to allocate memory for transmitting data
  uint16_t n_rx_nospace;     ///< Number of instances in which unable to allocate memory for receiving data
  uint16_t n_tx;             ///< Number of data packets transmitted
  uint16_t n_rx;             ///< Number of data packets received
  uint16_t n_rx_dropped;     ///< Number of data packets dropped
} lwb_data_stats_t;

/// @brief Stream requests and acknowledgement related statistics
typedef struct lwb_stream_req_ack_stats {
  uint16_t n_req_tx;         ///< Number of stream requests sent
  uint16_t n_req_rx;         ///< Number of stream requests received
  uint16_t n_ack_tx;         ///< Number of stream acknowledgements sent
  uint16_t n_ack_rx;         ///< Number of stream acknowledgements received
} lwb_stream_req_ack_stats_t;


/// @brief LWB context
typedef struct {

  lwb_schedule_t   current_sched;                     /**< Current schedule */
  lwb_schedule_t   old_sched;                         /**< Old schedule */
  uint32_t         time;                              /**< Global time in seconds */
  int32_t          skew;                              /**< Clock skew per second in rtimer ticks */
  rtimer_clock_t   t_sync_guard;                      /**< Guard time used for starting Glossy earlier for receiving schedule in rtimer ticks */
  rtimer_clock_t   t_sync_ref;                        /**< Time at the host when schedule is transmitted in rtimer ticks */
  rtimer_clock_t   t_last_sync_ref;                   /**< Time at the host when the last schedule is transmitted in rtimer ticks */
  rtimer_clock_t   t_start;                           /**< Start time of the first schedule transmission in a round in rtimer ticks */
  lwb_callbacks_t* p_callbacks;                       /**< Call back functions */
  uint8_t          lwb_mode;                          /**< Mode of LWB @see lwb_mode_t */
  struct rtimer    rt;                                /**< The rtimer used to start glossy phases */
  uint8_t          txrx_buf[LWB_MAX_TXRX_BUF_LEN];    /**< TX/RX buffer */
  uint8_t          txrx_buf_len;                      /**< The length of the data in TX/RX buffer */
  uint8_t          poll_flags;                        /**< Flags that indicate why LWB main process is polled */
  uint8_t          n_my_slots;                        /**< Number of slots allocated for the node */
  glossy_enc_t     enc;

  // source node
  volatile lwb_sync_state_t     sync_state;           /**< Synchronization state */
  volatile lwb_joining_state_t  joining_state;        /**< Joining state */
  volatile lwb_run_state_t      run_state;

  // host
  uint16_t        stream_akcs[LWB_SCHED_MAX_SLOTS];   /**< IDs of the nodes which stream acknowledgements to be sent */
  uint8_t         n_stream_acks;                      /**< Number of stream acknowledgements */

  // stats
  lwb_sync_stats_t            sync_stats;
  lwb_data_stats_t            data_stats;
  lwb_stream_req_ack_stats_t  stream_req_ack_stats;
  lwb_sched_stats_t           sched_stats;

  // for radio duty cycle of control packets
#if LWB_CTRL_ENERGEST_ON || LWB_SLOT_ENERGEST_ON
  uint32_t en_rx;
  uint32_t en_tx;
#endif
#if LWB_CTRL_ENERGEST_ON
  uint32_t en_control;
#endif
#if LWB_SLOT_ENERGEST_ON
  uint32_t en_slots[LWB_SCHED_MAX_SLOTS];
  uint8_t  n_en_slots;
#endif

} lwb_context_t;

typedef struct {
  struct pt* pt;
  void* cb;
} pt_state_t;


/// @brief Structure for data buffer element.
typedef struct data_buf {
  data_header_t   header;
  uint8_t         data[LWB_MAX_TXRX_BUF_LEN];
} data_buf_t;

/// @brief Structure for data buffer element.
typedef struct data_buf_lst_item {
  struct data_buf* next;
  uint16_t from_id;
  data_buf_t        buf;
} data_buf_lst_item_t;


typedef struct stream_req_lst_item {
  struct stream_req_lst_item* next;
  lwb_stream_req_t            req;
} stream_req_lst_item_t;


/// @defgroup Stream and schedule related macros
/// @{
#define LWB_GET_STREAM_ID(req_opt)                  (req_opt >> 2)
#define LWB_SET_STREAM_ID(req_opt, id)              (req_opt = (id << 2) | (req_opt & 0x03))
#define LWB_SET_STREAM_TYPE(req_opt, type)          (req_opt = ((req_opt >> 2) << 2) | (type & 0x03))
#define LWB_GET_STREAM_TYPE(req_opt)                (req_opt & 0x03)
#define LWB_GET_N_FREE_SLOTS(a)                     (a >> 6)
#define LWB_SET_N_FREE_SLOTS(slots, free_slots)     (slots = (slots & 0x3f) | (free_slots << 6))
#define LWB_GET_N_DATA_SLOTS(a)                     (a & 0x3f)
#define LWB_SET_N_DATA_SLOTS(slots, data_slots)     (slots = (slots & 0xc0) | (data_slots & 0x3f))
/// @}

/// @addtogroup UI32 Macros
///           Unsign 32-bit integer lated macros to get/set low/high segments.
/// @{
#define UI32_GET_LOW(var)        (uint16_t)(var & 0xffff)
#define UI32_GET_HIGH(var)       (uint16_t)(var >> 16)
#define UI32_SET_LOW(var, val)   (var = ((uint32_t)UI32_GET_HIGH(var) << 16) | (uint32_t)((val) & 0xffff))
#define UI32_SET_HIGH(var, val)  (var = ((uint32_t)(val) << 16) | (uint32_t)UI32_GET_LOW(var))
/// @}

/// @addtogroup glossy macros
/// @{
/// @brief Reference time of Glossy.
#define GLOSSY_T_REF                (glossy_get_t_ref())
/// @brief Check if glossy's reference time is updated.
#define GLOSSY_IS_SYNCED()          (glossy_is_t_ref_updated())
/// @brief Set glossy's reference time not updated.
#define GLOSSY_SET_UNSYNCED()       (set_t_ref_l_updated(0))
/// @}

void lwb_save_energest();

void lwb_update_ctrl_energest();

void lwb_reset_slot_energest();

void lwb_update_slot_energest();

#endif // __LWB_COMMON_H__
