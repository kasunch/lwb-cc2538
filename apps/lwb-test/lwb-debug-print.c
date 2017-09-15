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

#include <stdio.h>
#include <inttypes.h>

#include "contiki.h"

#include "lwb-debug-print.h"
#include "lwb.h"
#include "lwb-macros.h"

extern lwb_context_t lwb_context;

/*------------------------------------------------------------------------------------------------*/
void lwb_debug_print()
{
  uint8_t i;
  lwb_schedule_t* sched = lwb_context.lwb_mode == LWB_MODE_HOST ? &OLD_SCHEDULE() : &CURRENT_SCHEDULE();

  if (lwb_context.lwb_mode == LWB_MODE_HOST ||
      (lwb_context.sync_state == LWB_SYNC_STATE_QUASI_SYNCED
          || lwb_context.sync_state == LWB_SYNC_STATE_SYNCED
          || lwb_context.sync_state == LWB_SYNC_STATE_UNSYNCED_1)) {

    printf("sched : ");
    for (i = 0; i < LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots); i++) {
      printf("%04X ", sched->slots[i]);
    }
    printf("\n");

  }

  printf("time %"PRIu32", n_slots [data %"PRIu8", free %"PRIu8", my %"PRIu8"], T %"PRIu8"\n",
         sched->sched_info.time,
         LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots),
         LWB_GET_N_FREE_SLOTS(sched->sched_info.n_slots),
         lwb_context.n_my_slots,
         sched->sched_info.round_period);

  printf("time %"PRIu32", state %"PRIu8", skew %"PRId32", guard %"PRIu32"\n",
         sched->sched_info.time,
         lwb_context.sync_state,
         lwb_context.skew,
         lwb_context.t_sync_guard);

  printf("time %"PRIu32", sync rc_first_rx %"PRIu8", n_rx %"PRIu8"\n",
         sched->sched_info.time,
         lwb_context.sync_stats.relay_cnt_first_rx,
         lwb_context.sync_stats.n_rx);

  lwb_context.sync_stats.n_rx = 0;
  lwb_context.sync_stats.relay_cnt_first_rx = 0;

  printf("-------- %s --------\n", CONTIKI_VERSION_STRING);

}
