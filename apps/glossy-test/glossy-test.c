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
#include "dev/leds.h"
#include "sys/rtimer.h"

#include "glossy.h"
#include "deployment.h"


#define INITIATOR_ID                    1

#define GLOSSY_PERIOD                   (RTIMER_SECOND * 2)        /* 1 second */
#define GLOSSY_T_SLOT                   (RTIMER_SECOND / 33)       /* 30 ms*/
#define GLOSSY_T_GUARD                  (RTIMER_SECOND / 1000)     /* 1ms */
#define GLOSSY_N_TX                     3

#define WAIT_UNTIL(time) \
{\
  rtimer_set(&g_timer, (time), 0, (rtimer_callback_t)glossy_thread, rt);\
  PT_YIELD(&glossy_pt);\
}

PROCESS(glossy_test, "Glossy test");
AUTOSTART_PROCESSES(&glossy_test);

typedef struct {
  uint32_t seq_no;
  uint8_t  data[20];
} glossy_data_t;

static struct pt      glossy_pt;
static struct rtimer  g_timer;
static glossy_data_t  glossy_payload;
static uint8_t        sync_state = 0;
static uint16_t       bootstrap_cnt = 0;
static uint16_t       pkt_cnt = 0;
static uint16_t       miss_cnt = 0;

static uint8_t aes_key[32];

/*------------------------------------------------------------------------------------------------*/
PT_THREAD(glossy_thread(struct rtimer *rt))
{

  PT_BEGIN(&glossy_pt);

  printf("Starting Glossy. Node ID %u\n", node_id);

  while (1) {

    if(node_id == INITIATOR_ID) {

      glossy_start(node_id,
                   (uint8_t*)&glossy_payload,
                   sizeof(glossy_data_t),
                   GLOSSY_N_TX,
                   GLOSSY_WITH_SYNC);

      WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
      glossy_stop();

      printf("n_rx %"PRIu8", f_relay_cnt %"PRIu8", sent_seq %"PRIu32"\n",
              glossy_get_n_rx(),
              glossy_get_relay_cnt_first_rx(),
              glossy_payload.seq_no);

      glossy_debug_print();

      glossy_payload.seq_no++;

      WAIT_UNTIL(rt->time - GLOSSY_T_SLOT + GLOSSY_PERIOD);

    } else {

      if(!sync_state) {

        printf("BOOTSTRAP\r\n");

        bootstrap_cnt++;

        do {
          glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload,
                       GLOSSY_UNKNOWN_PAYLOAD_LEN,
                       GLOSSY_N_TX, GLOSSY_WITH_SYNC);
          WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
          glossy_stop();

        } while(!glossy_is_t_ref_updated());

        /* synchronized! */
        sync_state = 1;

      } else {

        /* already synchronized, receive a packet */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR, (uint8_t*)&glossy_payload,
                     GLOSSY_UNKNOWN_PAYLOAD_LEN,
                     GLOSSY_N_TX, GLOSSY_WITH_SYNC);
        WAIT_UNTIL(rt->time + GLOSSY_T_SLOT + GLOSSY_T_GUARD);
        glossy_stop();
      }

      /* at least one packet received? */
      if(glossy_get_n_rx()) {
        pkt_cnt++;
      } else {
        miss_cnt++;
      }

      /* has the reference time been updated? */
      if(glossy_is_t_ref_updated()) {
        /* sync received */
      } else {
        /* sync missed */
        sync_state = 0;
        continue;
      }

      printf("rcvd_seq %"PRIu32"\n", glossy_payload.seq_no);
      printf("n_rx %"PRIu8", f_relay_cnt %"PRIu8", rcvd %"PRIu16" missed %"PRIu16" bootpd %"PRIu16"\n",
              glossy_get_n_rx(),
              glossy_get_relay_cnt_first_rx(),
              pkt_cnt,
              miss_cnt,
              bootstrap_cnt);

      WAIT_UNTIL(glossy_get_t_ref() + GLOSSY_PERIOD - GLOSSY_T_GUARD);
    }

  }

  PT_END(&glossy_pt);
}

/*------------------------------------------------------------------------------------------------*/
PROCESS_THREAD(glossy_test, ev, data)
{
    PROCESS_BEGIN();

    deployment_load_ieee_addr();
    deployment_set_node_id_ieee_addr();
    deployment_print_id_info();

    glossy_init();

    aes_key[0] = 0xa5;  aes_key[1] = 0x86;
    aes_key[2] = 0xe5;  aes_key[3] = 0x1a;
    aes_key[4] = 0x96;  aes_key[5] = 0xfe;
    aes_key[6] = 0x56;  aes_key[7] = 0xaa;
    aes_key[8] = 0x7f;  aes_key[9] = 0xa3;
    aes_key[10] = 0xb4; aes_key[11] = 0x70;
    aes_key[12] = 0xee; aes_key[13] = 0xe6;
    aes_key[14] = 0x0b; aes_key[15] = 0x04;
    glossy_set_enc_key(aes_key, GLOSSY_AES_128_KEY_SIZE);

    glossy_set_enc(GLOSSY_ENC_ON);

    glossy_payload.seq_no = 0;

    rtimer_set(&g_timer, RTIMER_NOW() + RTIMER_SECOND * 2, 0, (rtimer_callback_t)glossy_thread, NULL);

    PROCESS_END();
}

