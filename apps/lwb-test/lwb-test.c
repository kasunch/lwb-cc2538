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
#include "lpm.h"

#include "lwb.h"
#include "deployment.h"
#include "lwb-debug-print.h"


PROCESS(lwb_test_process, "lwb test");
AUTOSTART_PROCESSES(&lwb_test_process);

typedef struct __attribute__ ((__packed__)) {
  uint32_t seq;
  uint8_t data[20];
} app_data_t;

static app_data_t app_data;
static struct etimer et;
static uint8_t aes_key[32];

#define LWB_HOST_ID  1
#define STREAM_ID    2
#define STREAM_IPI   5

/*------------------------------------------------------------------------------------------------*/
void on_schd_end(void)
{
  lwb_debug_print();
}

/*------------------------------------------------------------------------------------------------*/
void on_data(uint8_t *data, uint8_t data_len, uint16_t from_id)
{
  app_data_t* data_ptr = (app_data_t*)data;
  printf("DATA from %"PRIu16", seq %"PRIu32"\n", from_id, data_ptr->seq);
  /* Handle what happens to the received data in here */
}

lwb_callbacks_t callbacks = { on_data, on_schd_end };

/*------------------------------------------------------------------------------------------------*/
PROCESS_THREAD(lwb_test_process, ev, data)
{
  PROCESS_BEGIN();

  deployment_load_ieee_addr();
  deployment_set_node_id_ieee_addr();
  deployment_print_id_info();
  /* Need to change to LPM_PM0 to be able to use etimer events*/
  lpm_set_max_pm(LPM_PM0);


  aes_key[0] = 0xa5;  aes_key[1] = 0x86;
  aes_key[2] = 0xe5;  aes_key[3] = 0x1a;
  aes_key[4] = 0x96;  aes_key[5] = 0xfe;
  aes_key[6] = 0x56;  aes_key[7] = 0xaa;
  aes_key[8] = 0x7f;  aes_key[9] = 0xa3;
  aes_key[10] = 0xb4; aes_key[11] = 0x70;
  aes_key[12] = 0xee; aes_key[13] = 0xe6;
  aes_key[14] = 0x0b; aes_key[15] = 0x04;


  if (node_id == LWB_HOST_ID) {
    /* Initialize LWB as the host */
    lwb_init(LWB_MODE_HOST, &callbacks);
    /* Set Glossy encryption key */
    glossy_set_enc_key(aes_key, GLOSSY_AES_128_KEY_SIZE);
    /* Enable Glossy encryption */
    glossy_set_enc(GLOSSY_ENC_ON);

  } else {
    /* Initialize LWB as a source */
    lwb_init(LWB_MODE_SOURCE, &callbacks);
    /* Set Glossy encryption key */
    glossy_set_enc_key(aes_key, GLOSSY_AES_128_KEY_SIZE);
    /* Enable Glossy encryption */
    glossy_set_enc(GLOSSY_ENC_ON);

    /* Send stream modify request with ID=1 and IPI=5 seconds */
    lwb_request_stream_mod(STREAM_ID, STREAM_IPI);
    /* Set event timer to expire in 5 seconds */
    etimer_set(&et, CLOCK_SECOND * STREAM_IPI);

    while (1) {
      /* Wait until event timer expires */
      PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
      app_data.seq++;
      /* Queue data packet which is to be sent via LWB for all nodes */
      lwb_queue_packet((uint8_t*)&app_data, sizeof(app_data_t), 0);
      /* Set event timer to expire in 5 seconds */
      etimer_set(&et, CLOCK_SECOND * STREAM_IPI);
    }
  }

  PROCESS_END();

  return PT_ENDED;
}

