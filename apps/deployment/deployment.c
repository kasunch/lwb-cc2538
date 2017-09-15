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
#include <inttypes.h>
#include <stdio.h>

#include "contiki-conf.h"
#include "sys/node-id.h"
#include "ieee-addr.h"
#include "deployment.h"

#define IEEE_ADDR_LEN 8

#define DEPLOYMENT_DEBUG 1

#if DEPLOYMENT_DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif /* DEPLOYMENT_DEBUG */

static uint8_t ieee_addr[IEEE_ADDR_LEN];

struct id_addr {
  uint16_t id;
  uint8_t ieee_addr[IEEE_ADDR_LEN];
};


static struct id_addr id_addr_list[] = { 

  {1, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb5, 0xf0}},
  {2, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb4, 0x59}},
  {3, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb4, 0x79}},
  {4, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0x9a, 0xd0}},
  {5, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0x9a, 0xca}},
  {5, {0x00, 0x12, 0x4b, 0x00, 0x06, 0x0d ,0xb1, 0x47}},

  {0, {0, 0, 0, 0, 0, 0 ,0, 0}}
};

/* The total number of nodes in the deployment */
#define N_NODES ((sizeof(id_addr_list)/sizeof(struct id_addr))-1)

/* Returns the total number of nodes in the deployment */
uint16_t deployment_get_n_nodes(void)
{
  return N_NODES;
}

void deployment_load_ieee_addr(void)
{
  ieee_addr_cpy_to(ieee_addr, IEEE_ADDR_LEN);
}

uint8_t deployment_set_node_id_ieee_addr(void)
{
  struct id_addr *curr = id_addr_list;
  while (curr->id != 0) {
    if (memcmp(ieee_addr, curr->ieee_addr, IEEE_ADDR_LEN) == 0) {
      node_id_burn(curr->id);
      return 1;
    }
    curr++;
  }

  return 0;
}

void deployment_print_id_info(void)
{
  PRINTF("[DEPLOYMENT] Node ID  : %"PRId16"\n", node_id);
  PRINTF("[DEPLOYMENT] IEEE ADDR: %02x %02x %02x %02x %02x %02x %02x %02x\n",
         ieee_addr[0], ieee_addr[1],
         ieee_addr[2], ieee_addr[3],
         ieee_addr[4], ieee_addr[5],
         ieee_addr[6], ieee_addr[7]);
}

