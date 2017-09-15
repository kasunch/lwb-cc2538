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
 
#include "lwb-sched-compressor.h"
#include <string.h>


#define COMP_BUFFER_LEN 128
static uint8_t comp_buf[COMP_BUFFER_LEN];

/*------------------------------------------------------------------------------------------------*/
static inline uint8_t get_n_bits(uint16_t a) {
    uint8_t i;
    for (i = 15; i > 0; i--) {
        if (a & (1 << i)) {
            return i + 1;
        }
    }
    return i + 1;
}

/*------------------------------------------------------------------------------------------------*/
uint8_t lwb_sched_compress(lwb_schedule_t *sched, uint8_t* buf, uint8_t buf_len)
{
  uint8_t i;
  uint8_t max_n_bits;
  uint8_t n_bits;
  uint32_t tmp;
  uint16_t bit_start;
  uint8_t req_len;

  max_n_bits = 0;
  for (i = 0; i < LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots); i++) {
    n_bits = get_n_bits(sched->slots[i]);
    if (n_bits > max_n_bits) {
      max_n_bits = n_bits;
    }
  }

  req_len = (max_n_bits * LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots)) / 8;
  req_len += ((max_n_bits * LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots)) % 8) ? 1 : 0;

  if (buf_len < req_len + 1) {
    return 0;
  }

  if (COMP_BUFFER_LEN < ((max_n_bits * LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots) / 8) + 3)) {
    return 0;
  }

  memset(comp_buf, 0, COMP_BUFFER_LEN);

  for (i = 0; i < LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots); i++) {

    bit_start = (uint16_t)i * (uint16_t)max_n_bits;

    tmp = (uint32_t) comp_buf[(bit_start / 8)]
           | ((uint32_t) comp_buf[(bit_start / 8) + 1] << 8)
           | ((uint32_t) comp_buf[(bit_start / 8) + 2] << 16);

    tmp |= (uint32_t) sched->slots[i] << (bit_start % 8);

    comp_buf[(bit_start / 8)] = (uint8_t) (tmp & 0xFF);
    comp_buf[(bit_start / 8) + 1] = (uint8_t) ((tmp >> 8) & 0xFF);
    comp_buf[(bit_start / 8) + 2] = (uint8_t) ((tmp >> 16) & 0xFF);

  }

  buf[0] = max_n_bits;
  memcpy(buf + 1, comp_buf, req_len);

  return req_len + 1;
}

/*------------------------------------------------------------------------------------------------*/
uint8_t lwb_sched_decompress(lwb_schedule_t *sched, uint8_t* buf, uint8_t buf_len)
{
  uint8_t ui8_i = 0;
  uint8_t max_n_bits = 0;
  uint32_t tmp = 0;
  uint16_t bit_start = 0;

  if (buf_len < 1) {
    return 0;
  }

  max_n_bits = buf[0];

  if (COMP_BUFFER_LEN < ((max_n_bits * LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots) / 8) + 3)) {
    return 0;
  }

  memset(comp_buf, 0, COMP_BUFFER_LEN);
  memcpy(comp_buf, buf + 1, buf_len - 1);

  for (ui8_i = 0; ui8_i < LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots); ui8_i++) {

    bit_start = (uint16_t) ui8_i * (uint16_t) max_n_bits;

    tmp = (uint32_t) comp_buf[bit_start / 8]
          | ((uint32_t) comp_buf[bit_start / 8 + 1] << 8)
          | ((uint32_t) comp_buf[bit_start / 8 + 2] << 16);

    sched->slots[ui8_i] = (tmp >> (bit_start % 8)) & ((1 << max_n_bits) - 1);
  }

  return LWB_GET_N_DATA_SLOTS(sched->sched_info.n_slots);
}
