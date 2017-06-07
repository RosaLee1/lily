/*
 * Copyright (c) 2008 Dimas Abreu Dutra
 * All rights reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL DIMAS ABREU
 * DUTRA OR HIS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @author Dimas Abreu Dutra
 */

#ifndef RSSIDEMOMESSAGES_H__
#define RSSIDEMOMESSAGES_H__

enum {
  AM_RSSIMSG = 10
};

typedef nx_struct sender_msg
{
  nx_uint16_t    src_addr;
  nx_uint16_t    counter;
  nx_uint32_t    local_rx_timestamp;
  nx_uint32_t    global_rx_timestamp;
  nx_int32_t     skew_times_1000000;
  nx_uint8_t     is_synced;
  nx_uint16_t    ftsp_root_addr;
  nx_uint8_t     ftsp_seq;
  nx_uint8_t     ftsp_table_entries;
  nx_uint16_t    addr;
  nx_uint16_t    count;
  nx_uint32_t    local_timestamp;
  nx_uint32_t    global_timestamp;
  nx_int32_t     skew_times;
  nx_uint8_t     synced;
  nx_uint16_t    root_addr;
  nx_uint8_t     seq;
  nx_uint8_t     table_entries;
  nx_uint16_t    my_addr;
  nx_uint16_t    my_count;
  nx_uint32_t    my_local_timestamp;
  nx_uint32_t    my_global_timestamp;
  nx_int32_t     my_skew_times;
  nx_uint8_t     my_synced;
  nx_uint16_t    my_root_addr;
  nx_uint8_t     my_seq;
  nx_uint8_t     my_table_entries;
  nx_uint16_t    your_addr;
  nx_uint16_t    your_count;
  nx_uint32_t    your_local_timestamp;
  nx_uint32_t    your_global_timestamp;
  nx_int32_t     your_skew_times;
  nx_uint8_t     your_synced;
  nx_uint16_t    your_root_addr;
  nx_uint8_t     your_seq;
  nx_uint8_t     your_table_entries;
  nx_uint16_t    her_addr;
  nx_uint16_t    her_count;
  nx_uint32_t    her_local_timestamp;
  nx_uint32_t    her_global_timestamp;
  nx_int32_t     her_skew_times;
  nx_uint8_t     her_synced;
  nx_uint16_t    her_root_addr;
  nx_uint8_t     her_seq;
  nx_uint8_t     her_table_entries;
} sender_msg_t;

#endif //RSSIDEMOMESSAGES_H__
