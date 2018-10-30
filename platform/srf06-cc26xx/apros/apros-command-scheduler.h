/*
 * Copyright (c) 2005, Swedish Institute of Computer Science
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */
 /**
  * \file
  * Generic serial I/O process header filer
  * \author
  * Adam Dunkels
  * Sanghyun Kim
  *
  */
#ifndef APROS_COMMAND_SCHEDULER_H_
#define APROS_COMMAND_SCHEDULER_H_
#include "contiki.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "apros-packet-control.h"

#include <string.h>

#define APROS_COMM_PRIOR_1    0
#define APROS_COMM_PRIOR_2   (CLOCK_SECOND>>5)
#define APROS_COMM_PRIOR_3   (CLOCK_SECOND>>4)
#define APROS_COMM_PRIOR_4   (CLOCK_SECOND>>3)
#define APROS_COMM_PRIOR_5   (CLOCK_SECOND>>2)
#define APROS_COMM_PRIOR_6   (CLOCK_SECOND>>1)
#define APROS_COMM_PRIOR_7   (CLOCK_SECOND)
#define APROS_COMM_PRIOR_8   (CLOCK_SECOND + APROS_COMM_PRIOR_6)
#define APROS_COMM_PRIOR_9   (CLOCK_SECOND<<1)
/*---------------------------------------------------------------------------*/
typedef struct error_cb_st{
	struct msg_info* msg_info;
	uint8_t error_type;
}error_cb_st_t;
/*---------------------------------------------------------------------------*/
typedef struct msg_info {
	struct node_info* p_node_info;
	struct cont_block_info* p_cont_block_info;
#if APROS_DEV_TYPE == BASE
	struct ctimer timeout;
	uint8_t is_processing:1;
#endif
}msg_info_t;

#define MAX_NUM_MSG 40

typedef struct node_info {
	uint8_t node_id[8];
	uint8_t msg_num;
#if APROS_DEV_TYPE == NODE
	msg_info_t* msg_info;
#elif APROS_DEV_TYPE == BASE
	msg_info_t msg_info[MAX_NUM_MSG];
#else
#endif
}node_info_t;
/*---------------------------------------------------------------------------*/
#if APROS_DEV_TYPE == BASE
node_info_t* add_node_info(uint8_t node_id[8]);
msg_info_t* add_msg_info(node_info_t* node_info, uint8_t type);
node_info_t* find_node_info(uint8_t node_id[8]);
msg_info_t* find_msg_info(uint8_t node_id[8], uint8_t type);
#if APROS_TIMEOUT_ENABLE 
void set_timeout(msg_info_t* msg_info, clock_time_t time, void (*func)(void*));
void restart_timeout(msg_info_t* msg_info);
void release_timeout(msg_info_t* msg_info);
uint8_t is_processing_msg(msg_info_t* msg_info);
void set_processing_msg(msg_info_t* msg_info);
void clear_processing_msg(msg_info_t* msg_info);
#endif
#endif
/*---------------------------------------------------------------------------*/
typedef struct apros_comm_param{
	void (*func)(void*);
	void* func_param;
/*#if APROS_LED_EXIST
	uint8_t led;
#endif*/
	clock_time_t t_backoff;
}apros_comm_param_t;

/**** Caution : 
This function guarantees success only for func_param for a single reference(maximum : 40 Bytes).
For func_param, which be must perform beyond double reference, 
the user is responsible for variable memory management.

in that case, Recommend: declares static variables or global variables as the pointers in the func_param.
****/

void apros_command(apros_comm_param_t comm);

#endif
