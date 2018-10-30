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

#include "apros-command-scheduler.h"

/*---------------------------------------------------------------------------*/
/* Global Variables  */
#define MAX_COMM_INBUF 15
#define MAX_PARAM_SIZE 40//hard coding

static struct ctimer ct_comm_buffer[MAX_COMM_INBUF];
static uint8_t param_cp[MAX_COMM_INBUF][MAX_PARAM_SIZE];

/*#if APROS_LED_EXIST
static struct ctimer ct_comm_led_buffer[MAX_COMM_INBUF]; 
static uint8_t led_cp[MAX_COMM_INBUF];
#endif*/
#if APROS_DEV_TYPE == BASE
#define MAX_NODE_UNDER_BASE 7
static node_info_t node_cache[MAX_NODE_UNDER_BASE];
static uint8_t node_num;
/*---------------------------------------------------------------------------*/
/* Functions */
node_info_t* add_node_info(uint8_t node_id[8]){
	node_info_t* node_info = find_node_info(node_id);
	if(node_info == NULL && node_num < MAX_NODE_UNDER_BASE){
		memcpy(node_cache[node_num].node_id, node_id, 8);
		node_cache[node_num].msg_num = 0;

		return &node_cache[node_num++];
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
msg_info_t* add_msg_info(node_info_t* node_info, uint8_t type){
	if(node_info == NULL){
		return NULL;
	}else{
		msg_info_t* msg_info = find_msg_info(node_info->node_id, type);
		if(msg_info == NULL){
			if(node_info->msg_num < MAX_NUM_MSG){
				node_info->msg_info[node_info->msg_num].p_node_info = node_info;
				node_info->msg_info[node_info->msg_num].p_cont_block_info = find_cont_block_info(type);
				node_info->msg_info[node_info->msg_num].is_processing = 0;
				return &node_info->msg_info[node_info->msg_num++];
			}else{
				node_info->msg_info[0].p_node_info = node_info;
				node_info->msg_info[0].p_cont_block_info = find_cont_block_info(type);
				node_info->msg_info[0].is_processing = 0;
				return &node_info->msg_info[0];
			}
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
node_info_t* find_node_info(uint8_t node_id[8]){
	uint8_t i;
	for( i = 0 ; i < node_num ; i ++ ){
		if(!memcmp(node_cache[i].node_id, node_id, 8)){
			return &node_cache[i]; //O(n)
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
msg_info_t* find_msg_info(uint8_t node_id[8], uint8_t type){
	uint8_t i;
	node_info_t* node_info = find_node_info(node_id);
	if(node_info == NULL){
		return NULL;
	}
	for ( i = 0 ; i < node_info->msg_num ; i ++ ){
		if(node_info->msg_info[i].p_cont_block_info->type == type){
			return &node_info->msg_info[i]; //O(n^2)
		}
	}
	return NULL;
}
#if APROS_TIMEOUT_ENABLE 
/*---------------------------------------------------------------------------*/
void set_timeout(msg_info_t* msg_info, clock_time_t time, void (*func)(void*)){
	if(msg_info == NULL){
		return;
	}
	
	uint8_t index=0;
	static error_cb_st_t er_timeout[MAX_COMM_INBUF];
	
	while(er_timeout[index].msg_info != NULL && !ctimer_expired(&er_timeout[index].msg_info->timeout)){
		index++;
		if(index == MAX_COMM_INBUF){
			return;
		}
	}

	er_timeout[index].error_type = RESPONSE_TIMEOUT;
	er_timeout[index].msg_info = msg_info;
	
	ctimer_set(&msg_info->timeout, time, func, &er_timeout[index]);
}
/*---------------------------------------------------------------------------*/
void release_timeout(msg_info_t* msg_info){
	if(msg_info == NULL){
		return;
	}

	ctimer_stop(&msg_info->timeout);
}
/*---------------------------------------------------------------------------*/
void restart_timeout(msg_info_t* msg_info){
	if(msg_info == NULL){
		return;
	}

	ctimer_restart(&msg_info->timeout);
}
/*---------------------------------------------------------------------------*/
uint8_t is_processing_msg(msg_info_t* msg_info){
	if(msg_info == NULL){
		return -1;
	}
	return msg_info->is_processing;
}
/*---------------------------------------------------------------------------*/
void set_processing_msg(msg_info_t* msg_info){
	if(msg_info == NULL){
		return;
	}
	msg_info->is_processing = 1;
}
/*---------------------------------------------------------------------------*/
void clear_processing_msg(msg_info_t* msg_info){
	if(msg_info == NULL){
		return;
	}
	msg_info->is_processing = 0;
}
#endif
#endif
/*---------------------------------------------------------------------------*/
/*#if APROS_LED_EXIST
static void apros_led_set(void* ptr){
	uint8_t value = *((uint8_t*)ptr);
	if(value == 0){
		leds_off(LEDS_CONF_ALL);
	}else if(value > 0 && value <= LEDS_CONF_ALL){
		leds_on(value);
	}
}
#endif*/
/*---------------------------------------------------------------------------*/
void apros_command(apros_comm_param_t comm){
	uint16_t index = 0;
		
	if(comm.func == NULL){
		return;
	}
	
/*#if APROS_LED_EXIST
	while(!ctimer_expired(&ct_comm_buffer[index])||
		!ctimer_expired(&ct_comm_led_buffer[index])){
		index++;
		if(index == MAX_COMM_INBUF){
			return;
		}
	}

	if(comm.led >= 0 && comm.led <= LEDS_CONF_ALL){
		led_cp[index] = comm.led;
		ctimer_set(&ct_comm_led_buffer[index], comm.t_backoff, apros_led_set, (void*)&led_cp[index]);
	}
#else*/
	while(!ctimer_expired(&ct_comm_buffer[index])){
		index++;
		if(index == MAX_COMM_INBUF){
			return;
		}
	}
//#endif

	if(comm.func_param == NULL){
		ctimer_set(&ct_comm_buffer[index], comm.t_backoff, comm.func, NULL); 
	}else{
		memcpy(&param_cp[index], (uint8_t*)comm.func_param, MAX_PARAM_SIZE);
		ctimer_set(&ct_comm_buffer[index], comm.t_backoff, comm.func, (void*)&param_cp[index]); 
	}
}
/*---------------------------------------------------------------------------*/

