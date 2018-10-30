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
 
#include "apros-wireless-line.h"
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/* Packet */
typedef struct wireless_subfunc{
	void (*recv_uc_subfunc)(uint8_t*, uint16_t);
	void (*recv_bc_subfunc)(uint8_t*, uint16_t);
}wireless_subfunc_t;

static void null_uc_recv(uint8_t* a, uint16_t b){

}
static void null_bc_recv(uint8_t* a, uint16_t b){

}

static struct wireless_subfunc subfunc_set = {
	null_uc_recv,
	null_bc_recv
};
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from){
	subfunc_set.recv_uc_subfunc((uint8_t*)packetbuf_dataptr(), packetbuf_datalen());
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx){
	const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
	
	if(linkaddr_cmp(dest, &linkaddr_null)) {
		PRINTF("SENT ERROR a Unicast Packet\n");
	    return;
	}
	PRINTF("SENT a Unicast Packet\n");
}
/*---------------------------------------------------------------------------*/
static void
recv_bc(struct broadcast_conn *c, const linkaddr_t *from){
  	subfunc_set.recv_bc_subfunc((uint8_t*)packetbuf_dataptr(), packetbuf_datalen());
}
/*---------------------------------------------------------------------------*/
static void
sent_bc(struct broadcast_conn *c, int status, int num_tx){
	PRINTF("SENT a Broadcast Packet\n");
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_callbacks = {recv_bc, sent_bc};
static struct broadcast_conn bc;
/*---------------------------------------------------------------------------*/
void 
apros_wireless_line_init(void (*recv_uc_sub)(uint8_t*, uint16_t), void (*recv_bc_sub)(uint8_t*, uint16_t)){
	unicast_open(&uc, 146, &unicast_callbacks);
	broadcast_open(&bc, 129, &broadcast_callbacks);
	if(recv_uc_sub == NULL){
		subfunc_set.recv_uc_subfunc = null_uc_recv;
	}else{	
		subfunc_set.recv_uc_subfunc = recv_uc_sub;
	}
	if(recv_bc_sub == NULL){
		subfunc_set.recv_bc_subfunc = null_bc_recv;
	}else{
		subfunc_set.recv_bc_subfunc = recv_bc_sub;
	}
}
/*---------------------------------------------------------------------------*/
void 
apros_wireless_line_close(){
	unicast_close(&uc);
	broadcast_close(&bc);
}
/*---------------------------------------------------------------------------*/
uint8_t 
apros_send_unicast(linkaddr_t* dest_addr, uint8_t* packet, uint8_t pkt_length){
	if(linkaddr_node_addr.u8[0] == dest_addr->u8[0] &&
	   linkaddr_node_addr.u8[1] == dest_addr->u8[1]) {
		;
	}else{
	 	packetbuf_copyfrom(packet, pkt_length);
	  	unicast_send(&uc, dest_addr);
	  	return 1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t 
apros_send_broadcast(uint8_t* packet, uint8_t pkt_length){
	packetbuf_copyfrom(packet, pkt_length);
	broadcast_send(&bc);
	return 1;
}
/*---------------------------------------------------------------------------*/

