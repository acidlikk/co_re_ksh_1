/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
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
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc13xx-platforms
 * @{
 *
 * \defgroup cc13xx-examples CC13xx Example Projects
 *
 * Example projects for CC13xx-based platforms.
 * @{
 *
 * \defgroup cc13xx-demo CC13xx Demo Project
 *
 *   Example project demonstrating the CC13xx platforms
 *
 *   This example will work for the following boards:
 *   - The CC1310 Apros AP
 *
 *   By default, the example will build for the srf06-cc13xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=srf06/cc13xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This example also demonstrates CC13xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc13xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc13xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "sys/stimer.h"
#include "dev/leds.h"
#include "dev/watchdog.h"
#include "random.h"
#include "board-peripherals.h"
#include "net/rime/rime.h"

#include "ieee-addr.h"
#include "dev/uart1.h"
#include "apros-serial-line.h"
#include "apros-gpio-control.h"
#include "apros-packet-control.h"
#include "apros-wireless-line.h"
#include "apros-flash-control.c"
#include "apros-command-scheduler.c"
#include "net/netstack.h"

#include "ti-lib.h"

#include "cc13xx-base.h"

#include <stdio.h>
#include <stdint.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
#define NAME 0xCC131005

#define DEFAULT_BASE_CH 5

/** Channel **/
#define AVAILABLE_CHANNEL 0x00FD2490
// 5, 8, 11, 14, 17, 19, 20, 21, 22, 23, 24
#define DEFAULT_CCA_THRESHOLD -100

#define MAX_CCA_THRESHOLD -60
#define MIN_CCA_THRESHOLD -120

static struct etimer et;
static struct stimer st;
static struct ctimer ct;
#if APROS_BASE_AUTO_REBOOT
static uint8_t flag_cold = 1;
#endif

uint8_t flag_processing_reboot;
uint8_t flag_processing_flash;

static uint8_t base_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static apros_base_config_t base_config;
/*---------------------------------------------------------------------------*/
PROCESS(cc1310_process, "cc1310_process");
PROCESS(uart_rx_process, "uart rx process");
AUTOSTART_PROCESSES(&cc1310_process, &uart_rx_process);
/*---------------------------------------------------------------------------*/
/*//TODO::
void base_to_busy(){
	ti_lib_gpio_set_dio(BOARD_IOID_UART_CBUS);//High : Indicates Busy State on the base
}*/
/*---------------------------------------------------------------------------*/
/*//TODO::
void base_to_idle(){
	ti_lib_gpio_clear_dio(BOARD_IOID_UART_CBUS);//LOW : Indicates IDLE State on the base
}*/
/*---------------------------------------------------------------------------*/
void apply_channel(uint8_t channel){
	NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, channel);
	PRINTF(" Apply ch : %d\n", channel);
}
/*---------------------------------------------------------------------------*/
void apply_cca(uint8_t cca_value){
	if((int8_t)cca_value < MIN_CCA_THRESHOLD || (int8_t)cca_value > MAX_CCA_THRESHOLD){
		NETSTACK_RADIO.set_value(RADIO_PARAM_CCA_THRESHOLD, DEFAULT_CCA_THRESHOLD);
		PRINTF(" cca threshold = %d\n", DEFAULT_CCA_THRESHOLD);
	}else{
		NETSTACK_RADIO.set_value(RADIO_PARAM_CCA_THRESHOLD, cca_value);
		PRINTF(" cca threshold = %d\n", cca_value);
	}
}	
/*---------------------------------------------------------------------------*/
static uint8_t verity_ch(uint8_t ch){
	uint32_t val = 1;
	return (((val << (ch-1)) & AVAILABLE_CHANNEL) != 0);
}
/*---------------------------------------------------------------------------*/
static void cc1310_config_init(void){
	base_config.dev_name = NAME;
	base_config.channel = DEFAULT_BASE_CH;
	base_config.cca_th = DEFAULT_CCA_THRESHOLD;
}
/*---------------------------------------------------------------------------*/
static void cc1310_init(void)
{
	ieee_addr_cpy_to(base_mac, 8);

	apros_base_config_load(&base_config);
	add_all_block_info();
	
	if(base_config.dev_name != NAME){
		cc1310_config_init();
		apros_base_config_save(&base_config);
	}

	apply_channel(base_config.channel);
	apply_cca(base_config.cca_th);
}
/*---------------------------------------------------------------------------*/
void
to_busy(){
//	ti_lib_gpio_set_dio(BOARD_IOID_UART_CBUS);//High : Indicates Busy State on the base
	leds_on(LEDS_YELLOW);

}
/*---------------------------------------------------------------------------*/
void 
to_idle(){
	leds_off(LEDS_YELLOW);
//	ti_lib_gpio_clear_dio(BOARD_IOID_UART_CBUS);//LOW : Indicates IDLE State on the base

}
/*---------------------------------------------------------------------------*/
//This Function is only for SHORT Address
static uint8_t
generate_pkt_wireless(linkaddr_t* dest_short_addr, uint8_t m_type, struct tlv_set* tlv_buf, uint8_t tlv_num){
	static uint8_t pkt[APROS_PKT_MAX_SIZE];
 	uint8_t tlv_index;
	uint8_t ret = 0;

	if(apros_init_pkt(pkt, APROS_PKT_MAX_SIZE, m_type)){
		for(tlv_index = 0 ; tlv_index < tlv_num ; tlv_index++){
			if(apros_insert_tlv(pkt, tlv_buf+tlv_index)){

			}else{
				return ret;
			}
		}
		if(apros_complete_pkt(pkt)){
			if(frame802154_is_broadcast_addr(FRAME802154_SHORTADDRMODE, (uint8_t*)dest_short_addr)){
				PRINTF("SEND BROADCAST\n");

				apros_send_broadcast(pkt, get_pkt_length(pkt));
				ret = 1;
			}else{
				PRINTF("SEND UNICAST to %x.%x\n", dest_short_addr->u8[0], dest_short_addr->u8[1]);
				apros_send_unicast(dest_short_addr, pkt, get_pkt_length(pkt));
				ret = 1;
			}
		}
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
static uint8_t
generate_pkt_serial(uint8_t m_type, struct tlv_set* tlv_buf, uint16_t tlv_num){
	static uint8_t pkt[APROS_PKT_MAX_SIZE];
 	uint8_t tlv_index;
	uint8_t ret = 0;

	if(apros_init_pkt(pkt, APROS_PKT_MAX_SIZE, m_type)){
		
		for(tlv_index = 0 ; tlv_index < tlv_num ; tlv_index++){
			if(apros_insert_tlv(pkt, tlv_buf+tlv_index)){
					
			}else{
				return ret;
			}
		}
		if(apros_complete_pkt(pkt)){
			apros_serial_send(pkt, get_pkt_length(pkt));
		}
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
static uint8_t verify_addr(uint8_t* pl){
	struct tlv_set id_tlv;
	
	id_tlv.type = pl[0];
	id_tlv.length = pl[1];
	id_tlv.value = pl+2;
	
	if(id_tlv.type == BLK_TYPE_ID && id_tlv.length == 8){
		if(!memcmp(base_mac, id_tlv.value, 8)){ 
			return MY_ADDR;
		}else if(frame802154_is_broadcast_addr(FRAME802154_LONGADDRMODE, id_tlv.value)){
			return BR_ADDR;
		}else{
			return OTHER_ADDR;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static void subfunc_response_serial(void* ptr){
	tlv_buf_hdr_t* tlv_buf_hdr = (tlv_buf_hdr_t*)ptr;
	generate_pkt_serial(MSG_TYPE_CONTROL, tlv_buf_hdr->tlv_p, tlv_buf_hdr->tlv_num);
}
/*---------------------------------------------------------------------------*/
#if !APROS_BASE_BYPASS_FROM_WIRELESS
static void subfunc_error_rsp_serial(void* ptr){
	error_cb_st_t* er = (error_cb_st_t*)ptr;
	tlv_buf_hdr_t tlv_hdr;
	tlv_set_t tlv_set[2];
	static uint8_t rsp[20];

	uint16_t i = 0;
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = er->msg_info->p_node_info->node_id;
	
	tlv_set[i].type = er->msg_info->p_cont_block_info->type;
	tlv_set[i].length = er->msg_info->p_cont_block_info->rsp_length;
/*#if APROS_OLD_DF
	if(tlv_set[i].type == BLK_TYPE_DATA_FETCH|| tlv_set[i].type == BLK_TYPE_RETT)
#else
	if(tlv_set[i].type == BLK_TYPE_DATA_FETCH_READY || tlv_set[i].type == BLK_TYPE_RETT)
#endif
	{
		static uint8_t rsp[6];
		tlv_set[i].length = 6;
		rsp[0] = er->error_type;
		rsp[1] = 0xff;
		rsp[2] = 0xff;
		rsp[3] = 0xff;
		rsp[4] = 0xff;
		rsp[5] = 0xff;
		tlv_set[i++].value = rsp;
	}
#if !APROS_OLD_DF
	else if(tlv_set[i].type == BLK_TYPE_NODE_INFO){
		static uint8_t rsp[20];
		
		rsp[0] = er->error_type;
		for(uint8_t j=1; j < tlv_set[i].length; i++){
			rsp[j] = 0xff;
		}
		
		tlv_set[i++].value = rsp;
	}
#endif
	else{
		tlv_set[i].length = 1;
		tlv_set[i++].value = &er->error_type;
	}
*/

	rsp[0] = er->error_type;
	for(uint8_t j=1 ;j < tlv_set[i].length ; j++){
		rsp[j] = 0xff;
	}
	tlv_set[i++].value = rsp;

	
	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
#if APROS_TIMEOUT_ENABLE
	if(er->error_type == RESPONSE_TIMEOUT){
		clear_processing_msg(er->msg_info);
	}
#endif
	generate_pkt_serial(MSG_TYPE_CONTROL, tlv_hdr.tlv_p, tlv_hdr.tlv_num);
}
#endif
/*---------------------------------------------------------------------------*/
static void subfunc_apply_ch(void* ptr){
	apply_channel(base_config.channel);
}
/*---------------------------------------------------------------------------*/
static void dev_reset(void* ptr){
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
static void subfunc_base_reset(void* ptr){
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	
	PRINTF("CALL base_reset\n");
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_BASE_RESET;
	tlv_set[i].length = BLK_RSP_LEN_BASE_RESET;
	rsp = RESPONSE_SUCCESS;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;

	generate_pkt_serial(MSG_TYPE_CONTROL, tlv_hdr.tlv_p, tlv_hdr.tlv_num);

	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_cca(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t base_cca;

	if(flag_processing_flash){
		return;
	}

	PRINTF("START set_ourput_cca\n");	
	flag_processing_flash = 1;
	
	base_cca = base_config.cca_th;
	base_config.cca_th = *((uint8_t*)ptr);
		
	if((int8_t)base_config.cca_th < MIN_CCA_THRESHOLD || (int8_t)base_config.cca_th > MAX_CCA_THRESHOLD){
		rsp = RESPONSE_INVALID_ARGU;
		base_config.cca_th = base_cca;
	}else{
	    if(apros_base_config_save(&base_config)){
			rsp = RESPONSE_SUCCESS;
			apply_cca(base_config.cca_th);
		}else{
			rsp = RESPONSE_FLASH_ERROR;
			base_config.cca_th = base_cca;
			apros_base_config_save(&base_config);
		}
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_BASECCA;
	tlv_set[i].length = BLK_RSP_LEN_SET_BASECCA;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	
	flag_processing_flash = 0;
	
	PRINTF("END set_cca\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_cca(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t cca_val;

	uint16_t i = 0;
	
	PRINTF("START get_cca\n");

	cca_val = base_config.cca_th;
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_BASECCA;
	tlv_set[i].length = BLK_RSP_LEN_GET_BASECCA;
	tlv_set[i++].value = &cca_val;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_cca\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_basech(void* ptr){
	static apros_comm_param_t comm[2];
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t base_ch_backup;

	if(flag_processing_flash){
		return;
	}
	
	PRINTF("START set_base_ch\n");	
	flag_processing_flash = 1;
	
	base_ch_backup = base_config.channel;
	base_config.channel = *((uint8_t*)ptr);

	if(verity_ch(base_config.channel)){
		if(apros_base_config_save(&base_config)){
			rsp = RESPONSE_SUCCESS;
			PRINTF("BASE_CH = %d\n", base_config.channel);
		}else{
			rsp = RESPONSE_FLASH_ERROR;
			base_config.channel = base_ch_backup;
			apros_base_config_save(&base_config);
			PRINTF("FAIL\n");
		}
	}else{
		rsp = RESPONSE_NOT_SURPT_CH;
		base_config.channel = base_ch_backup;
		PRINTF("FAIL\n")
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_BASECH;
	tlv_set[i].length = BLK_RSP_LEN_SET_BASECH;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm[0].func = subfunc_response_serial;
	comm[0].func_param = &tlv_hdr;
	comm[0].t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm[0]);
	
	if(rsp == RESPONSE_SUCCESS){
		comm[1].func = subfunc_apply_ch;
		comm[1].func_param = NULL;
		comm[1].t_backoff = APROS_COMM_PRIOR_2;
		
		apros_command(comm[1]);
	}
	flag_processing_flash = 0;
	PRINTF("END set_base_ch\n");
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_basech(void* ptr){
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t base_ch;

	uint16_t i = 0;
	
	PRINTF("START get_base_ch\n");

	if(base_config.channel < 25 &&
		base_config.channel > 0){
		base_ch = base_config.channel;	
	}else{
		base_ch = RESPONSE_INVALID_ARGU;
	}
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_BASECH;
	tlv_set[i].length = BLK_RSP_LEN_GET_BASECH;
	tlv_set[i++].value = &base_ch;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_base_ch\n");
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_baseid(void* ptr){
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	
	PRINTF("START get_base_id\n");
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = base_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_BASEID;
	tlv_set[i].length = BLK_RSP_LEN_GET_BASEID;
	rsp = RESPONSE_SUCCESS;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_base_id\n");
}
/*---------------------------------------------------------------------------*/
#if !APROS_BASE_BYPASS_FROM_WIRELESS
static uint8_t verify_wireless_payload(uint8_t* pl, uint8_t pl_len, uint8_t m_type){
	uint8_t addr_type;
	uint8_t pl_index;
	static struct tlv_set tlv;
	//static struct apros_comm_param comm;

	tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[6];
	uint16_t i = 0;
	static uint8_t src_node_id[8];
	static node_info_t* p_n_info=NULL;
	static msg_info_t* p_m_info=NULL;
	static msg_info_t* p_m_info_fetch_start=NULL;
	static uint8_t prev_type=0;
	addr_type = verify_addr(pl); 
	if(addr_type == 0){
		return 0;
	}
	
	pl_index = BLK_LEN_ID+2; //next to ID TLV

	tlv_set[i].type = pl[0];
	tlv_set[i].length = pl[1];
	tlv_set[i++].value = pl+2;
	
	if(memcmp(src_node_id, pl+2, 8)){
		memcpy(src_node_id, pl+2, 8);
		if((p_n_info=find_node_info(src_node_id)) == NULL){
			p_n_info=add_node_info(src_node_id); //add new node
		}
	}
	
	while(pl_len > pl_index){
		//TLV
		tlv.type = pl[pl_index];
		tlv.length = pl[pl_index+1];
		if(tlv.length){
			tlv.value = pl+pl_index+2;
		}else{
			tlv.value = NULL;
		}
		if(addr_type == OTHER_ADDR){
			
			if(tlv.type == BLK_TYPE_ACCEL_DATA){
				//not control message
			}else{
				if(prev_type == tlv.type){

				}else{
					if((p_m_info=find_msg_info(src_node_id, tlv.type)) == NULL){
						p_m_info=add_msg_info(p_n_info, tlv.type); //add new comm
						prev_type=tlv.type;
					}
					if(tlv.type == BLK_TYPE_DATA_FETCH_START){
						p_m_info_fetch_start = p_m_info;
					}
				}
			}
			
			switch(tlv.type){
				default:
#if APROS_TIMEOUT_ENABLE			
					if(tlv.type == BLK_TYPE_ACCEL_DATA){
						if(p_m_info_fetch_start == NULL){
							p_m_info_fetch_start=find_msg_info(src_node_id, BLK_TYPE_DATA_FETCH_START);
						}
						restart_timeout(p_m_info_fetch_start);
					}else{
						release_timeout(p_m_info);
						clear_processing_msg(p_m_info);
					}
				
#endif					
					tlv_set[i].type = tlv.type;
					tlv_set[i].length = tlv.length;
					tlv_set[i++].value = tlv.value;	
			}
		}				
		pl_index += tlv.length + 2; //length(1) + type(1) + value
	}
	if( i > 1 ){
		tlv_hdr.tlv_p = tlv_set;
		tlv_hdr.tlv_num = i;
		generate_pkt_serial(m_type, tlv_hdr.tlv_p, tlv_hdr.tlv_num); 	
	}
	return 0;
}
#endif
/*---------------------------------------------------------------------------*/
static uint8_t verify_serial_payload(uint8_t* pl, uint8_t pl_len){
	uint8_t addr_type;
	uint8_t pl_index;
	struct tlv_set tlv;
	struct apros_comm_param comm;

	tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[6];
	uint16_t i = 0;
	uint8_t dest_node_id[8];
	
	addr_type = verify_addr(pl); 
	
	if(addr_type == 0){
		return 0;
	}
	
	if(pl[0] != BLK_TYPE_ID){
		return -1;
	}
	
	pl_index = BLK_LEN_ID+2; 

	tlv_set[i].type = pl[0];
	tlv_set[i].length = pl[1];
	tlv_set[i++].value = pl+2;
	memcpy(dest_node_id, pl+2, 8);

	while(pl_len > pl_index){
		node_info_t* p_n_info=NULL;
		msg_info_t* p_m_info=NULL;
		basic_block_info_t* p_b_info=NULL;
		cont_block_info_t* p_c_info=NULL;
		//TLV
		tlv.type = pl[pl_index];
		tlv.length = pl[pl_index+1];
		if(tlv.length){
			tlv.value = pl+pl_index+2;
		}else{
			tlv.value = NULL;
		}

		if((p_b_info = find_basic_block_info(tlv.type)) != NULL){
			if(p_b_info->length != tlv.length){
				return -1;
			}
		}else if((p_c_info = find_cont_block_info(tlv.type)) != NULL){
			if(p_c_info->req_length != tlv.length){
				return -1;
			}
		}else{
			return -1;
		}

		if(flag_processing_reboot){
			return -1;
		}
		
		if(addr_type == OTHER_ADDR){
			if((p_n_info=find_node_info(dest_node_id)) == NULL){
				p_n_info=add_node_info(dest_node_id); //add new node
			}
			if((p_m_info=find_msg_info(dest_node_id, tlv.type)) == NULL){
				p_m_info=add_msg_info(p_n_info, tlv.type); //add new msg
			}
#if APROS_TIMEOUT_ENABLE			
			if(is_processing_msg(p_m_info) == 1){//ing
				static error_cb_st_t er_exist;
				er_exist.error_type = RESPONSE_ALREADY_EXIST;
				er_exist.msg_info = p_m_info;

				subfunc_error_rsp_serial((void*)&er_exist);
				
				pl_index += tlv.length + 2; //length(1) + type(1) + value
				continue;
			}
#endif

		}				
		switch(tlv.type){
			case BLK_TYPE_BASE_RESET:
				if(addr_type == MY_ADDR){
					flag_processing_reboot = 1;
					comm.func = subfunc_base_reset;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_6;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_BASECCA:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_set_cca;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_4;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_BASECCA:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_cca;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_5;

					apros_command(comm);
				}	
				break;
			case BLK_TYPE_SET_BASECH:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_set_basech;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_4;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_BASECH:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_basech;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_5;

					apros_command(comm);
				}	
				break;
			case BLK_TYPE_GET_BASEID:
				if(addr_type == BR_ADDR){
					comm.func = subfunc_get_baseid;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_5;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_CAPT_REQUEST:
			case BLK_TYPE_AUTO_OFFSET:
				{
#if APROS_TIMEOUT_ENABLE
					set_processing_msg(p_m_info);
					set_timeout(p_m_info, CLOCK_SECOND*6, subfunc_error_rsp_serial);
#endif
					tlv_set[i].type = tlv.type;
					tlv_set[i].length = tlv.length;
					tlv_set[i++].value = tlv.value;
					break;
				}
				//for defrecated data fetch method
#if APROS_OLD_DF
			case BLK_TYPE_DATA_FETCH:
#else
			case BLK_TYPE_DATA_FETCH_START:
#endif

				{
#if APROS_TIMEOUT_ENABLE
					set_processing_msg(p_m_info);
					set_timeout(p_m_info, CLOCK_SECOND*7, subfunc_error_rsp_serial);
#endif					
					tlv_set[i].type = tlv.type;
					tlv_set[i].length = tlv.length;
					tlv_set[i++].value = tlv.value;
					break;
				}
			default:
				{
#if APROS_TIMEOUT_ENABLE
					set_processing_msg(p_m_info);
					set_timeout(p_m_info, CLOCK_SECOND*4, subfunc_error_rsp_serial);
#endif					
					tlv_set[i].type = tlv.type;
					tlv_set[i].length = tlv.length;
					tlv_set[i++].value = tlv.value;
				}
		}
		pl_index += tlv.length + 2; //length(1) + type(1) + value
	}

	if( i > 1 ){
		tlv_hdr.tlv_p = tlv_set;
		tlv_hdr.tlv_num = i;

		generate_pkt_wireless((linkaddr_t*)&pl[8], MSG_TYPE_CONTROL, tlv_hdr.tlv_p, tlv_hdr.tlv_num); 

	}
	return 1;
}
/*---------------------------------------------------------------------------*/
static void packet_recv(uint8_t* pbuf, uint16_t plen){
#if APROS_BASE_AUTO_REBOOT
  	flag_cold = 0;
#endif
/*
	if(pbuf[17] == 0xA0 && pbuf[18] == 0x30){
		for(uint8_t i = 19; i < 19+48; i+=3){
			printf("%02x%02x%02x\n", pbuf[i],pbuf[i+1], pbuf[i+2] );
		}
 	}else{
 */
#if APROS_LED_EXIST
	leds_toggle(LEDS_YELLOW);
#endif
#if APROS_BASE_BYPASS_FROM_WIRELESS
	apros_serial_send(pbuf, plen);
#else
	static uint8_t pl[APROS_PL_MAX_SIZE], pl_length;
	uint8_t m_type;
	if(apros_base_pkt_parser(pbuf, plen, pl, &pl_length, &m_type) == 1){
		verify_wireless_payload(pl, pl_length, m_type);
	}
#endif
 //	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc1310_process, ev, data)
{
  PROCESS_BEGIN();

  cc1310_init();

  etimer_set(&et, CLOCK_SECOND);
#if APROS_BASE_AUTO_REBOOT
  stimer_set(&st, APROS_BASE_AUTO_REBOOT_PERIOD);
#endif
  while(1) {
    PROCESS_YIELD();
#if APROS_BASE_AUTO_REBOOT
    if(stimer_expired(&st)){
		if(flag_cold){
			watchdog_reboot();
		}
		flag_cold = 1;
		stimer_set(&st, APROS_BASE_AUTO_REBOOT_PERIOD);
    }
#endif
    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        leds_toggle(LEDS_RED);
        
        etimer_set(&et, CLOCK_SECOND>>1);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_rx_process, ev, data)
{
  PROCESS_EXITHANDLER(apros_wireless_line_close();)
  PROCESS_BEGIN();

  uint8_t *pbuf;
  uint16_t plen;

  uart1_set_input(apros_serial_line_input_byte);
  apros_serial_line_init();
  apros_wireless_line_init(packet_recv, packet_recv);
  
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == apros_serial_line_event_message && data != NULL);
    pbuf = ((sl_data_t*)data)->buffer;
    plen = ((sl_data_t*)data)->length;

	static uint8_t pkt_buf[APROS_PKT_MAX_SIZE], pkt_len = 0;
	static uint8_t pl[APROS_PL_MAX_SIZE], pl_len;		
	uint8_t m_type;
	
	if(pkt_len + plen <= APROS_PKT_MAX_SIZE) {
      memcpy(pkt_buf + pkt_len, pbuf, plen);
      pkt_len += plen;
    }
    else {
      memcpy(pkt_buf, pbuf, plen);
      pkt_len = plen;
    }
	if(pkt_len >= APROS_PKT_MIN_SIZE){ 
		if(apros_base_pkt_parser(pkt_buf, pkt_len, pl, &pl_len, &m_type) == 1){
			if(m_type == MSG_TYPE_CONTROL){
				verify_serial_payload(pl, pl_len);
				pkt_len = 0;
				continue;
			}
		}
	}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
