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
#include "dev/watchdog.h"
#include "random.h"
#include "board-peripherals.h"
#include "net/rime/rime.h"

#include "ieee-addr.h"
#include "dev/uart1.h"
#include "net/netstack.h"

#include "ti-lib.h"

#include "cc13xx-node.h"

#include <stdio.h>
#include <stdint.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*---------------------------------------------------------------------------*/
/** DEV Identity **/
#define NAME 0xCC131011

#define DEFAULT_VENDOR_NAME "APROS Co."

#define DEFAULT_NODE_CH 11

#define DEFAULT_CCA_THRESHOLD -70

#define MAX_CCA_THRESHOLD -60
#define MIN_CCA_THRESHOLD -120

#define DEFAULT_PKT_PERIOD 1 //Second

#define MAX_PKT_PERIOD	10800
#define MIN_PKT_PERIOD  10

static apros_node_config_t node_config;

static uint8_t flag_registered;

static uint8_t flag_processing_msg;
static uint8_t flag_processing_reboot;
static uint8_t flag_processing_flash;

/** Channel **/
#define AVAILABLE_CHANNEL 0x00FD2490
// 5, 8, 11, 14, 17, 19, 20, 21, 22, 23, 24

/** flash **/
static apros_node_config_t node_config;

/** Primary Timer **/
static struct etimer et;
static struct stimer st;
//static struct rtimer rt;
static struct ctimer ct; 
/** Dedicated Timer **/


/** MAC Address **/
static uint8_t node_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/** Others. **/
static node_info_t my_node_info;
static msg_info_t my_msg_info;


static clock_time_t br_ack_backoff_time;
static int val_temp, val_humid, val_bat_read;
#if APROS_LIGHT_ENABLE
static int val_light;
#endif
#if APROS_PIR_ENABLE
static uint8_t val_pir;
#endif
/*---------------------------------------------------------------------------*/
/* Init  */
/*---------------------------------------------------------------------------*/
static void packet_recv(uint8_t* pbuf, uint16_t plen);

static inline clock_time_t calc_backoff(uint8_t mac_addr[]){
	uint16_t base_value = ((uint16_t)mac_addr[6] << 8) + mac_addr[7];

	return (clock_time_t)base_value%128;
}
/*---------------------------------------------------------------------------*/
void apply_channel(uint8_t channel){
	NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, channel);
	PRINTF(" apply ch : %d\n", channel);
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
static void cc1310_config_init(void){
	node_config.dev_name = NAME;
	memset(node_config.base_id, 0, 8);
	node_config.channel = DEFAULT_NODE_CH;
	node_config.cca_th = DEFAULT_CCA_THRESHOLD;
#if APROS_PERIODIC_PKT
	node_config.pkt_period = DEFAULT_PKT_PERIOD;
#endif	
	memset(node_config.__vendor, 0, VENDOR_MAX_LEN);
	memcpy(node_config.__vendor, DEFAULT_VENDOR_NAME, sizeof(DEFAULT_VENDOR_NAME));
}
/*---------------------------------------------------------------------------*/
static void init_setting(void)
{
	ieee_addr_cpy_to(node_mac, 8);
#if APROS_EXT_FLASH
	apros_node_config_load(&node_config);
	if(node_config.dev_name != NAME){
		cc1310_config_init();
		apros_node_config_save(&node_config);
	}
#else
	cc1310_config_init();
#endif 

	memcpy(my_node_info.node_id, node_mac, 8);
	my_node_info.msg_num = 0;//not used value;
	my_node_info.msg_info = &my_msg_info;
	my_msg_info.p_node_info = &my_node_info;
	my_msg_info.p_cont_block_info = NULL;//Null value

#if APROS_UNICAST
	if(linkaddr_cmp((linkaddr_t*)(node_config.base_id+6), &linkaddr_null)){
		flag_registered = 0;
	}else{
		flag_registered = 1;
	}
#else
	flag_registered = 1;
#endif
	add_all_block_info();
	  
	br_ack_backoff_time = calc_backoff(node_mac);
	apply_channel(node_config.channel);
	apply_cca(node_config.cca_th);
  	apros_serial_line_init();

	apros_wireless_line_init(packet_recv, packet_recv);
}
/*---------------------------------------------------------------------------*/
/* Communication  */
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
/*static uint8_t
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
}*/
/*---------------------------------------------------------------------------*/
static uint8_t verify_addr(uint8_t* pl){
	struct tlv_set id_tlv;
	
	id_tlv.type = pl[0];
	id_tlv.length = pl[1];
	id_tlv.value = pl+2;
	
	if(id_tlv.type == BLK_TYPE_ID && id_tlv.length == 8){
		if(!memcmp(node_mac, id_tlv.value, 8)){ 
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
#if APROS_PERIODIC_PKT
static void packet_send_periodic(void* ptr);
static void packet_send_periodic(void* ptr){
	static tlv_set_t tlv_set[7];
//	static uint16_t sequence_num = 0;
//	static uint8_t sq_num_8[2];
	static uint8_t temp_8[2], humid_8[2], battery_8[2];
#if APROS_LIGHT_ENABLE
	static uint8_t light_8[2];
#endif
#if APROS_PIR_ENABLE
	static uint8_t pir;
#endif
	
	uint8_t i=0;
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;

	tlv_set[i].type = BLK_TYPE_TEMP;
	tlv_set[i].length = BLK_LEN_TEMP;

	temp_8[0] = val_temp & 0xff;
	temp_8[1] = (val_temp >> 8) & 0xff;

	tlv_set[i++].value = temp_8;

	tlv_set[i].type = BLK_TYPE_HUMID;
	tlv_set[i].length = BLK_LEN_HUMID;
	
	humid_8[0] = val_humid & 0xff;
	humid_8[1] = (val_humid >> 8) & 0xff;
	
	tlv_set[i++].value = humid_8;
#if APROS_LIGHT_ENABLE
	tlv_set[i].type = BLK_TYPE_LIGHT;
	tlv_set[i].length = BLK_LEN_LIGHT;
	
	light_8[0] = val_light & 0xff;
	light_8[1] = (val_light >> 8) & 0xff;
	
	tlv_set[i++].value = light_8;
#endif
#if APROS_PIR_ENABLE
	tlv_set[i].type = BLK_TYPE_MOTION;
	tlv_set[i].length = BLK_LEN_MOTION;

	//read gpio
	/*val_pir = ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI);
	val_pir <<= 1;
	val_pir |= ti_lib_gpio_read_dio(BOARD_IOID_PIR_LO);*/
	pir = val_pir;
	val_pir = 0;
	tlv_set[i++].value = &pir;
#endif
	tlv_set[i].type = BLK_TYPE_BATTERY;
	tlv_set[i].length = BLK_LEN_BATTERY;
	
	ti_lib_gpio_set_dio(BOARD_IOID_BAT_ADC_ENABLE);
	val_bat_read = read_bat_voltage();
	ti_lib_gpio_clear_dio(BOARD_IOID_BAT_ADC_ENABLE);

	battery_8[0] = val_bat_read & 0xff;
	battery_8[1] = (val_bat_read >> 8) & 0xff;

	tlv_set[i++].value = battery_8;
	
/*
	tlv_set[i].type = BLK_TYPE_SEQNUM;
	tlv_set[i].length = BLK_LEN_SEQNUM;

	sq_num_8[0] = sequence_num & 0xff;
	sq_num_8[1] = (sequence_num >> 8) & 0xff;
	
	tlv_set[i++].value = sq_num_8;
	sequence_num++;
*/
#if APROS_UNICAST
	generate_pkt_wireless((linkaddr_t*)(node_config.base_id+6), MSG_TYPE_REPORT, tlv_set, i);
#else
	static uint8_t br_addr[2] = {0xff, 0xff};
	generate_pkt_wireless((linkaddr_t*)br_addr, MSG_TYPE_REPORT, tlv_set, i);
#endif
#if APROS_LED_EXIST
	leds_toggle(LEDS_ORANGE);
#endif
}
#endif
/*---------------------------------------------------------------------------*/
/* Sub-Function for Command */
/*---------------------------------------------------------------------------*/
static void dev_reset(void* ptr){
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
static uint8_t verity_ch(uint8_t ch){
	uint32_t val = 1;
	return (((val << (ch-1)) & AVAILABLE_CHANNEL) != 0);
}
/*---------------------------------------------------------------------------*/
static void subfunc_response_wireless(void* ptr){
	tlv_buf_hdr_t* tlv_buf_hdr = (tlv_buf_hdr_t*)ptr;

	generate_pkt_wireless((linkaddr_t*)(node_config.base_id+6), MSG_TYPE_CONTROL, 
		tlv_buf_hdr->tlv_p, tlv_buf_hdr->tlv_num);
}
/*---------------------------------------------------------------------------*/
static void subfunc_error_rsp_wireless(void* ptr){
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

	tlv_set[i++].value = rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;

	generate_pkt_wireless((linkaddr_t*)(node_config.base_id+6), MSG_TYPE_CONTROL,
		tlv_hdr.tlv_p, tlv_hdr.tlv_num);
}
/*---------------------------------------------------------------------------*/
static void subfunc_apply_ch(void* ptr){
	apply_channel(node_config.channel);
}
/*---------------------------------------------------------------------------*/
static void subfunc_dev_reset(void* ptr){
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	
	PRINTF("CALL dev_reset\n");
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_DEV_RESET;
	tlv_set[i].length = BLK_RSP_LEN_DEV_RESET;
	rsp = RESPONSE_SUCCESS;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;

	generate_pkt_wireless((linkaddr_t*)(node_config.base_id+6), MSG_TYPE_CONTROL, 
		tlv_hdr.tlv_p, tlv_hdr.tlv_num);
	
	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);
}
/*---------------------------------------------------------------------------*/
static void subfunc_report_period(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint16_t node_pkt_period;
	uint8_t* p_input_period;
	
	PRINTF("START report_period\n");	

	p_input_period = (uint8_t*)ptr;
	node_pkt_period = node_config.pkt_period;
	node_config.pkt_period = p_input_period[0] | (p_input_period[1] << 8);
	
	if(node_config.pkt_period < MIN_PKT_PERIOD|| node_config.pkt_period > MAX_PKT_PERIOD){
		rsp = RESPONSE_INVALID_ARGU;
		node_config.pkt_period = node_pkt_period;
	}else{
	    if(apros_node_config_save(&node_config)){
			rsp = RESPONSE_SUCCESS;
		}else{
			rsp = RESPONSE_FLASH_ERROR;
			node_config.pkt_period = node_pkt_period;
			apros_node_config_save(&node_config);
		}
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_REPORT_PERIOD;
	tlv_set[i].length = BLK_RSP_LEN_REPORT_PERIOD;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);

	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);
	PRINTF("END report_period\n");
}
/*---------------------------------------------------------------------------*/
static void subfunc_prompt_report(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	
	PRINTF("CALL prompt_report\n");

	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_PROMPT_REPORT;
	tlv_set[i].length = BLK_RSP_LEN_PROMPT_REPORT;

	if(stimer_expired(&st)){
		rsp = RESPONSE_SUCCESS;	
		ctimer_set(&ct, CLOCK_SECOND>>3, packet_send_periodic, NULL);
	}else{
		rsp = RESPONSE_NODE_BUSY;
	}
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;

	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
}
/*---------------------------------------------------------------------------*/
static void subfunc_factory_reset(void* ptr){
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	
	PRINTF("CALL factory_reset\n");

	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_FTRY_RESET;
	tlv_set[i].length = BLK_RSP_LEN_FTRY_RESET;
	rsp = RESPONSE_SUCCESS;
	
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	generate_pkt_wireless((linkaddr_t*)(node_config.base_id+6), MSG_TYPE_CONTROL, 
		tlv_hdr.tlv_p, tlv_hdr.tlv_num);

	cc1310_config_init();
	
	apros_node_config_save(&node_config);
	
	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);	
}
/*---------------------------------------------------------------------------*/
static void subfunc_register_node(void* ptr){	
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t base_id_backup[8];
	
	PRINTF("START register\n");
	
	memcpy(base_id_backup, node_config.base_id, 8);
	memcpy(node_config.base_id, (uint8_t*)ptr, 8);
	
	if(apros_node_config_save(&node_config)){
		rsp = RESPONSE_SUCCESS;

		PRINTF("BASE ID = ");
		for(int j = 0 ; j < 8 ; j ++){
			PRINTF("%02X ", node_config.base_id[j]);
		}
		PRINTF("\n");
		
		flag_registered = 1;
	}else{
		rsp = RESPONSE_FLASH_ERROR;
		memcpy(node_config.base_id, base_id_backup, 8);
		apros_node_config_save(&node_config);
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_REGISTER_NODE;
	tlv_set[i].length = BLK_RSP_LEN_REGISTER_NODE;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	
	apros_command(comm);
	flag_processing_flash = 0;
	PRINTF("END register\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_cca(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t node_cca;
	
	PRINTF("START set_cca\n");	
	
	node_cca = node_config.cca_th;
	node_config.cca_th = *((uint8_t*)ptr);
	
	if((int8_t)node_config.cca_th < MIN_CCA_THRESHOLD || (int8_t)node_config.cca_th > MAX_CCA_THRESHOLD){
		rsp = RESPONSE_INVALID_ARGU;
		node_config.cca_th = node_cca;
	}else{
	    if(apros_node_config_save(&node_config)){
			rsp = RESPONSE_SUCCESS;
			apply_cca(node_config.cca_th);
		}else{
			rsp = RESPONSE_FLASH_ERROR;
			node_config.cca_th = node_cca;
			apros_node_config_save(&node_config);
		}
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_NODECCA;
	tlv_set[i].length = BLK_RSP_LEN_SET_NODECCA;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	
	flag_processing_flash=0;
	
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

	cca_val = node_config.cca_th;
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_NODECCA;
	tlv_set[i].length = BLK_RSP_LEN_GET_NODECCA;
	tlv_set[i++].value = &cca_val;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_cca\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_nodech(void* ptr){
	apros_comm_param_t comm[2];
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t node_ch_backup;
	
	PRINTF("START set_node_ch\n");	
	
	node_ch_backup = node_config.channel;
	node_config.channel = *((uint8_t*)ptr);
	
	/*if(apros_node_config_save(&node_config) && 
		verity_ch(node_config.channel)){
		rsp = RESPONSE_SUCCESS;
		PRINTF("NODE_CH = %d\n", node_config.channel);
	}else{
		rsp = RESPONSE_FLASH_ERROR;
		node_config.channel = node_ch_backup;
		apros_node_config_save(&node_config);
		PRINTF("FAIL\n");
	}*/

	if(verity_ch(node_config.channel)){
		if(apros_node_config_save(&node_config)){
			rsp = RESPONSE_SUCCESS;
			PRINTF("NODE_CH = %d\n", node_config.channel);
		}else{
			rsp = RESPONSE_FLASH_ERROR;
			node_config.channel = node_ch_backup;
			apros_node_config_save(&node_config);
			PRINTF("FAIL\n");
		}
	}else{
		rsp = RESPONSE_NOT_SURPT_CH;
		node_config.channel = node_ch_backup;
		PRINTF("FAIL\n")
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_NODECH;
	tlv_set[i].length = BLK_RSP_LEN_SET_NODECH;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm[0].func = subfunc_response_wireless;
	comm[0].func_param = &tlv_hdr;
	comm[0].t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm[0]);
	
	if(rsp == RESPONSE_SUCCESS){
		comm[1].func = subfunc_apply_ch;
		comm[1].func_param = NULL;
		comm[1].t_backoff = APROS_COMM_PRIOR_7;
		
		apros_command(comm[1]);
	}
	flag_processing_flash=0;
	
	PRINTF("END set_node_ch\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_nodech(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t node_ch;

	uint16_t i = 0;
	
	PRINTF("START get_node_ch\n");

	if(node_config.channel < 25 &&
		node_config.channel > 0){
		node_ch = node_config.channel;	
	}else{
		node_ch = RESPONSE_INVALID_ARGU;
	}
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_NODECH;
	tlv_set[i].length = BLK_RSP_LEN_GET_NODECH;
	tlv_set[i++].value = &node_ch;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_node_ch\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_vendor(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t vendor_backup[VENDOR_MAX_LEN];
	
	PRINTF("START set_vendor\n");	

	memcpy(vendor_backup, node_config.__vendor, VENDOR_MAX_LEN);
	memcpy(node_config.__vendor, (uint8_t*)ptr, VENDOR_MAX_LEN);
	
	if(apros_node_config_save(&node_config)){
		rsp = RESPONSE_SUCCESS;
		PRINTF("VENDOR = %s\n", node_config.__vendor);
	}else{
		rsp = RESPONSE_FLASH_ERROR;
		memcpy(node_config.__vendor, vendor_backup, VENDOR_MAX_LEN);

		apros_node_config_save(&node_config);
		PRINTF("FAIL\n");
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_VENDOR;
	tlv_set[i].length = BLK_RSP_LEN_SET_VENDOR;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	
	apros_command(comm);
	flag_processing_flash=0;
	PRINTF("END set_vendor\n");
	//
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_vendor(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	uint16_t i = 0;
	
	PRINTF("START get_vendor\n");
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_VENDOR;
	tlv_set[i].length = BLK_RSP_LEN_GET_VENDOR;//APROS Co.
	tlv_set[i++].value = node_config.__vendor;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_wireless;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_vendor\n");
	//
}
/*---------------------------------------------------------------------------*/
static uint8_t consume_wireless_payload(uint8_t* pl, uint8_t pl_len){
	static uint8_t addr_type;
	uint8_t pl_index;
	static struct tlv_set tlv;
	static struct apros_comm_param comm;
	const linkaddr_t *src = packetbuf_addr(PACKETBUF_ADDR_SENDER);
	const uint8_t is_frombase = linkaddr_cmp(src, (linkaddr_t*)(node_config.base_id+6));
	flag_processing_msg = 1;

	addr_type = verify_addr(pl); 
	if(addr_type == BR_ADDR || addr_type == MY_ADDR){
		pl_index = BLK_LEN_ID+2; //next to ID TLV
	}else{
		return 0;
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
		
		
		if(flag_registered){
			PRINTF("REGISTERED\n");
		}else{
			PRINTF("NOT REGISTERED\n");
		}

		if(flag_processing_reboot || flag_processing_flash){
			static error_cb_st_t er_node_busy;
			er_node_busy.error_type = RESPONSE_NODE_BUSY;
			my_msg_info.p_cont_block_info = find_cont_block_info(tlv.type);
			er_node_busy.msg_info = &my_msg_info;

			subfunc_error_rsp_wireless((void*)&er_node_busy);
			PRINTF("ERR: NODE BUSY\n");
			flag_processing_msg = 0;
			return -1;
		}

		switch(tlv.type){
			case BLK_TYPE_REGISTER_NODE:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_register_node;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_DEV_RESET:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_reboot = 1;
					comm.func = subfunc_dev_reset;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_9;
					
					apros_command(comm);
				}
				break;
#if APROS_PERIODIC_PKT
			case BLK_TYPE_REPORT_PERIOD:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_reboot = 1;
					comm.func = subfunc_report_period;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;
					
					apros_command(comm);
				}
				break;
			case BLK_TYPE_PROMPT_REPORT:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					comm.func = subfunc_prompt_report;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;
					
					apros_command(comm);
				}
				break;
#endif

			case BLK_TYPE_FTRY_RESET:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_reboot = 1;
					comm.func = subfunc_factory_reset;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_9;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_NODECCA:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_flash = 1;
					comm.func = subfunc_set_cca;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_NODECCA:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					comm.func = subfunc_get_cca;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_NODECH:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_flash = 1;
					comm.func = subfunc_set_nodech;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_NODECH:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					comm.func = subfunc_get_nodech;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_VENDOR:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					flag_processing_flash = 1;
					comm.func = subfunc_set_vendor;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;
					
					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_VENDOR:
				if(flag_registered && addr_type == MY_ADDR && is_frombase){
					comm.func = subfunc_get_vendor;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
		}

		pl_index += tlv.length + 2; //length(1) + type(1) + value
	}
	flag_processing_msg = 0;
	return 1;
}
/*---------------------------------------------------------------------------*/
static void packet_recv(uint8_t* pbuf, uint16_t plen){
	static uint8_t pl[APROS_PL_MAX_SIZE], pl_length;
	PRINTF("RECV packet : %d\n", plen);
	for(uint16_t i = 0 ; i < plen ; i++){
		PRINTF("%02x ", pbuf[i]);
	}
	PRINTF(" \n");

	if(apros_node_pkt_parser(pbuf, plen, pl, &pl_length) == 1 && !flag_processing_msg){
		consume_wireless_payload(pl, pl_length);
		PRINTF("CONSUME PKT\n");
	}else{
		PRINTF("INVALID FORMAT\n");
	}
}
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND * node_config.pkt_period/2)
#define SENSOR_READING_RANDOM (CLOCK_SECOND * node_config.pkt_period/2)
#if APROS_LIGHT_ENABLE
static struct ctimer opt_timer;
#endif
static struct ctimer hdc_timer;
/*---------------------------------------------------------------------------*/
#if APROS_LIGHT_ENABLE
static void init_opt_reading(void *not_used);
#endif
static void init_hdc_reading(void *not_used);
#if APROS_PIR_ENABLE
//static void init_ira_reading(void* not_used);
static void init_ekmc_reading(void* not_used);
#endif
/*---------------------------------------------------------------------------*/
#if APROS_LIGHT_ENABLE
static void
get_light_reading()
{
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  val_light = opt_3001_sensor.value(0);
  if(val_light != CC26XX_SENSOR_READING_ERROR) {
    //printf("OPT: Light=%d.%02d lux\n", val_light / 100, val_light % 100);
  } else {
    //printf("OPT: Light Read Error\n");
  }
  val_light /= 100;
  if(val_light > 65535){
	val_light = 65535;
  }
  /* The OPT will turn itself off, so we don't need to call its DEACTIVATE */
  ctimer_set(&opt_timer, next, init_opt_reading, NULL);
}
#endif
/*---------------------------------------------------------------------------*/
static void
get_hdc_reading()
{
  clock_time_t next = SENSOR_READING_PERIOD +
    (random_rand() % SENSOR_READING_RANDOM);

  val_temp = hdc_1010_sensor.value(HDC_1010_SENSOR_TYPE_TEMP);
  if(val_temp != CC26XX_SENSOR_READING_ERROR) {
    //printf("HDC: Temp=%d.%02d C\n", val_temp / 100, val_temp % 100);
  } else {
   // printf("HDC: Temp Read Error\n");
  }

  val_humid = hdc_1010_sensor.value(HDC_1010_SENSOR_TYPE_HUMIDITY);
  if(val_humid != CC26XX_SENSOR_READING_ERROR) {
    //printf("HDC: Humidity=%d.%02d %%RH\n", val_humid / 100, val_humid % 100);
  } else {
    //printf("HDC: Humidity Read Error\n");
  }
	
  if(val_humid < 0 || val_humid > 10000 || val_temp <= -4000 || val_temp >= 12500){
	dev_reset(NULL);
  }

  ctimer_set(&hdc_timer, next, init_hdc_reading, NULL);
}
/*---------------------------------------------------------------------------*/
#if APROS_LIGHT_ENABLE
static void
init_opt_reading(void *not_used)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}
#endif
/*---------------------------------------------------------------------------*/
static void
init_hdc_reading(void *not_used)
{
  SENSORS_ACTIVATE(hdc_1010_sensor);
}
/*---------------------------------------------------------------------------*/
#if APROS_PIR_ENABLE
static void init_ekmc_reading(void* not_used)
{
  SENSORS_ACTIVATE(ekmc_1603113_sensor);
}
/*
static void
init_ira_reading(void *not_used)
{
  SENSORS_ACTIVATE(pir_hi_sensor);
  SENSORS_ACTIVATE(pir_lo_sensor);
}
*/
#endif
/*---------------------------------------------------------------------------*/
PROCESS(cc13xx_process, "cc13xx process");
AUTOSTART_PROCESSES(&cc13xx_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc13xx_process, ev, data)
{
  PROCESS_BEGIN();
#if APROS_RADIO_OFF_MODE
  NETSTACK_RADIO.off();
#endif
  init_setting();
#if APROS_LIGHT_ENABLE
  init_opt_reading(NULL);
#endif
  init_hdc_reading(NULL);
#if APROS_PIR_ENABLE
  init_ekmc_reading(NULL);
//init_ira_reading(NULL); 
#endif

#if APROS_INIT_TIME
  etimer_set(&et, CLOCK_SECOND*30);
  stimer_set(&st, 25);
#if APROS_RADIO_OFF_MODE
  NETSTACK_RADIO.on(); 
#endif
#else
  etimer_set(&et, 10*CLOCK_SECOND);
  stimer_set(&st, 5);
#endif
  while(1) {
    PROCESS_YIELD();
	
   	if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {	
#if APROS_PERIODIC_PKT
#if APROS_RADIO_OFF_MODE
		  NETSTACK_RADIO.on();
		  if(flag_registered){
		  	packet_send_periodic(NULL);
		  }
		  NETSTACK_RADIO.off();
#else
		  if(flag_registered){
			packet_send_periodic(NULL);
	      }
#endif
		  etimer_set(&et, CLOCK_SECOND*node_config.pkt_period);

#endif
       }
  	 } 
     if(ev == sensors_event && data == &hdc_1010_sensor ){
	     get_hdc_reading(); 
     }
#if APROS_LIGHT_ENABLE 
     else if(ev == sensors_event && data == &opt_3001_sensor) {
        get_light_reading();
     }
#endif
#if APROS_PIR_ENABLE
     else if(ev == sensors_event && data == &ekmc_1603113_sensor){
	 	val_pir = 1;
	 } 
#endif

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */ 
