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
#include "sys/rtimer.h"
#include "dev/leds.h"
#include "lib/sensors.h"
#include "dev/watchdog.h"
#include "random.h"
#include "board-peripherals.h"
#include "net/rime/rime.h"

#include "ieee-addr.h"
#include "dev/uart1.h"
#include "apros-wireless-line.h"
#include "apros-command-scheduler.h"
#include "apros-serial-line.h"
#include "apros-gpio-control.h"
#include "apros-packet-control.h"
#include "apros-flash-control.h"
#include "net/netstack.h"
#include "adxl355.h"

#include "ti-lib.h"

#include "cc13xx-node.h"

#include <stdio.h>

#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

/*****************************************************************************
Global Variable & Definition
******************************************************************************/

/*---------------------------------------------------------------------------*/
/** DEV Identity **/
#define NAME 0xADC2005

#define DEFAULT_VENDOR_NAME "APROS Co."
/** Channel **/
#define DEFAULT_NODE_CH 5

#define DEFAULT_CCA_THRESHOLD -70

#define MAX_CCA_THRESHOLD -60
#define MIN_CCA_THRESHOLD -120

/** Sampling **/
#define MIN_SAMPLING_EXP 6  // 64
#define MIN_SAMPLING_NUM (2<<(MIN_SAMPLING_EXP-1))

#define MAX_SAMPLING_EXP 10 // 1024
#define MAX_SAMPLING_NUM (2<<(MAX_SAMPLING_EXP-1))

#define DEFAULT_SAMPLING_EXP 8

/** Offset **/
#ifdef SENSOR_BOARD_CONF_OFFSET
#define DEFAULT_OFFSET SENSOR_BOARD_CONF_OFFSET
#else
#define DEFAULT_OFFSET 0
#endif 
/** Bit Unit **/
#define BYTE_UNIT 3 // 24bit

/** Axis **/
#define MAX_AXIS 3
#define MIN_AXIS 1
/** Offset **/
#define MAX_OFFSET_BOUND_SIGNED ((1<<((BYTE_UNIT<<3)-1))-1)
#define MIN_OFFSET_BOUND_SIGNED (-(1<<((BYTE_UNIT<<3)-1)))

#define MAX_VALUE_BOUND_SIGNED ((1<<((BYTE_UNIT<<3)-1))-1)
#define MIN_VALUE_BOUND_SIGNED (-(1<<((BYTE_UNIT<<3)-1)))

#define MAX_VALUE_BOUND_UNSIGNED ((1<<(BYTE_UNIT<<3))-1)
#define MIN_VALUE_BOUND_UNSIGNED 0

/** Buffer for ADC Read **/ 
#define MAX_ACCEL_DATA_LEN 48

static uint8_t x_buf[MAX_SAMPLING_NUM*BYTE_UNIT];
static uint8_t y_buf[MAX_SAMPLING_NUM*BYTE_UNIT];
static uint8_t z_buf[MAX_SAMPLING_NUM*BYTE_UNIT];
uint8_t accel_data_littleed[MAX_ACCEL_DATA_LEN];



/** Channel **/
#define AVAILABLE_CHANNEL 0x00FD2490
// 5, 8, 11, 14, 17, 19, 20, 21, 22, 23, 24

/** flash **/
static apros_node_config_t node_config;

/** Primary Timer **/
static struct etimer et;
static struct rtimer rt;
static struct ctimer ct; 
/** Dedicated Timer **/


/** MAC Address **/
static uint8_t node_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/** Others. **/
static node_info_t my_node_info;
static msg_info_t my_msg_info;

static uint8_t last_axis_value;
static uint8_t last_accel_data_len;
static uint16_t last_max_index;

uint32_t bat_val_capt;
uint8_t temperature_capt[2];

static clock_time_t br_ack_backoff_time;

static uint8_t flag_registered;

static uint8_t flag_processing_msg;
static uint8_t flag_processing_fetch;
static uint8_t flag_processing_capt;
static uint8_t flag_processing_reboot;
static uint8_t flag_processing_flash;

static uint8_t flag_exist_capt;

static uint8_t error_pass_seq;
#if !APROS_OLD_DF
static uint8_t flag_sent_ready_rsp;
#endif
/*****************************************************************************
Funtion Declaration
******************************************************************************/


/*****************************************************************************
Function Definition
******************************************************************************/
/*---------------------------------------------------------------------------*/
/* Init  */
/*---------------------------------------------------------------------------*/
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
	node_config.sampling_exp = DEFAULT_SAMPLING_EXP;
	node_config.cca_th = DEFAULT_CCA_THRESHOLD;
	
	node_config.ofc[0] = DEFAULT_OFFSET & 0xff;
	node_config.ofc[1] = (DEFAULT_OFFSET>>8) & 0xff;
	node_config.ofc[2] = (DEFAULT_OFFSET>>16) & 0xff;
	memset(node_config.__vendor, 0, VENDOR_MAX_LEN);
	memcpy(node_config.__vendor, DEFAULT_VENDOR_NAME, sizeof(DEFAULT_VENDOR_NAME));
}
/*---------------------------------------------------------------------------*/
static void init_setting(void)
{
	ieee_addr_cpy_to(node_mac, 8);

	apros_node_config_load(&node_config);
	if(node_config.dev_name != NAME){
		cc1310_config_init();
		apros_node_config_save(&node_config);
	}
	
	memcpy(my_node_info.node_id, node_mac, 8);
	my_node_info.msg_num = 0;//not used value;
	my_node_info.msg_info = &my_msg_info;
	my_msg_info.p_node_info = &my_node_info;
	my_msg_info.p_cont_block_info = NULL;//Null value

	if(linkaddr_cmp((linkaddr_t*)(node_config.base_id+6), &linkaddr_null)){
		flag_registered = 0;
	}else{
		flag_registered = 1;
	}

	add_all_block_info();
	  
	br_ack_backoff_time = calc_backoff(node_mac);
	apply_channel(node_config.channel);
	apply_cca(node_config.cca_th);
  	apros_serial_line_init();
  	apros_wireless_line_init(NULL, NULL);	
}
/*---------------------------------------------------------------------------*/
/* Communication  */
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
			//apros_serial_send(pkt, get_pkt_length(pkt));
			apros_send_broadcast(pkt, get_pkt_length(pkt));
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
static void subfunc_response_serial(void* ptr){
	tlv_buf_hdr_t* tlv_buf_hdr = (tlv_buf_hdr_t*)ptr;

	generate_pkt_serial(MSG_TYPE_CONTROL, tlv_buf_hdr->tlv_p, tlv_buf_hdr->tlv_num);
}
/*---------------------------------------------------------------------------*/
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
	
	rsp[0] = er->error_type;
	for(uint8_t j=1 ;j < tlv_set[i].length ; j++){
		rsp[j] = 0xff;
	}
	if(er->msg_info->p_cont_block_info->type == BLK_TYPE_DATA_FETCH_READY){
		rsp[1] = error_pass_seq;
	}
	if(er->msg_info->p_cont_block_info->type == BLK_TYPE_DATA_FETCH_START){
		rsp[1] = error_pass_seq;
	}
	tlv_set[i++].value = rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;

	generate_pkt_serial(MSG_TYPE_CONTROL, tlv_hdr.tlv_p, tlv_hdr.tlv_num);
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

	generate_pkt_serial( MSG_TYPE_CONTROL, 
		tlv_hdr.tlv_p, tlv_hdr.tlv_num);
	
	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);
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
	
	generate_pkt_serial(MSG_TYPE_CONTROL, 
		tlv_hdr.tlv_p, tlv_hdr.tlv_num);

	cc1310_config_init();
	
	apros_node_config_save(&node_config);
	
	ctimer_set(&ct, CLOCK_SECOND, dev_reset, NULL);	
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
	
	comm.func = subfunc_response_serial;
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
	
	comm.func = subfunc_response_serial;
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
	
	comm[0].func = subfunc_response_serial;
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
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_node_ch\n");
	
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_nodeinfo(void* ptr){	
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp[BLK_RSP_LEN_GET_NODEINFO];
	
	uint16_t i = 0;
	uint8_t j = 0;
	uint8_t addr_type;

	uint32_t bat_val;
	uint8_t temperature[2];
	PRINTF("START get_nodeinfo\n");
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_NODEINFO;
	tlv_set[i].length = BLK_RSP_LEN_GET_NODEINFO;

	rsp[j++] = RESPONSE_SUCCESS; //Status
	rsp[j++] = (MIN_SAMPLING_EXP<<4) | (MAX_SAMPLING_EXP&0x0f); //Sampling Range
	rsp[j++] = node_config.sampling_exp; //Sampling
	rsp[j++] = BYTE_UNIT; 
	rsp[j++] = (MIN_AXIS<<4) | (MAX_AXIS&0x0f); //Shaft Range
	rsp[j] = AVAILABLE_CHANNEL&0xff;
	rsp[j+1] = (AVAILABLE_CHANNEL >> 8)&0xff;
	rsp[j+2] = (AVAILABLE_CHANNEL >> 16)&0xff;
	rsp[j+3] = (AVAILABLE_CHANNEL >> 24)&0xff; //Available Channel
	j+=4;
	rsp[j++] = node_config.channel;

	// Offset
	rsp[j] = 0;
	rsp[j+1] = node_config.ofc[0];
	rsp[j+2] = node_config.ofc[1];
	rsp[j+3] = node_config.ofc[2];
	rsp[j+4] = 0;
	j+=5;
	// Base ID
	for(uint8_t k = 0 ; k < BLK_LEN_ID ; k ++){
		if(flag_registered){
			rsp[j+k] = node_config.base_id[k];
		}else{
			rsp[j+k] = 0;
		}
	}
	j+=BLK_LEN_ID;
	// Vendor
	memcpy(rsp+j, node_config.__vendor, VENDOR_MAX_LEN);
	j+=VENDOR_MAX_LEN;
	// Firmware Version
	for(uint8_t k = 0 ; k < 4 ; k ++){
		rsp[j+k] = (NAME>>(k<<3))&0xff;
	}
	j+=4;
	// Battery
	/*bat_val = read_bat_voltage();
	for(uint8_t k = 0 ; k < 2 ; k ++){
		rsp[j+k] = (bat_val>>(k<<3))&0xff;
	}*/
	if(bat_val_capt == 0){ //not captured yet
		bat_val = 0xffff; 
	}else{
		bat_val = bat_val_capt;
	}
	for(uint8_t k = 0 ; k < 2 ; k ++){
		rsp[j+k] = (bat_val>>(k<<3))&0xff;
	}
	j+=2;
	// Temperature
	/*temperature = 0; //TODO::
	for(uint8_t k = 0 ; k < 2; k ++){
		rsp[j+k] = (temperature>>(k<<3))&0xff;
	}*/
	if(temperature_capt[0] == 0 && temperature_capt[1] == 0){ //not captured yet
		temperature[0] = temperature[1] = 0xff;
	}else{
		temperature[0] = temperature_capt[0];
		temperature[1] = temperature_capt[1];
	}
	rsp[j] = temperature[1];
	rsp[j+1] = temperature[0];
	j+=2;
	
	tlv_set[i++].value = rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;

	addr_type = *((uint8_t*)ptr);
	if(addr_type == BR_ADDR){
		comm.t_backoff = br_ack_backoff_time;
	}else{
		comm.t_backoff = APROS_COMM_PRIOR_1;
	}

	apros_command(comm);
	PRINTF("END get_nodeinfo\n");
	//
}
/*---------------------------------------------------------------------------*/
static void subfunc_set_sampling(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint16_t i = 0;
	uint8_t sampling_backup;
	
	PRINTF("START set_sampling\n");	
	
	sampling_backup = node_config.sampling_exp;
	node_config.sampling_exp = *((uint8_t*)ptr);
	
	if(apros_node_config_save(&node_config) && 
		node_config.sampling_exp <= MAX_SAMPLING_EXP &&
		node_config.sampling_exp >= MIN_SAMPLING_EXP){
		rsp = RESPONSE_SUCCESS;
		PRINTF("SAMPLING_EXP = %d\n", node_config.sampling_exp);
	}else{
	    if(node_config.sampling_exp <= MAX_SAMPLING_EXP &&
		node_config.sampling_exp >= MIN_SAMPLING_EXP){
			rsp = RESPONSE_FLASH_ERROR;
	    }else{
			rsp = RESPONSE_INVALID_ARGU;
		}
		node_config.sampling_exp = sampling_backup;
		apros_node_config_save(&node_config);
		PRINTF("FAIL\n");
	}
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_SAMPLING;
	tlv_set[i].length = BLK_RSP_LEN_SET_SAMPLING;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	if(sampling_backup != node_config.sampling_exp){
		flag_exist_capt = 0;
		flag_sent_ready_rsp = 0;
	}
	apros_command(comm);
	flag_processing_flash=0;
	PRINTF("END set_sampling\n");
	//
}
/*---------------------------------------------------------------------------*/
static void subfunc_get_sampling(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t sampling;

	uint16_t i = 0;
	
	PRINTF("START get_sampling\n");

	if(node_config.sampling_exp <= MAX_SAMPLING_EXP &&
		node_config.sampling_exp > 0){
		sampling = node_config.sampling_exp;	
	}else{
		sampling = RESPONSE_INVALID_ARGU;
	}
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_SAMPLING;
	tlv_set[i].length = BLK_RSP_LEN_GET_SAMPLING;
	tlv_set[i++].value = &sampling;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_sampling\n");
	//
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
	
	comm.func = subfunc_response_serial;
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
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_vendor\n");
	//
}
static void subfunc_data_fetch_start(void* ptr);
static void subfunc_data_fetch_ready(void* ptr);

/*---------------------------------------------------------------------------*/
static void subfunc_capture_req(void* ptr){
	static uint16_t sampling_num;

	static uint16_t max_index;
	
	static uint16_t index = 0;
	static uint8_t init_sensor = 0;
	uint32_t x_val_32, y_val_32, z_val_32;
	if(!init_sensor){
		sampling_num = (1 << node_config.sampling_exp);
		max_index = sampling_num*BYTE_UNIT;
		init_sensor = 1;
		ctimer_set(&ct, 0, subfunc_capture_req, NULL);
		return;
	}

	z_val_32 = adxl355.value(ADXL355_READ_Z);
	x_val_32 = adxl355.value(ADXL355_READ_X);
	y_val_32 = adxl355.value(ADXL355_READ_Y);


	x_buf[index] = x_val_32 >> 16;
	x_buf[index+1] = x_val_32 >> 8;
	x_buf[index+2] = x_val_32;

	y_buf[index] = y_val_32 >> 16;
	y_buf[index+1] = y_val_32 >> 8;
	y_buf[index+2] = y_val_32;
	
	z_buf[index] = z_val_32 >> 16;
	z_buf[index+1] = z_val_32 >> 8;
	z_buf[index+2] = z_val_32;

	
	index += BYTE_UNIT;
	
	if(index >= max_index){
 
		index = 0;
		//flag_exist_capt = 0;
		//flag_sent_ready_rsp = 0;
		flag_exist_capt = 1;
		flag_processing_capt = 0;
		ctimer_set(&ct, 0, subfunc_data_fetch_ready, NULL);
		max_index = 0;
		init_sensor = 0;
	}else{
		//ctimer_set(&ct, 1, (void *)subfunc_caputre_req, NULL); 
		rtimer_set(&rt, RTIMER_NOW()+RTIMER_ARCH_SECOND/sampling_num-13, 0, (void *)subfunc_capture_req, NULL); 
		//warning : rtimer cant pass parameter with NULL.
	}
}
/*---------------------------------------------------------------------------*/
static void subfunc_data_fetch_ready(void* ptr){
	uint8_t max_index_8[2]={0,0};
	uint16_t max_index;
	
	uint8_t axis;
	uint8_t sam_divisor;
	uint16_t sampling_num;

	uint8_t accel_data_len;

	uint8_t seq_num;

	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];

	uint16_t i;
	
	static uint8_t rsp[BLK_RSP_LEN_DATA_FETCH_READY];	
	PRINTF("START fetch ready\n");
	if(ptr != NULL){
		axis = ((uint8_t*)ptr)[0];
		seq_num = ((uint8_t*)ptr)[1]; 
	}else{
		axis = 3; //x, y, z
		seq_num = 0;
	}
	if(axis > MAX_AXIS || axis < 1){
		rsp[0] = RESPONSE_INVALID_ARGU;
		PRINTF("FAIL: Invalid Argument\n");
	}else if(!flag_exist_capt){
		rsp[0] = RESPONSE_NOT_EXIST_CAPT;
		PRINTF("FAIL: Not exist Capture\n");
	}else{
		rsp[0] = RESPONSE_SUCCESS;
		
		//calculate max index
		sampling_num  = (2 << (node_config.sampling_exp-1));
		sam_divisor = MAX_ACCEL_DATA_LEN/axis/BYTE_UNIT; //Never change to just "sam_divisor" //TODO:: replace division with shift op
		if(sampling_num % sam_divisor == 0){
			max_index = (sampling_num / sam_divisor - 1); //TODO:: replace division with shift op
		}else{
			max_index = (sampling_num / sam_divisor); //TODO:: replace division with shift op
		}

		max_index_8[0] = max_index & 0xff;
		max_index_8[1] = (max_index >> 8) & 0xff;

		accel_data_len = sam_divisor * axis * BYTE_UNIT;

		last_axis_value = axis;
		last_accel_data_len = accel_data_len;
		last_max_index = max_index;
					
		PRINTF("ACCEL_DATA_LEN = %d\n", accel_data_len);
		PRINTF("MAX_INDEX = %d\n", max_index);
		PRINTF("SAMPLING_EXP = %d\n", node_config.sampling_exp);
		PRINTF("SAMPLING_NUM = %d\n", sampling_num);
	}
	
	i = 0;
	
	tlv_set[i].type = BLK_TYPE_ID;   
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_DATA_FETCH_READY;
	tlv_set[i].length = BLK_RSP_LEN_DATA_FETCH_READY;
	
	if(rsp[0] == RESPONSE_SUCCESS){
		rsp[1] = seq_num;
		rsp[2] = node_config.sampling_exp;
		rsp[3] = BYTE_UNIT;
		rsp[4] = axis;
		rsp[5] = max_index_8[0];
		rsp[6] = max_index_8[1];
		rsp[7] = bat_val_capt & 0xff;
		rsp[8] = (bat_val_capt >> 8) & 0xff;
		rsp[9] = temperature_capt[1]; //temperature
		rsp[10] = temperature_capt[0]; //temperatue
	}else{
		rsp[1] = seq_num;
		rsp[2] = 0xff;
		rsp[3] = 0xff;
		rsp[4] = 0xff;
		rsp[5] = 0xff;
		rsp[6] = 0xff;
		rsp[7] = 0xff;
		rsp[8] = 0xff;
		rsp[9] = 0xff;
		rsp[10] = 0xff;
	}
	
	tlv_set[i++].value = rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	
	apros_command(comm);

	if(rsp[0] == RESPONSE_SUCCESS){
		flag_sent_ready_rsp = 1;
		
	}
	if(ptr == NULL){
			ctimer_set(&ct, 3, subfunc_data_fetch_start, NULL);
	}

	PRINTF("END fetch ready\n");
}

/*---------------------------------------------------------------------------*/
static void subfunc_data_fetch_start(void* ptr){
	static uint16_t pkt_index = 0;
	static uint8_t seq_num = 0;
	uint8_t pkt_index_8[2];

	static uint16_t c_index = 0; // index in the buffer such as z_axis_val, x_axis_val,.. 

	static uint16_t sampling_num;

	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[3];
	
	
	uint16_t i=0;

	static uint8_t rsp[2];
	
	if(c_index == 0){
		seq_num = *((uint8_t*)ptr);
		if(!flag_sent_ready_rsp){
			PRINTF("FAIL\n");
			tlv_set[i].type = BLK_TYPE_ID;
			tlv_set[i].length = BLK_LEN_ID;
			tlv_set[i++].value = node_mac;
			
			tlv_set[i].type = BLK_TYPE_DATA_FETCH_START;
			tlv_set[i].length = BLK_RSP_LEN_DATA_FETCH_START;

			seq_num = *((uint8_t*)ptr);
			rsp[0] = RESPONSE_NOT_FETCH_RDY;
			rsp[1] = seq_num;
			tlv_set[i++].value = rsp;

			tlv_hdr.tlv_p = tlv_set;

			tlv_hdr.tlv_num = i;
		
			comm.func = subfunc_response_serial;
			comm.func_param = &tlv_hdr;
			comm.t_backoff = APROS_COMM_PRIOR_1;
			c_index = 0;
			pkt_index = 0;
			
			apros_command(comm);

			flag_processing_fetch = 0;
			PRINTF("END fetch start\n");
			return;
		}else{
			PRINTF("START fetch start\n");
#if APROS_LED_EXIST
		    leds_on(LEDS_BLUE);
#endif
			sampling_num  = (1 << node_config.sampling_exp);
		}
	}
	
	i = 0;

	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_ACCEL_INDEX;
	tlv_set[i].length = BLK_LEN_ACCEL_INDEX;
	
	pkt_index_8[0] = pkt_index & 0xff;
	pkt_index_8[1] = (pkt_index >> 8) & 0xff;
	
	tlv_set[i++].value = pkt_index_8;

	tlv_set[i].type = BLK_TYPE_ACCEL_DATA;
	if(c_index + last_accel_data_len/last_axis_value > sampling_num*BYTE_UNIT){
		tlv_set[i].length = (sampling_num*BYTE_UNIT-c_index)*last_axis_value;
	}else{
		tlv_set[i].length = last_accel_data_len;
	}
	//little endian
	{
		uint8_t ad_index = 0; //accel data index
		for(uint16_t k = c_index ; k < c_index+tlv_set[i].length ; k+=BYTE_UNIT){ 
			for(uint8_t j = 0 ; j < BYTE_UNIT ; j++){
				accel_data_littleed[ad_index+BYTE_UNIT-1-j] = x_buf[k+j];
			}
			ad_index += BYTE_UNIT;
			
			for(uint8_t j = 0 ; j < BYTE_UNIT ; j++){
				accel_data_littleed[ad_index+BYTE_UNIT-1-j] = y_buf[k+j];
			}
			ad_index += BYTE_UNIT;
			
			for(uint8_t j = 0 ; j < BYTE_UNIT ; j++){
				accel_data_littleed[ad_index+BYTE_UNIT-1-j] = z_buf[k+j];
			}
			ad_index += BYTE_UNIT;
		}
		c_index += tlv_set[i].length/last_axis_value;
	}
	tlv_set[i].value = accel_data_littleed;
	
	//big endian
	//tlv_set[i].value = z_axis_val+z_index; 
	
	i++;
	generate_pkt_serial( MSG_TYPE_RAWDATA, tlv_set, i);
	
	pkt_index++;
	
	if(c_index >= sampling_num*BYTE_UNIT){	
		
		i = 0;
		
		tlv_set[i].type = BLK_TYPE_ID;
		tlv_set[i].length = BLK_LEN_ID;
		tlv_set[i++].value = node_mac;
		
		tlv_set[i].type = BLK_TYPE_DATA_FETCH_START;
		tlv_set[i].length = BLK_RSP_LEN_DATA_FETCH_START;
		rsp[0] = RESPONSE_SUCCESS;
		rsp[1] = seq_num; 
		tlv_set[i++].value = rsp;

		tlv_hdr.tlv_p = tlv_set;

		tlv_hdr.tlv_num = i;
		
		comm.func = subfunc_response_serial;
		comm.func_param = &tlv_hdr;
		comm.t_backoff = 6;
		
		c_index = 0;
		seq_num = 0;
		pkt_index = 0;
		
		apros_command(comm);
#if APROS_LED_EXIST   
		leds_off(LEDS_CONF_ALL);
#endif
		flag_processing_fetch = 0;
		ctimer_set(&ct, 10, subfunc_capture_req, NULL);
		PRINTF("END fetch start\n");
		return;
	}else{
#if APROS_LED_EXIST
		leds_toggle(LEDS_RED);
#endif
		ctimer_set(&ct, 5, subfunc_data_fetch_start, NULL);
		return;
	}
}
/*---------------------------------------------------------------------------*/
/*static void subfunc_retrans_req(void* ptr){
	uint16_t pkt_index;
	static uint8_t pkt_index_8[2];

	static uint8_t accel_data_len;
//	static uint8_t accel_data_cp[MAX_ACCEL_DATA_LEN];
	
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[6];
	
	static uint8_t accel_data_littleed[MAX_ACCEL_DATA_LEN];
	
	uint16_t i;
	
	static uint8_t rsp[BLK_RSP_LEN_RETT];
	
    PRINTF("START rett\n");
	pkt_index_8[0] = ((uint8_t*)ptr)[0];
	pkt_index_8[1] = ((uint8_t*)ptr)[1];

	pkt_index = ((uint16_t)pkt_index_8[1] << 8) | pkt_index_8[0];	

	if(last_accel_data_len == 0||last_axis_value == 0){
		rsp[0] = RESPONSE_NOT_EXIST_CAPT;
		PRINTF("FAIL\n");
	}else if(pkt_index > last_max_index||pkt_index < 0){
		rsp[0] = RESPONSE_INVALID_ARGU;
		PRINTF("FAIL\n");
	}else{
		rsp[0] = RESPONSE_SUCCESS;
		PRINTF("INDEX = %d\n", pkt_index);
		
		i = 0;
		
		tlv_set[i].type = BLK_TYPE_ID;
		tlv_set[i].length = BLK_LEN_ID;
		tlv_set[i++].value = node_mac;
		
		tlv_set[i].type = BLK_TYPE_ACCEL_INDEX;
		tlv_set[i].length = BLK_LEN_ACCEL_INDEX;
		tlv_set[i++].value = pkt_index_8;
		
		tlv_set[i].type = BLK_TYPE_ACCEL_DATA;
		accel_data_len = last_accel_data_len;
		tlv_set[i].length = accel_data_len;
		
		//little endian
		{
			uint8_t ad_index = 0;
			uint16_t r_index = pkt_index*accel_data_len;
			for(uint16_t i = r_index ; i < r_index+last_accel_data_len ; i+=BYTE_UNIT){ 
				for(uint8_t j = 0 ; j < BYTE_UNIT ; j++){
					accel_data_littleed[ad_index+BYTE_UNIT-1-j] = z_axis_val[i+j];
				}
				ad_index += BYTE_UNIT;
			}
		}
		tlv_set[i++].value = accel_data_littleed;

		//big endian
		//tlv_set[i++].value = z_axis_val+pkt_index*accel_data_len;
		
		//tlv_set[i++].value = accel_data_cp;
		generate_pkt_serial( MSG_TYPE_RAWDATA, tlv_set, i);
	}
	
	i = 0;
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_RETT;
	tlv_set[i].length = BLK_RSP_LEN_RETT;

	if(rsp[0] == RESPONSE_SUCCESS){
		rsp[1] = node_config.sampling_exp;
		rsp[2] = BYTE_UNIT;
		rsp[3] = last_axis_value;
		rsp[4] = pkt_index_8[0];
		rsp[5] = pkt_index_8[1];
	}else{
		rsp[1] = 0xff;
		rsp[2] = 0xff;
		rsp[3] = 0xff;
		rsp[4] = 0xff;
		rsp[5] = 0xff;
	}
	
	tlv_set[i++].value = rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	
	apros_command(comm);
	PRINTF("END rett\n");
}*/
/*---------------------------------------------------------------------------*/
/*static uint8_t subfunc_write_offset(uint8_t* offset_arr){
	uint8_t ofc_backup[3];
	int32_t offset_value;
	uint8_t ret;

	offset_value = 
		offset_arr[0]|
		((offset_arr[1]>>8)&0xff)|
		((offset_arr[2]>>16)&0xff)|
		((offset_arr[3]>>24)&0xff);

	if(offset_value > 0xFFFFFF){//only for 24bit
		return RESPONSE_INVALID_ARGU;
	}
	else if(offset_value > 0x7FFFFF){//only for 24bit
		offset_value = offset_value - 0x1000000;
	}

	for(uint8_t i = 0 ; i < BYTE_UNIT; i++){
		ofc_backup[i] = node_config.ofc[i];
		node_config.ofc[i] = offset_arr[i];
	}
	
	if(apros_node_config_save(&node_config) && 
		offset_value<= MAX_OFFSET_BOUND_SIGNED &&
		offset_value>= MIN_OFFSET_BOUND_SIGNED){
		ret = RESPONSE_SUCCESS;
		//ads127l01_register_write_8(ADS127L01_REG_OFC0, node_config.ofc[0]);
		//ads127l01_register_write_8(ADS127L01_REG_OFC1, node_config.ofc[1]);
		//ads127l01_register_write_8(ADS127L01_REG_OFC2, node_config.ofc[2]);
		PRINTF("OFFSET = %d\n", (int)offset_value);
	}else{
		if(offset_value<= MAX_OFFSET_BOUND_SIGNED &&
		offset_value>= MIN_OFFSET_BOUND_SIGNED){
			ret = RESPONSE_FLASH_ERROR;
		}else{
			ret = RESPONSE_INVALID_ARGU;
		}

		for(uint8_t i = 0 ; i < BYTE_UNIT; i++){
			node_config.ofc[i] = ofc_backup[i];
		}
		apros_node_config_save(&node_config);
		PRINTF("FAIL\n");
	}
	return ret;
}*/
/*---------------------------------------------------------------------------*/
/*static void subfunc_set_offset(void* ptr){
	apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	
	static uint8_t rsp;

	uint8_t* offset_input;
	uint16_t i = 0;
	//uint8_t ofc_backup[3];
	
	PRINTF("START set_offset\n");	

	offset_input = (uint8_t*)ptr;

	rsp = subfunc_write_offset(offset_input);
	
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_SET_OFFSET;
	tlv_set[i].length = BLK_RSP_LEN_SET_OFFSET;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;
	
	apros_command(comm);
	flag_processing_flash=0;
	PRINTF("END set_offset\n");
	//
}*/
/*---------------------------------------------------------------------------*/
/*static void subfunc_get_offset(void* ptr){
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	static uint8_t converted_arr[5];
	uint16_t i = 0;
	
	PRINTF("START get_offset\n");

	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_GET_OFFSET;
	tlv_set[i].length = BLK_RSP_LEN_GET_OFFSET;

	converted_arr[0] = RESPONSE_SUCCESS;
	for(uint8_t i=0; i<3; i++){//only for 24bit
		converted_arr[i+1] = node_config.ofc[i]; 
	}
	
	tlv_set[i++].value = converted_arr;

	tlv_hdr.tlv_p = tlv_set;

	tlv_hdr.tlv_num = i;
	
	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	PRINTF("END get_offset\n");
	//
}*/
/*---------------------------------------------------------------------------*/
/*static void subfunc_auto_offset_calibration(void* ptr){
	static apros_comm_param_t comm;
	static tlv_buf_hdr_t tlv_hdr;
	static tlv_set_t tlv_set[2];
	static uint8_t rsp;
	uint16_t i = 0;
	uint16_t sampling_num = (2 << (node_config.sampling_exp-1));
	int32_t max_z_val=0, min_z_val=0, center_z_val;
	//int32_t org_offset;
	uint8_t offset_input[4];
	
	PRINTF("START auto_offset_calibration\n");
	
	for(uint16_t i = 0 ; i < sampling_num ; i++){
		int32_t z_val = 0;
		for(uint16_t j = 0 ; j < BYTE_UNIT ; j++){
			z_val |= z_axis_val[i*BYTE_UNIT+j]<<((BYTE_UNIT-1-j)<<3);
		};
		
		if(z_val > 0x7FFFFF){
			z_val = (z_val&0xffffff)-0x1000000;
		}
		if(i == 0){
			max_z_val = min_z_val = z_val;
		}else{
			if(z_val > max_z_val){
			max_z_val = z_val;
			}	
			if(z_val < min_z_val){
				min_z_val = z_val;
			}
		}
	}

	center_z_val = (max_z_val + min_z_val) / 2 ;//+ org_offset;
	//printf("center_z_val = %03x\n", center_z_val);
	for(uint8_t i = 0 ; i < 4 ; i ++){
		offset_input[i] = (center_z_val>>(i<<3))&0xff; 
	}
		
	rsp=subfunc_write_offset(offset_input);
		
	tlv_set[i].type = BLK_TYPE_ID;
	tlv_set[i].length = BLK_LEN_ID;
	tlv_set[i++].value = node_mac;
	
	tlv_set[i].type = BLK_TYPE_AUTO_OFFSET;
	tlv_set[i].length = BLK_RSP_LEN_AUTO_OFFSET;
	tlv_set[i++].value = &rsp;

	tlv_hdr.tlv_p = tlv_set;
	tlv_hdr.tlv_num = i;

	comm.func = subfunc_response_serial;
	comm.func_param = &tlv_hdr;
	comm.t_backoff = APROS_COMM_PRIOR_1;

	apros_command(comm);
	ads127l01_disable();
	PRINTF("END auto_offset_calibration\n");
}*/
/*---------------------------------------------------------------------------*/
/*static void subfunc_realtime(void* ptr){
	//TODO:: mode change to realtime sending
}*/
/*---------------------------------------------------------------------------*/
static uint8_t consume_serial_payload(uint8_t* pl, uint8_t pl_len){
	static uint8_t addr_type;
	uint8_t pl_index;
	static struct tlv_set tlv;
	static struct apros_comm_param comm;
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
		
		if(flag_processing_fetch || flag_processing_capt || flag_processing_reboot || flag_processing_flash){
			static error_cb_st_t er_node_busy;
			er_node_busy.error_type = RESPONSE_NODE_BUSY;
			my_msg_info.p_cont_block_info = find_cont_block_info(tlv.type);
			er_node_busy.msg_info = &my_msg_info;

			if(tlv.type == BLK_TYPE_DATA_FETCH_READY){
				error_pass_seq = tlv.value[1];
			}
			if(tlv.type == BLK_TYPE_DATA_FETCH_START){
				error_pass_seq = tlv.value[0];
			}
			subfunc_error_rsp_serial((void*)&er_node_busy);
			PRINTF("ERR: NODE BUSY\n");
			flag_processing_msg = 0;
			return -1;
		}

		switch(tlv.type){
			case BLK_TYPE_DEV_RESET:
				if(addr_type == MY_ADDR){
					flag_processing_reboot = 1;
					comm.func = subfunc_dev_reset;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_9;
					
					apros_command(comm);
				}
				break;
			case BLK_TYPE_FTRY_RESET:
				if( addr_type == MY_ADDR){
					flag_processing_reboot = 1;
					comm.func = subfunc_factory_reset;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_9;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_NODECCA:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_set_cca;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_NODECCA:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_cca;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_NODECH:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_set_nodech;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_NODECH:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_nodech;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_SAMPLING:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_set_sampling;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;
					
					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_SAMPLING:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_sampling;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_SET_VENDOR:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_set_vendor;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;
					
					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_VENDOR:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_vendor;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			/*case BLK_TYPE_CAPT_REQUEST:	
					flag_processing_capt = 1;
					NETSTACK_RDC.off(1);
					comm.func = subfunc_capture_req;
					comm.func_param = &addr_type; 
					comm.t_backoff = APROS_COMM_PRIOR_3;

					apros_command(comm);
	
				break;
			case BLK_TYPE_DATA_FETCH_READY:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_data_fetch_ready;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_DATA_FETCH_START:
				if(addr_type == MY_ADDR){
					flag_processing_fetch = 1;
					comm.func = subfunc_data_fetch_start;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;*/
			case BLK_TYPE_GET_NODEINFO:
					comm.func = subfunc_get_nodeinfo;
					comm.func_param = &addr_type;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				break;
		/*	case BLK_TYPE_RETT:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_retrans_req;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
		
			case BLK_TYPE_SET_OFFSET:
				if(addr_type == MY_ADDR){
					flag_processing_flash = 1;
					comm.func = subfunc_set_offset;
					comm.func_param = tlv.value;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_GET_OFFSET:
				if(addr_type == MY_ADDR){
					comm.func = subfunc_get_offset;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_1;

					apros_command(comm);
				}
				break;
			case BLK_TYPE_AUTO_OFFSET:
				if(addr_type == MY_ADDR){
					flag_processing_capt = 1;
					NETSTACK_RDC.off(1);
					comm.func=subfunc_capture_req;
					comm.func_param = NULL;

					apros_command(comm);
										
					comm.func = subfunc_auto_offset_calibration;
					comm.func_param = NULL;
					comm.t_backoff = APROS_COMM_PRIOR_9+APROS_COMM_PRIOR_9;

					apros_command(comm);
				}
				break;
				*/
		}

		pl_index += tlv.length + 2; //length(1) + type(1) + value
	}
	flag_processing_msg = 0;
	return 1;
}
/*---------------------------------------------------------------------------*/
static void capture_send(void* ptr){
	subfunc_capture_req(NULL);
}
/*---------------------------------------------------------------------------*/
PROCESS(cc13xx_process, "cc13xx process");
PROCESS(uart_rx_process, "uart rx process");
AUTOSTART_PROCESSES(&cc13xx_process, &uart_rx_process);
/*---------------------------------------------------------------------------*/ 
PROCESS_THREAD(cc13xx_process, ev, data)
{
  PROCESS_BEGIN();
  
  init_setting();
  SENSORS_ACTIVATE(adxl355);
  adxl355.configure(ADXL355_RANGE, ADXL355_RANGE_8G);

  etimer_set(&et, CLOCK_SECOND);
  
  while(1) {
	capture_send(NULL);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  }
  /*
  while(1) {
    PROCESS_YIELD();
	 
   	if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {	
		  data_fetch_periodic();
		  etimer_set(&et, 0);
       }
  	 } 
  }*/
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_rx_process, ev, data)
{
  PROCESS_BEGIN();

  uint8_t *pbuf;
  uint16_t plen;

  uart1_set_input(apros_serial_line_input_byte);
  
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == apros_serial_line_event_message && data != NULL);
    pbuf = ((sl_data_t*)data)->buffer;
    plen = ((sl_data_t*)data)->length;

	static uint8_t pkt_buf[APROS_PKT_MAX_SIZE], pkt_len = 0;
	static uint8_t pl[APROS_PL_MAX_SIZE], pl_len;		
	
	if(pkt_len + plen <= APROS_PKT_MAX_SIZE) {
      memcpy(pkt_buf + pkt_len, pbuf, plen);
      pkt_len += plen;
    }
    else {
      memcpy(pkt_buf, pbuf, plen);
      pkt_len = plen;
    }
	if(pkt_len >= APROS_PKT_MIN_SIZE){ 
		if(apros_node_pkt_parser(pbuf, plen, pl, &pl_len) == 1 && !flag_processing_msg){
			consume_serial_payload(pl, pl_len);
			PRINTF("CONSUME PKT\n");
		}else{
			PRINTF("INVALID FORMAT\n");
		}
	}
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */ 

