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
#include "net/netstack.h"

#include "button-sensor.h"
#include "adis16223-sensor.h"

#include "ti-lib.h"

#include "cc13xx-node.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
/** Macro Variable **/
#define NUM_AGGR_SIZE 8
#define MAX_ADIS16223_SAMPLE 1024

/* Bootup State */
#define BU_FLASH_ERROR   0
#define BU_NORMAL        1
#define BU_FACTORY       2

/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET   0
#define CONFIG_MAGIC          0xCC131002
/*---------------------------------------------------------------------------*/
/** Global Variables for this file **/
static struct etimer et;

static struct ctimer ct_cp_buf, ct_set_avg_cnt, ct_dev_reset, ct_factory_reset, ct_set_baseid, ct_set_nodech, ct_data_fetch, ct_rett; 
static struct ctimer ct_command_start, ct_command_end;

static clock_time_t capture_time;
static clock_time_t backoff_time;

static uint8_t node_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};

static uint8_t parsing_packet[100];
static uint8_t parsing_length;

static uint8_t flag_can_recv = 0;

static uint8_t flag_processing = 0; //if the mcu is doing something, the node do not consume any packet at that time.
static uint8_t flag_registered = 0;

static uint8_t flag_startled_on;
static uint16_t i, i_data_fetch, i_rett;

typedef struct mems_node_config {
 	uint8_t base_id[8];
  	uint8_t channel;
	uint8_t mode; // passive or active, passive : 0, active : 1
	uint8_t active_mode_cycle; // in active mode, transmission cycle.
	uint8_t avg_cnt; //0~10
  	uint32_t magic;
} mems_node_config_t;

static mems_node_config_t mems_node_config;

typedef struct response_msg{
    uint8_t f_type;
	uint8_t msg_len;
	uint8_t* msg;
} response_msg_t;

static response_msg_t rsp_msg;
/*---------------------------------------------------------------------------*/
PROCESS(capture_process, "capture process");
AUTOSTART_PROCESSES(&capture_process);
/*---------------------------------------------------------------------------*/
static void uni_adis16223read_send(uint16_t index, uint8_t);
static uint8_t send_unicast_to_dest(linkaddr_t* dest_id, uint8_t* packet);
static uint8_t send_broadcast(uint8_t* packet);
static void copy_to_buff(void* ptr);
/*---------------------------------------------------------------------------*/
static inline clock_time_t calc_capturetime(uint8_t avg_cnt){
	uint8_t i;
	uint16_t ret;
/*
 * Capture Time with No Flash : tc = 0.014 + 1/70700 * 1024 * 2^AVG_CNT
 * when AVG_CNT=0, tc = 0.028484 s
 */
	ret = CLOCK_SECOND>>8; //if AVG_CNT == 0, ret == 0.015625 that is the approximate value to 1/70700 * 1024 == 0.014484
	
	for(i = 0 ; i < avg_cnt ; i++){
		ret = (ret<<1);
	}
	ret += CLOCK_SECOND/64; //add 0.015625 that is approximate value to 0.014(copy time)
	return (clock_time_t)ret;
}
/*---------------------------------------------------------------------------*/
static inline clock_time_t calc_backoff(uint8_t mac_addr[]){
	uint16_t base_value = ((uint16_t)mac_addr[6] << 8) + mac_addr[7];

	return (clock_time_t)base_value%128;
}
/*---------------------------------------------------------------------------*/
int
load_config(void)
{
  mems_node_config_t tmp_config;

  int rv = ext_flash_open();

  if(!rv) {
    ext_flash_close();
    return BU_FLASH_ERROR;
  }

  rv = ext_flash_read(CONFIG_FLASH_OFFSET, sizeof(tmp_config), (uint8_t *)&tmp_config);
  ext_flash_close();

  if(!rv) {
    return BU_FLASH_ERROR;
  }

  if(tmp_config.magic == CONFIG_MAGIC) {
    memcpy(&mems_node_config, &tmp_config, sizeof(mems_node_config));
    return BU_NORMAL;
  } else {
    return BU_FACTORY;
  }
}
/*---------------------------------------------------------------------------*/
int
save_config(void)
{
  int rv = ext_flash_open();

  if(!rv) {
    ext_flash_close();
    return BU_FLASH_ERROR;
  }

  rv = ext_flash_erase(CONFIG_FLASH_OFFSET, sizeof(mems_node_config));

  if(!rv) {
    return BU_FLASH_ERROR;
  } else {
    mems_node_config.magic = CONFIG_MAGIC;

    rv = ext_flash_write(CONFIG_FLASH_OFFSET, sizeof(mems_node_config), (uint8_t *)&mems_node_config);
    if(!rv) {
      return BU_FLASH_ERROR;
    }
  }

  ext_flash_close();
  return BU_NORMAL;
}
/*---------------------------------------------------------------------------*/
void
mems_config_init(void)
{
  memset(mems_node_config.base_id, 0 , 8);
  mems_node_config.channel = 1;
}
/*---------------------------------------------------------------------------*/
void apply_channel(uint8_t channel){
	NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, channel);
}
/*---------------------------------------------------------------------------*/
void
mems_init(void)
{
	int bu;

	mems_config_init();
	  
	bu = load_config();
	if(bu == BU_FACTORY) {
	  // save default
	  	save_config();
	}
	if(linkaddr_cmp((linkaddr_t*)(mems_node_config.base_id+6),&linkaddr_null)){
		flag_registered = 0;
	}else{
		flag_registered = 1;
	}
	  
	ieee_addr_cpy_to(node_mac, 8);
	backoff_time = calc_backoff(node_mac);
	capture_time = calc_capturetime(mems_node_config.avg_cnt);
	apply_channel(mems_node_config.channel);
}
/*---------------------------------------------------------------------------*/
static int8_t
generate_wireless(linkaddr_t* dest_addr, uint8_t m_type, uint8_t* frm_id, uint8_t f_type, uint8_t f_vlength, void* f_value, uint8_t signal_flag){
	static uint8_t pkt[50];
	uint8_t total_len = f_vlength + 18; 
	// STX[1] + T_LENGTH[1] + MSG_TYPE[1] + ID(TLV)[10] + CRC[2] + ETX[1] + f_type[1] + f_vlength[1]
	uint8_t success = 0;

	switch(signal_flag){
			case BR_ADDR:
			case UNI_ADDR:
			break;
			default:
				return -1;
	}
	if(make_initial_packet(pkt, total_len, m_type)){
		if(add_TLV(pkt, total_len, FRM_TYPE_ID, 8, frm_id)){
			if(add_TLV(pkt, total_len, f_type, f_vlength, f_value)){
				if(make_final_packet(pkt, total_len)){
					if(signal_flag == BR_ADDR){
						send_broadcast(pkt);
						success = 1;
					}else if(signal_flag == UNI_ADDR){
						send_unicast_to_dest(dest_addr, pkt);
						success = 1;
					}
				}
			}
		}
	}
	return success;
}
/*---------------------------------------------------------------------------*/
static void start_command(void* ptr){
    flag_processing = 1;
	leds_on(*((uint8_t*)ptr));
}
/*---------------------------------------------------------------------------*/
static void end_command(void* ptr){
	response_msg_t* rsp = (response_msg_t*)ptr;

	generate_wireless((linkaddr_t*)(mems_node_config.base_id+6), MSG_TYPE_CONTROL, node_mac, rsp->f_type, rsp->msg_len, rsp->msg, UNI_ADDR);
	leds_off(LEDS_ALL);
	flag_processing = 0;
}
/*---------------------------------------------------------------------------*/
static void dev_reset(void* ptr){
	watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
static void factory_reset(void* ptr){
	mems_config_init();
	save_config();
	dev_reset(NULL);
}
/*---------------------------------------------------------------------------*/
static void set_baseid(void* ptr){
	memcpy(mems_node_config.base_id, (uint8_t*)ptr, 8);
	save_config();
	flag_registered = 1;
}
/*---------------------------------------------------------------------------*/
static void set_nodech(void* ptr){
	mems_node_config.channel = *((uint8_t*)ptr);
	save_config();
	apply_channel();
}
/*---------------------------------------------------------------------------*/
static void data_fetch(void *ptr){
	uni_adis16223read_send(i_data_fetch++, ((uint8_t*)ptr)[0]);

	if(i_data_fetch == MAX_NUM_SAMPLE){
		static uint8_t value;
		rsp_msg.f_type = FRM_TYPE_DATA_FETCH;
		rsp_msg.msg_len = 1;
		rsp_msg.msg = &value;
		rsp_msg.msg[0] = RESPONSE_SUCCESS;
		
		adis16223_capture_offset_reset();
		i_data_fetch = 0;

		ctimer_set(&ct_command_end, CLOCK_SECOND, end_command, &rsp_msg);
	}else{
		ctimer_set(&ct_data_fetch, CLOCK_SECOND>>7, data_fetch, ptr);
	}
}
/*---------------------------------------------------------------------------*/
static void copy_to_buff(void* ptr){
	static uint16_t i = 0;
	uint8_t input = *((uint8_t*)ptr); 

	switch(input){
		case 1 : // x
			{
				adis16223_read_8(ADIS16223_CAPT_BUFFX, x_value[i++]);
				if( i == 1024 ){
					static uint8_t next = 2;
					i = 0;
					ctimer_set(&ct_cp_buf, 0, copy_to_buff, &next);
				}else{
					ctimer_set(&ct_cp_buf, 0, copy_to_buff, ptr);
				}
			}
			break;
		case 2 : // y
			{
				adis16223_read_8(ADIS16223_CAPT_BUFFY, y_value[i++]);
				if( i == 1024 ){
					static uint8_t next = 3;
					i = 0;
					ctimer_set(&ct_cp_buf, 0, copy_to_buff, &next);
				}else{
					ctimer_set(&ct_cp_buf, 0, copy_to_buff, ptr);
				}
			}
			break;
		case 3 : // z
			{
				adis16223_read_8(ADIS16223_CAPT_BUFFZ, z_value[i++]);
				if( i == 1024 ){
					i = 0;
				}else{
					ctimer_set(&ct_cp_buf, 0, copy_to_buff, ptr);
				}
			}
			break;
	}
}
/*---------------------------------------------------------------------------*/
static void rett(void *ptr){
	static uint8_t aggr_index = 0;

	uni_adis16223read_send(i_rett++, ((uint8_t*)ptr)[2]);
	aggr_index++;

	if(aggr_index == NUM_AGGR_SIZE){
		static uint8_t value;
		rsp_msg.f_type = FRM_TYPE_RETT;
		rsp_msg.msg_len = 1;
		rsp_msg.msg = &value;
		rsp_msg.msg[0] = RESPONSE_SUCCESS;
		
		adis16223_capture_offset_reset();
		i_rett = 0;
		aggr_index = 0;

		ctimer_set(&ct_command_end, CLOCK_SECOND, end_command, &rsp_msg);
	}else{
		ctimer_set(&ct_rett, CLOCK_SECOND>>7, rett, ptr);
	}
}
/*---------------------------------------------------------------------------*/
static void command_dev_reset(){
	static uint8_t led = LEDS_WHITE;
	static uint8_t value;
	rsp_msg.f_type = FRM_TYPE_DVC_RESET;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = RESPONSE_SUCCESS;

	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7, start_command, &led);

	ctimer_set(&ct_dev_reset, CLOCK_SECOND+backoff_time+2, dev_reset, NULL);
	
	ctimer_set(&ct_command_end, CLOCK_SECOND+backoff_time, end_command, &rsp_msg);
}
/*---------------------------------------------------------------------------*/
static void command_factory_reset(){
	static uint8_t led = LEDS_RED;
	static uint8_t value;
	
	rsp_msg.f_type = FRM_TYPE_FTRY_RESET;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = RESPONSE_SUCCESS;
	
	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7 , start_command, &led);

	ctimer_set(&ct_factory_reset, CLOCK_SECOND+backoff_time+2, factory_reset, NULL);

	ctimer_set(&ct_command_end, CLOCK_SECOND+backoff_time, end_command, &rsp_msg);
}
/*---------------------------------------------------------------------------*/
static void command_register_node(void* ptr){
	static uint8_t led = LEDS_MAGENTA;
	static uint8_t value;
	
	rsp_msg.f_type = FRM_TYPE_REGISTER_NODE;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = RESPONSE_SUCCESS;

	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7 , start_command, &led);

	ctimer_set(&ct_set_baseid, CLOCK_SECOND, set_baseid, ptr);

	ctimer_set(&ct_command_end, CLOCK_SECOND*2, end_command, &rsp_msg);
}
/*---------------------------------------------------------------------------*/
static void command_set_nodech(void* ptr){
	static uint8_t led = LEDS_YELLOW;
	static uint8_t value;

	rsp_msg.f_type = FRM_TYPE_SET_NODECH;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = RESPONSE_SUCCESS;
	
	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7 , start_command, &led);

	ctimer_set(&ct_set_nodech, CLOCK_SECOND*2, set_nodech, ptr);

	ctimer_set(&ct_command_end, CLOCK_SECOND, end_command, &rsp_msg);
}
/*---------------------------------------------------------------------------*/
static void command_get_nodech(){
	static uint8_t led = 0;
	static uint8_t value;

	rsp_msg.f_type = FRM_TYPE_GET_NODECH;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = mems_node_config.channel;
	
	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7, start_command, &led);

	ctimer_set(&ct_command_end, CLOCK_SECOND, end_command, &rsp_msg);
}
/*---------------------------------------------------------------------------*/
static void command_capture_req(){
	static uint8_t led = LEDS_GREEN;
	static uint8_t value;

	rsp_msg.f_type = FRM_TYPE_CAPT_REQUEST;
	rsp_msg.msg_len = 1;
	rsp_msg.msg = &value;
	rsp_msg.msg[0] = RESPONSE_SUCCESS;
	
	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7, start_command, &led);

	adis16223_capture_trigger();
	ctimer_set(&ct_cp_buf, capture_time, copy_to_buff, &value);
	//copy the data to the cc1310 buffer from the adis16223 buffer after the capture time
	
	ctimer_set(&ct_command_end, capture_time+backoff_time, end_command, &rsp_msg);
}

/*---------------------------------------------------------------------------*/
static void command_data_fetch(void* ptr){
	static uint8_t led = LEDS_BLUE;
	
	ctimer_set(&ct_command_start, CLOCK_SECOND >> 7, start_command, &led);
	
	ctimer_set(&ct_data_fetch, CLOCK_SECOND, data_fetch, ptr); //TODO:: changeable time according to AVG setting.
}
/*---------------------------------------------------------------------------*/
static void command_rett(void *ptr){
	static uint8_t led = LEDS_BLUE;
	
	i_rett = (((uint8_t*)ptr)[0]) + (((uint8_t*)ptr)[1]<<8);
	i_rett *= 8 ;

	if(i_rett > MAX_NUM_SAMPLE-NUM_AGGR_SIZE){
			i_rett = 0;
			//TODO :: send error 
			return;
	}else{
		ctimer_set(&ct_command_start, CLOCK_SECOND >> 7, start_command, &led);
		ctimer_set(&ct_rett, CLOCK_SECOND>>6, rett, ptr);
	}
}
/*---------------------------------------------------------------------------*/
static uint8_t is_mine(uint8_t* pkt, uint8_t plen){
	uint8_t type, length;
	uint8_t *value;

	type = pkt[0];
	length = pkt[1];
	value = pkt+2;
	
	if(type == FRM_TYPE_ID && length == 8){
		if(!memcmp(node_mac, value, 8) || frame802154_is_broadcast_addr(FRAME802154_LONGADDRMODE, value)){
			return 1;
		}else{
			return 0;
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t consume_packet(uint8_t* pkt, uint8_t plen){
	uint8_t i = 0;
	uint8_t type = 0;
	uint8_t length = 0;
	uint8_t* value = NULL;
	
	if(is_mine(pkt,plen)){
		flag_can_recv = 1;
		i = 10;
	}else{
		flag_can_recv = 0;
		//ctimer_set(&ct_error, CLOCK_SECOND, callback_error_start, NULL);
		return -1;
	}
	while(plen > i){
		//TLV
		type = pkt[i];
		length = pkt[i+1];
		if(length){
			value = pkt+i+2;
		}
		if(flag_can_recv && !flag_processing){
			switch(type){
				case FRM_TYPE_REGISTER_NODE:
					command_register_node(value);
					break;
				case FRM_TYPE_DVC_RESET:
					if(flag_registered){
						command_dev_reset();
					}
					break;
				case FRM_TYPE_FTRY_RESET:
					if(flag_registered){
						command_factory_reset();
					}
					break;
				case FRM_TYPE_SET_NODECH:
					if(flag_registered){
						command_set_nodech(value);
					}
					break;
				case FRM_TYPE_GET_NODECH:
					if(flag_registered){
						command_get_nodech();
					}
					break;
				case FRM_TYPE_CAPT_REQUEST:	
					if(flag_registered){
						command_capture_req();
					}
					break;
				case FRM_TYPE_DATA_FETCH:
					if(flag_registered){
						command_data_fetch(value);
					}
					break;
				case FRM_TYPE_RETT:
					if(flag_registered){
						command_rett(value); //value[0]~[1] : index, [2]: shaft
					}
					break;
			}
		}
		i += length + 2; //length(1) + type(1) + value
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
	if(mems_packet_parser((uint8_t*)packetbuf_dataptr(), packetbuf_datalen(), parsing_packet, &parsing_length) == 1 
		&& flag_processing == 0){
		consume_packet(parsing_packet, parsing_length);
	}
}
/*---------------------------------------------------------------------------*/
static void
sent_uc(struct unicast_conn *c, int status, int num_tx)
{
  const linkaddr_t *dest = packetbuf_addr(PACKETBUF_ADDR_RECEIVER);
  if(linkaddr_cmp(dest, &linkaddr_null)) {
    return;
  }
}
/*---------------------------------------------------------------------------*/
static void
recv_broadcast(struct broadcast_conn *c, const linkaddr_t *from)
{
  if(mems_packet_parser((uint8_t*)packetbuf_dataptr(), packetbuf_datalen(), parsing_packet, &parsing_length) == 1){
	 consume_packet(parsing_packet, parsing_length);
  }
}
/*---------------------------------------------------------------------------*/
static const struct unicast_callbacks unicast_callbacks = {recv_uc, sent_uc};
static struct unicast_conn uc;
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_callbacks = {recv_broadcast};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
static uint8_t send_unicast_to_dest(linkaddr_t* dest_id, uint8_t* packet){
	if(linkaddr_node_addr.u8[0] == dest_id->u8[0] &&
	   linkaddr_node_addr.u8[1] == dest_id->u8[1]) {
		;
	}else{
	  packetbuf_copyfrom(packet, get_finalpkt_length(packet));
	  unicast_send(&uc, dest_id);
	  return 1;
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t send_broadcast(uint8_t* packet){
	packetbuf_copyfrom(packet, get_finalpkt_length(packet));
	broadcast_send(&broadcast);
	return 1;
}
/*---------------------------------------------------------------------------*/
static void
uni_adis16223read_send(uint16_t index, uint8_t shaft)
{
	static uint8_t pkt[MAX_FETCH_PKT_SIZE]; // STX(1) LENGTH(1) MSG_TYPE(1) TYPE(1) LENGTH(1) ID(8) TYPE(1) LENGTH(1) INDEX(2) TYPE(1) LENGTH(1) VALUE(48) CRC(2) ETX(1) == 70
	static uint8_t value[48];
	static uint8_t value_index = 0;
	static uint8_t packet_index[2];
	static uint8_t chunk_size;
	static uint8_t tmp;
	static uint8_t f_type;
	
	if(value_index == 0){
		tmp = index/8;  //xyz * 8 
		packet_index[0] = tmp & 0xff;
		packet_index[1] = (tmp >> 8) & 0xff;
			
		if(make_initial_packet(pkt, MAX_FETCH_PKT_SIZE, MSG_TYPE_RAWDATA)){
			if(add_TLV(pkt, MAX_FETCH_PKT_SIZE, FRM_TYPE_ID, LENGTH_DATA_ID, node_mac)){
				if(add_TLV(pkt, MAX_FETCH_PKT_SIZE, FRM_TYPE_ACCEL_INDEX, LENGTH_ACCEL_INDEX, packet_index)){

				}
			}
		}
	}
	
	switch(shaft){
			case 3:
				f_type  =  FRM_TYPE_ACCEL_TRI;
				value[value_index]   = x_value[index][0][1];
				value[value_index+1] = x_value[index][0][0];
				value[value_index+2] = y_value[index][0][1];
				value[value_index+3] = y_value[index][0][0];
				value[value_index+4] = z_value[index][0][1];
				value[value_index+5] =
 z_value[index][0][0];
				break;
			case 2:
				f_type 	=  FRM_TYPE_ACCEL_DOB;
				value[value_index]   = x_value[index][0][1];
				value[value_index+1] = x_value[index][0][0];
				value[value_index+2] = y_value[index][0][1];
				value[value_index+3] = y_value[index][0][0];
				break;
			case 1:	
				f_type	=  FRM_TYPE_ACCEL_SIN;
				value[value_index+4] = z_value[index][0][1];
				value[value_index+5] =
 z_value[index][0][0];
				break;
				
	}

	chunk_size = shaft*2; //ADIS16223 data unit size is 16bit
	value_index += chunk_size; // if it is triple shaft, chunk size must be 6 bytes.

	if(value_index == chunk_size*NUM_AGGR_SIZE){
		if(add_TLV(pkt, MAX_FETCH_PKT_SIZE, f_type, chunk_size*NUM_AGGR_SIZE, value)){
			if(make_final_packet(pkt, MAX_FETCH_PKT_SIZE)){
				if(send_broadcast(pkt)){
					value_index = 0;
				}
			}
		}
	}
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(capture_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);broadcast_close(&broadcast);)

  PROCESS_BEGIN();

  flag_processing = 1;
  mems_init();
  
  unicast_open(&uc, 146, &unicast_callbacks);
  broadcast_open(&broadcast, 129, &broadcast_callbacks);
  
  adis16223_enable();

  adis16223_capture_configure(
  	(uint8_t)0,
  	ADIS16223_CAPT_CTRL_MODE_MANUAL,
  	(uint8_t)0,
  	(uint8_t)0,
  	(uint8_t)0,
  	(uint8_t)0);

  adis16223_capture_offset_reset();

  adis16223_avg_cnt_configure(ADIS16223_AVG_CNT_D6); //1k under : 6, 1k over : 4
  
  adis16223_dio_configure(
  	ADIS16223_DIO_CTRL_DIO1_POL_H,
  	ADIS16223_DIO_CTRL_DIO1_FUNC_BUSY_IND,
  	ADIS16223_DIO_CTRL_DIO2_POL_H,
  	ADIS16223_DIO_CTRL_DIO2_FUNC_CAPT_TR);
  
  leds_on(LEDS_WHITE);
  etimer_set(&et, CLOCK_SECOND);

  flag_processing = 0;
  
  while(1) {
    PROCESS_YIELD();
	if(ev == PROCESS_EVENT_TIMER) {
		if(flag_registered){
			leds_off(LEDS_WHITE);
		}else{
			leds_toggle(LEDS_WHITE);
			etimer_set(&et, CLOCK_SECOND);
		}

	}else if(ev == sensors_event) {
      if(data == &button_user_sensor) {
        if((&button_user_sensor)->value(BUTTON_SENSOR_VALUE_DURATION) > CLOCK_SECOND * 3) {
          ctimer_set(&ct_dev_reset, CLOCK_SECOND >> 3, dev_reset, NULL);
        }
      }
    }
  }
  PROCESS_END();
}

/**
 * @}
 * @}
 * @}
 */

