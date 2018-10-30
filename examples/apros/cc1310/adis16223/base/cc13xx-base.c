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

#include "ti-lib.h"
#include "lib/crc16.h"

#include "cc13xx-base.h"

#include <stdio.h>
#include <stdint.h>

#include "ti-lib.h"

/*---------------------------------------------------------------------------*/
static struct etimer et_live;
static struct ctimer ct_err_id, ct_err_timeout, ct_err_malform;
/*---------------------------------------------------------------------------*/
unsigned char node_mac[8];

uint8_t sent_f_type;

uint8_t flag_can_send; //0 == block, 1 == can send; 

uint8_t parsing_packet_serial[128];
uint8_t parsing_length_serial;
uint8_t parsing_packet_sun[128];
uint8_t parsing_length_sun;

PROCESS(cc13xx_demo_process, "cc13xx demo process");
PROCESS(uart_rx_process, "uart rx process");
AUTOSTART_PROCESSES(&cc13xx_demo_process, &uart_rx_process);
/*---------------------------------------------------------------------------*/
/* Bootup State */
#define BU_FLASH_ERROR   0
#define BU_NORMAL        1
#define BU_FACTORY       2

/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET   0
#define CONFIG_MAGIC          0xCC131002
/*---------------------------------------------------------------------------*/
typedef struct mems_base_config {
  uint8_t channel;
  uint32_t magic;
} mems_base_config_t;

mems_base_config_t mems_base_config;
static int8_t generate_serial(uint8_t m_type, uint8_t* frm_id, uint8_t f_type, uint8_t f_vlength, void* f_value);
static uint8_t check_ack(uint8_t* pkt, uint8_t plen);
/*---------------------------------------------------------------------------*/
int
load_config(void)
{
  mems_base_config_t tmp_config;

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
    memcpy(&mems_base_config, &tmp_config, sizeof(mems_base_config));
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

  rv = ext_flash_erase(CONFIG_FLASH_OFFSET, sizeof(mems_base_config));

  if(!rv) {
    return BU_FLASH_ERROR;
  } else {
    mems_base_config.magic = CONFIG_MAGIC;

    rv = ext_flash_write(CONFIG_FLASH_OFFSET, sizeof(mems_base_config), (uint8_t *)&mems_base_config);
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
    mems_base_config.channel = 1;
}
/*---------------------------------------------------------------------------*/
void
to_busy(){
	ti_lib_gpio_set_dio(BOARD_IOID_UART_CBUS);//High : Indicates Busy State on the base
	leds_on(LEDS_YELLOW);

}
/*---------------------------------------------------------------------------*/
void 
to_idle(){
	leds_off(LEDS_YELLOW);
	ti_lib_gpio_clear_dio(BOARD_IOID_UART_CBUS);//LOW : Indicates IDLE State on the base

}
/*---------------------------------------------------------------------------*/
void apply_channel(void){
//	radio_value_t val = 0;

	NETSTACK_RADIO.set_value(RADIO_PARAM_CHANNEL, mems_base_config.channel);
	
//	NETSTACK_RADIO.get_value(RADIO_PARAM_CHANNEL, &val);
//  	printf(" RF: Channel %d\n", val);
}
/*---------------------------------------------------------------------------*/
void set_basech(uint8_t channel){
	uint8_t value = RESPONSE_SUCCESS;
	mems_base_config.channel = channel;
	save_config();
	apply_channel();
	generate_serial(MSG_TYPE_CONTROL, node_mac, FRM_TYPE_SET_BASECH, 1, (void*)&value);
	to_idle();
}
/*---------------------------------------------------------------------------*/
void get_basech(){
	uint8_t value = mems_base_config.channel;
	generate_serial(MSG_TYPE_CONTROL, node_mac, FRM_TYPE_GET_BASECH, 1, (void*)&value);
	to_idle();
}
/*---------------------------------------------------------------------------*/
void get_baseid(){
	uint8_t value = RESPONSE_SUCCESS;
	generate_serial(MSG_TYPE_CONTROL, node_mac, FRM_TYPE_GET_BASEID, 1, (void*)&value);
	to_idle();
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
  ieee_addr_cpy_to(node_mac, 8);
 
  apply_channel();
}
/*---------------------------------------------------------------------------*/
static void
recv_uc(struct unicast_conn *c, const linkaddr_t *from)
{
	static uint16_t packetlength;
	static uint8_t *packetbuf_pntr;

	packetlength = packetbuf_datalen();
	packetbuf_pntr = packetbuf_dataptr();
	
	if((mems_packet_parser(packetbuf_pntr, packetlength, parsing_packet_sun, &parsing_length_sun)) == 1) { // control check
			if(check_ack(parsing_packet_sun, parsing_length_sun)){
				to_idle();
				ctimer_stop(&ct_err_timeout);
			}else{
				ctimer_restart(&ct_err_timeout);
			}
			apros_serial_send(packetbuf_pntr, packetlength);
			memset(parsing_packet_sun, 0, parsing_length_sun);
			parsing_length_sun = 0 ;
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
static int8_t
generate_serial(uint8_t m_type, uint8_t* frm_id, uint8_t f_type, uint8_t f_vlength, void* f_value){
	static uint8_t pkt[50];
	uint8_t total_len = f_vlength + 18; 
	// STX[1] + T_LENGTH[1] + MSG_TYPE[1] + ID(TLV)[10] + CRC[2] + ETX[1] + f_type[1] + f_vlength[1]
	uint8_t success = 0;
	
	if(make_initial_packet(pkt, total_len, m_type)){
		if(add_TLV(pkt, total_len, FRM_TYPE_ID, 8, frm_id)){
			if(add_TLV(pkt, total_len, f_type, f_vlength, f_value)){
				if(make_final_packet(pkt, total_len)){
					apros_serial_send(pkt, total_len);
					success = 1;
				}
			}
		}
	}
	return success;
}
/*---------------------------------------------------------------------------*/
static uint8_t set_dest(uint8_t* pkt, uint8_t plen, uint8_t* signal, uint8_t* dest_mac){
	uint8_t type, length;
	uint8_t *value;

	type = pkt[0];
	length = pkt[1];
	value = pkt+2;
	
	if(type == FRM_TYPE_ID && length == 8){
		if(frame802154_is_broadcast_addr(FRAME802154_LONGADDRMODE, value)){
			*signal = BR_ADDR;
		}else{
			*signal = UNI_ADDR;
		}
		memcpy(dest_mac, value, 8);
		return 1;
	}else{
		return 0;
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
void send_err(void* ptr){
	generate_serial(MSG_TYPE_CONTROL, node_mac, FRM_TYPE_ERROR, 1, ptr);
	to_idle();
	if(((uint8_t*)ptr)[0] == ERROR_TYPE_TIMEOUT){
//		unicast_close(&uc);
//		unicast_open(&uc, 146, &unicast_callbacks);
	}
}
/*---------------------------------------------------------------------------*/
static uint8_t check_ack(uint8_t* pkt, uint8_t plen){
	static uint8_t i;
	static uint8_t type = 0;
	static uint8_t length = 0;

	i = 10;
	while(plen > i){
		type = pkt[i];
		length = pkt[i+1];
		
		if(sent_f_type == type){
			sent_f_type = 0;
			return 1;
		}
		i += length + 2; //length(1) + type(1) + value
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
static uint8_t consume_packet(uint8_t* pkt, uint8_t plen){
	static uint8_t i = 0;
	static uint8_t type = 0;
	static uint8_t length = 0;
	static uint8_t* value;
	static uint8_t signal_type;
	static uint8_t dest_mac[8];
	
	if(set_dest(pkt,plen, &signal_type, dest_mac)){
		flag_can_send = 1;//if the base recognizes a certain destination, the packet will be sent.
		i = 10;
	}else{
		static uint8_t er_type = ERROR_TYPE_MALFORM;
		flag_can_send = 0;
		ctimer_set(&ct_err_malform, CLOCK_SECOND, send_err, &er_type);
		return 0;
	}
	while(plen > i){
		//TLV
		type = pkt[i];
		length = pkt[i+1];
		if(length){
			value = pkt+i+2;
		}else{
			value = NULL;
		}
		if(flag_can_send){
			switch(type){
				
				//Base Command
				case FRM_TYPE_SET_BASECH:
					if(signal_type == UNI_ADDR){
						if(is_mine(pkt,plen)){
							set_basech(value[0]);
						}else{
							static uint8_t er_type = ERROR_TYPE_ID;
							ctimer_set(&ct_err_id, CLOCK_SECOND, send_err, &er_type);
						}
					}
					break;
				case FRM_TYPE_GET_BASECH:
					if(signal_type == UNI_ADDR){
						if(is_mine(pkt,plen)){
							get_basech();
						}else{
							static uint8_t er_type = ERROR_TYPE_ID;
							ctimer_set(&ct_err_id, CLOCK_SECOND, send_err, &er_type);
						}
					}
					break;
				case FRM_TYPE_GET_BASEID:
					if(signal_type == BR_ADDR){
						get_baseid();
					}
					break;
					
				//Node Command
				case FRM_TYPE_DVC_RESET:
				case FRM_TYPE_FTRY_RESET:
					if(signal_type == BR_ADDR || signal_type == UNI_ADDR){
						static uint8_t er_type = ERROR_TYPE_TIMEOUT;
						generate_wireless((linkaddr_t*)(dest_mac+6), MSG_TYPE_CONTROL, dest_mac, type, length, value, signal_type);
						ctimer_set(&ct_err_timeout, 5*CLOCK_SECOND, send_err, &er_type);
						sent_f_type = type; //To Utilize to check the packet of the type when this base receives an ack.
					}
					break;
				
				case FRM_TYPE_REGISTER_NODE:
				case FRM_TYPE_SET_NODECH:
				case FRM_TYPE_GET_NODECH:
				case FRM_TYPE_DATA_FETCH:
				case FRM_TYPE_RETT:
					if(signal_type == UNI_ADDR){
						static uint8_t er_type = ERROR_TYPE_TIMEOUT;
						generate_wireless((linkaddr_t*)(dest_mac+6), MSG_TYPE_CONTROL, dest_mac, type, length, value, signal_type);
						ctimer_set(&ct_err_timeout, 5*CLOCK_SECOND, send_err, &er_type);
						sent_f_type = type;
					}
					break;
				
				case FRM_TYPE_CAPT_REQUEST: 
					if(signal_type == BR_ADDR){
						static uint8_t er_type = ERROR_TYPE_TIMEOUT;
						generate_wireless((linkaddr_t*)(dest_mac+6), MSG_TYPE_CONTROL, dest_mac, type, length, value, signal_type);
						ctimer_set(&ct_err_timeout, 5*CLOCK_SECOND, send_err, &er_type);
						sent_f_type = type;
					}
					break;
				default:
					{
						static uint8_t er_type = ERROR_TYPE_MALFORM;
						ctimer_set(&ct_err_malform, CLOCK_SECOND, send_err, &er_type);
					}
			}
		}
		i += length + 2; //length(1) + type(1) + value
	}
	return 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc13xx_demo_process, ev, data)
{
  PROCESS_BEGIN();

  etimer_set(&et_live, CLOCK_SECOND);
  
  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et_live) {
		leds_toggle(LEDS_RED);
        etimer_set(&et_live, CLOCK_SECOND);
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_rx_process, ev, data)
{
  PROCESS_EXITHANDLER(unicast_close(&uc);broadcast_close(&broadcast);)

  PROCESS_BEGIN();
  uint8_t *pbuf;
  uint16_t plen;
  
  unicast_open(&uc, 146, &unicast_callbacks);
  broadcast_open(&broadcast, 129, &broadcast_callbacks);

  mems_init();
  
  uart1_set_input(apros_serial_line_input_byte);
  apros_serial_line_init();

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == apros_serial_line_event_message && data != NULL);
	pbuf = ((sl_data_t*)data)->buffer;
    plen = ((sl_data_t*)data)->length;
	
	to_busy();
	if((mems_packet_parser(pbuf, plen, parsing_packet_serial, &parsing_length_serial) ) == 1) { // control check
		if(parsing_length_serial > 0 ){
        	consume_packet(parsing_packet_serial, parsing_length_serial);
		}
    }else{
		static uint8_t er_type = ERROR_TYPE_MALFORM;
		ctimer_set(&ct_err_malform, CLOCK_SECOND, send_err, &er_type);
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
