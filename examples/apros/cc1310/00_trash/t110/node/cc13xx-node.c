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

#include "button-sensor.h"
#include "sht-sensor.h"

#include "ti-lib.h"

#include "cc13xx-node.h"

#include <stdio.h>
#include <stdint.h>

/*---------------------------------------------------------------------------*/
static struct etimer et;
static struct stimer st;
struct ctimer c_register_timer, c_change_timer, c_reset_timer, c_boundary_set_timer, c_boundary_get_timer;
struct ctimer c_mcdl_off;

uint8_t node_mac[8] = {0, 0, 0, 0, 0, 0, 0, 0};
linkaddr_t addr;
uint8_t frm_payload[50];
uint8_t parsing_packet[128], parsing_length;

int temperature, humidity, co2_gas;
uint8_t flag_errorled_on = 0; //apros-ksh
uint8_t flag_startled_on = 0; //apros-ksh
uint8_t flag_errorledco2_on = 0; //apros-ksh
uint8_t flag_co2_led = 0;//apros-ksh
uint8_t flag_mcdl_ing = 0;
//static struct etimer errorcheck_et; //apros-ksh
//static struct ctimer errorcheck_ct
static struct ctimer ledoff_ct; //apros-ksh

/* Bootup State */
#define BU_FLASH_ERROR   0
#define BU_NORMAL        1
#define BU_FACTORY       2

/* Saved settings on flash: store, offset, magic */
#define CONFIG_FLASH_OFFSET   0
#define CONFIG_MAGIC          0xCC131001
/*---------------------------------------------------------------------------*/
typedef struct co2_led_config { //Configuration for co2 sensor with led model
  uint16_t boundary_end1; //led_boundary
  uint16_t boundary_end2;
  uint16_t boundary_end3;
  uint16_t boundary_end4;  //0~end1 : blue, end1~end2 : green, end2~end3 : yellow, end3~end4 : magenta, end4~ : red
  uint16_t transmit_period; //5~0x2A30
  uint32_t magic;
} co2_led_config_t;

co2_led_config_t co2_led_config;

/*---------------------------------------------------------------------------*/
PROCESS(cc13xx_demo_process, "cc13xx demo process");
PROCESS(uart_rx_process, "uart rx process");
//PROCESS(apros_co2_check_process, "CO2 Dev Check");//apros-ksh
	

AUTOSTART_PROCESSES(&cc13xx_demo_process, &uart_rx_process);
/*---------------------------------------------------------------------------*/
static uint8_t consume_packet(uint8_t* pkt, uint8_t plen);
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
  if(mems_packet_parser((uint8_t*)packetbuf_dataptr(), packetbuf_datalen(), parsing_packet, &parsing_length) == 1){
	 consume_packet(parsing_packet, parsing_length);
  }
}
/*---------------------------------------------------------------------------*/
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static struct broadcast_conn broadcast;
/*---------------------------------------------------------------------------*/
void
send_packet(uint8_t m_type, const uint8_t* payload, uint16_t payload_length)
{
  uint8_t making_packet[100];
  uint8_t packet_length;

  packet_length = make_packet(m_type, payload, payload_length, making_packet);

  //apros_serial_send(making_packet, packet_length);

  packetbuf_copyfrom(making_packet, packet_length);
  broadcast_send(&broadcast);
}
/*---------------------------------------------------------------------------*/
void
callback_device_reset(void *ptr)
{
  watchdog_reboot();
}
/*---------------------------------------------------------------------------*/
int
load_config(void)
{
  co2_led_config_t tmp_config;

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
    memcpy(&co2_led_config, &tmp_config, sizeof(co2_led_config));
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

  rv = ext_flash_erase(CONFIG_FLASH_OFFSET, sizeof(co2_led_config));

  if(!rv) {
    return BU_FLASH_ERROR;
  } else {
    co2_led_config.magic = CONFIG_MAGIC;

    rv = ext_flash_write(CONFIG_FLASH_OFFSET, sizeof(co2_led_config), (uint8_t *)&co2_led_config);
    if(!rv) {
      return BU_FLASH_ERROR;
    }
  }

  ext_flash_close();
  return BU_NORMAL;
}
/*---------------------------------------------------------------------------*/
void
co2_config_init(void)
{
  co2_led_config.boundary_end1 = 500;
  co2_led_config.boundary_end2 = 1000;
  co2_led_config.boundary_end3 = 1300;
  co2_led_config.boundary_end4 = 1600;
  co2_led_config.transmit_period = 5;
}
/*---------------------------------------------------------------------------*/

static uint8_t send_broadcast(uint8_t* packet){
	packetbuf_copyfrom(packet, get_finalpkt_length(packet));
	broadcast_send(&broadcast);
	return 1;
}
static int8_t
generate_wireless(uint8_t signal_flag, linkaddr_t* dest_addr, uint8_t m_type, uint8_t* frm_id, uint8_t f_type, uint8_t f_vlength, void* f_value){
	static uint8_t pkt[50];
	uint8_t total_len = f_vlength + 18; 
	// STX[1] + T_LENGTH[1] + MSG_TYPE[1] + ID(TLV)[10] + CRC[2] + ETX[1] + f_type[1] + f_vlength[1]
	uint8_t success = 0;
	
//	switch(signal_flag){
//			case SUN_BR:
//			case SUN_UNI:
//			break;
//			default:
//				return -1;
//	}
	if(make_initial_packet(pkt, total_len, m_type)){
		if(add_TLV(pkt, total_len, FRM_TYPE_ID, 8, frm_id)){
			if(add_TLV(pkt, total_len, f_type, f_vlength, f_value)){
				if(make_final_packet(pkt, total_len)){
//					if(signal_flag == SUN_BR){
						send_broadcast(pkt);
						success = 1;
//					}else if(signal_flag == SUN_UNI){
//						send_runicast_to_dest(dest_addr, pkt);
//						success = 1;
//					}
				}
			}
		}
	}
	return success;
}

/*---------------------------------------------------------------------------*/
static void callback_boundary_get(void* ptr){
	static uint8_t pkt[16]; // STX(1) LENGTH(1) MSG_TYPE(1) TYPE(1) LENGTH(1) VALUE(8) CRC(2) ETX(1) == 16
	static uint8_t value[8];
	
	value[0] = co2_led_config.boundary_end1 & 0xff;
	value[1] = co2_led_config.boundary_end1 >> 8;
	value[2] = co2_led_config.boundary_end2 & 0xff;
	value[3] = co2_led_config.boundary_end2 >> 8;
	value[4] = co2_led_config.boundary_end3 & 0xff;
	value[5] = co2_led_config.boundary_end3 >> 8;
	value[6] = co2_led_config.boundary_end4 & 0xff;
	value[7] = co2_led_config.boundary_end4 >> 8;

	if(make_initial_packet(pkt, 16, MSG_TYPE_CONTROL)){
		if(add_TLV(pkt, 16, FRM_TYPE_CO2_OS_RD, 8, value)){
			if(make_final_packet(pkt, 16)){
				send_broadcast(pkt);
			}
		}
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
	uint8_t* value;
	if(is_mine(pkt,plen)){
		while(plen > i){
			//TLV
			type = pkt[i];
			length = pkt[i+1];
			value = pkt+i+2;

			switch(type){
				case FRM_TYPE_RPT_PERIOD:
					{
						uint8_t result;
						uint16_t tmp_p = value[0] + (value[1]<<8);
						if(tmp_p < 1 || tmp_p > 0x2a30){
							result = RESPONSE_FAIL;
						}else{
							co2_led_config.transmit_period = value[0] + (value[1]<<8);
							result = RESPONSE_SUCCESS;
						}

						generate_wireless(0, NULL, MSG_TYPE_CONTROL, node_mac, FRM_TYPE_RPT_PERIOD, 1, &result);
					}
					break;
				case FRM_TYPE_DVC_RESET:
					{
						uint8_t result = RESPONSE_SUCCESS;
						generate_wireless(0, NULL, MSG_TYPE_CONTROL, node_mac, FRM_TYPE_DVC_RESET, 1, &result);
						watchdog_reboot();
					}
					break;
				case FRM_TYPE_FTRY_RESET:
					{
						uint8_t result = RESPONSE_SUCCESS;
						generate_wireless(0, NULL, MSG_TYPE_CONTROL, node_mac, FRM_TYPE_FTRY_RESET, 1, &result);
						co2_config_init();
						save_config();
						watchdog_reboot();
					}
					break;
				case FRM_TYPE_CO2_OS_WR:
					{
						uint16_t end[4];
						uint8_t result;
						end[1] = value[0] + (value[1] << 8);
						end[2] = value[2] + (value[3] << 8);
						end[3] = value[4] + (value[5] << 8);
						end[4] = value[6] + (value[7] << 8);
						
						if(end[1]  < end[2]  && 
						   end[2]  < end[3]  && 
						   end[3]  < end[4] ){
							flag_co2_led = 0;
							co2_led_config.boundary_end1 = end[1];
							co2_led_config.boundary_end2 = end[2];
							co2_led_config.boundary_end3 = end[3];
							co2_led_config.boundary_end4 = end[4];
							save_config();
							flag_co2_led =1;
							result = RESPONSE_SUCCESS;
						}else{
							result = RESPONSE_FAIL;
						}

						generate_wireless(0, NULL, MSG_TYPE_CONTROL, node_mac, FRM_TYPE_CO2_OS_WR, 1, &result);
						//ctimer_set(&c_boundary_set_timer, CLOCK_SECOND>>7, callback_boundary_set, value);
					}
					break;
					
				case FRM_TYPE_CO2_OS_RD:
					{
						uint8_t result[8];
						result[0] = co2_led_config.boundary_end1 & 0xff;
						result[1] = co2_led_config.boundary_end1 >> 8;
						result[2] = co2_led_config.boundary_end2 & 0xff;
						result[3] = co2_led_config.boundary_end2 >> 8;
						result[4] = co2_led_config.boundary_end3 & 0xff;
						result[5] = co2_led_config.boundary_end3 >> 8;
						result[6] = co2_led_config.boundary_end4 & 0xff;
						result[7] = co2_led_config.boundary_end4 >> 8;

						generate_wireless(0, NULL, MSG_TYPE_CONTROL, node_mac, FRM_TYPE_CO2_OS_RD, 8, &result);
						//ctimer_set(&c_boundary_get_timer, CLOCK_SECOND>>7, callback_boundary_get, NULL);
					}
					break;	
					
				case FRM_TYPE_DVC_RESET:
				 	ctimer_set(&c_reset_timer, CLOCK_SECOND*3, callback_device_reset, NULL);
					break;
			}
			i += length + 2;  //length(1) + type(1) + value
		}
	}
	return 0;
}
/*---------------------------------------------------------------------------*/
void
read_send()
{
  temperature = sht_sensor.value(SHT_VAL_TEMP)-160; //-160 is the temp offset according to temp increase by co2 sensor
  humidity = sht_sensor.value(SHT_VAL_HUM);
  
  //temperature = SHT_ERROR; //error TEST
  //humidity = SHT_ERROR; //error TEST

  if(temperature == SHT_ERROR || humidity == SHT_ERROR){
  	  if(flag_errorled_on){
	  	  
  	  }else{ 
//		  leds_on(LEDS_WHITE);
		  flag_errorled_on = 1;
  	  }	
	  temperature = 0x8888; //error indicator
	  //0x8888 = -30584(-305.84C), this value is under alsolute zero.
	  
	  humidity = -1; //error indicator
	  //if temp/hum value is ridiculous number, there is the error on temp/hum sensor.

	  //if there is error on the co2 sensor, the co2 value will be -1 by error checker.
	 
  } else{
      if(flag_errorled_on){ 
//		  leds_off(LEDS_WHITE);
		  flag_errorled_on = 0;
      }
  }
	  // ID
	  frm_payload[0] = FRM_TYPE_ID; frm_payload[1] = LENGTH_DATA_ID;
	  frm_payload[2] = node_mac[0]; frm_payload[3] = node_mac[1]; frm_payload[4] = node_mac[2]; frm_payload[5] = node_mac[3];
	  frm_payload[6] = node_mac[4]; frm_payload[7] = node_mac[5]; frm_payload[8] = node_mac[6]; frm_payload[9] = node_mac[7];
	  // Temperature
	  frm_payload[10] = FRM_TYPE_TEMPERATURE; frm_payload[11] = LENGTH_DATA_TEMP;
	  frm_payload[12] = temperature & 0xFF; frm_payload[13] = (temperature >> 8) & 0xFF;
	  // Humidity
	  frm_payload[14] = FRM_TYPE_HUMIDITY; frm_payload[15] = LENGTH_DATA_HUM;
	  frm_payload[16] = humidity & 0xFF; frm_payload[17] = (humidity >> 8) & 0xFF;
	  // CO2
	  frm_payload[18] = FRM_TYPE_CO2; frm_payload[19] = LENGTH_DATA_CO2;
	  frm_payload[20] = co2_gas & 0xFF; frm_payload[21] = (co2_gas >> 8) & 0xFF;
	  
	  send_packet(MSG_TYPE_REPORT, frm_payload, 22);
}
/*---------------------------------------------------------------------------*/
static void callback_ledoff(void *ptr){
	leds_off(LEDS_CONF_ALL);
}
/*---------------------------------------------------------------------------*/

//static void callback_er(void *ptr){
//	if(flag_errorledco2_on){
//	}else{
//		leds_on(LEDS_WHITE);
//		flag_errorledco2_on = 1;
//	}
//	co2_gas = -1; //error indicator
//	 //if co2_gas value is ridiculous, the meaning is the error on co2 sensor.
//	ctimer_reset(&errorcheck_ct);
//}
/*---------------------------------------------------------------------------*/
static void
configure_co2_acdl(void){
	/*
	acdl mcdl
	l	h : acdl 
	h	h : normal
	h	l : mcdl
	*/
    ti_lib_gpio_clear_dio(IOID_11);
	ti_lib_gpio_set_dio(IOID_12);
}
/*---------------------------------------------------------------------------*/
static void
cdl_pin_reset(void* ptr){
   	ti_lib_gpio_set_dio(IOID_11);
	ti_lib_gpio_set_dio(IOID_12);
	flag_mcdl_ing = 0;
}
/*---------------------------------------------------------------------------*/
static void
configure_co2_mcdl(void){
	/*
	acdl mcdl
	l	h : acdl 
	h	h : normal
	h	l : mcdl
	*/
	flag_mcdl_ing = 1;
	ti_lib_gpio_set_dio(IOID_11);
	ti_lib_gpio_clear_dio(IOID_12);
	
	ctimer_set(&c_mcdl_off, 720*CLOCK_SECOND, cdl_pin_reset, NULL); 
}
/*---------------------------------------------------------------------------*/
void
co2_init(void)
{
  int bu;
  co2_config_init();
  bu = load_config();
  if(bu == BU_FACTORY) {
    // save default
    //printf("BU_FACTORY\n");
    save_config();
  }
  //configure_co2_acdl();
  //configure_co2_mcdl();
  ieee_addr_cpy_to(node_mac, 8);
  flag_co2_led = 1;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc13xx_demo_process, ev, data)
{
  PROCESS_BEGIN();
 
  ieee_addr_cpy_to(node_mac, 8);
  SENSORS_ACTIVATE(sht_sensor);
  sht_sensor.configure(SHT_RESOLUTION, SHT_RES_14T_12RH);
  
  etimer_set(&et, CLOCK_SECOND*120);
  
  co2_init();
  
  while(1) {
    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {	
	  	if(!flag_mcdl_ing){
        	read_send();
		}
        etimer_set(&et, CLOCK_SECOND*co2_led_config.transmit_period);
      }
    } else if(ev == sensors_event) {
      if(data == &button_user_sensor) {
        if((&button_user_sensor)->value(BUTTON_SENSOR_VALUE_DURATION) > CLOCK_SECOND * 3) {
          ctimer_set(&c_reset_timer, CLOCK_SECOND >> 3, callback_device_reset, NULL);
        }
      }
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(uart_rx_process, ev, data)
{
  PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
  PROCESS_BEGIN();

  uint8_t *pbuf;
  //uint16_t plen;

  uart1_set_input(apros_serial_line_input_byte);
  apros_serial_line_init();

  broadcast_open(&broadcast, 129, &broadcast_call);

  stimer_set(&st, 120);//Fixed value : init time
//  ctimer_set(&errorcheck_ct, 20*CLOCK_SECOND, callback_er, NULL); 
   
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(ev == apros_serial_line_event_message && data != NULL);
	if(flag_errorledco2_on){
//		leds_off(LEDS_WHITE);
		flag_errorledco2_on = 0;
	}
    pbuf = ((sl_data_t*)data)->buffer;
    //plen = ((sl_data_t*)data)->length;

    //apros_serial_send(pbuf, plen);
    
//	ctimer_set(&errorcheck_ct, 20*CLOCK_SECOND, callback_er, NULL);
	if((atoi((const char*)pbuf) < 300) ){
	}else{
    	co2_gas = atoi((const char*)pbuf);
	}
	if(stimer_expired(&st)){
		if(flag_co2_led){
			if(co2_gas < co2_led_config.boundary_end1){
				leds_on(LEDS_BLUE); 
			}else if(co2_gas < co2_led_config.boundary_end2){
				leds_on(LEDS_GREEN);
			}else if(co2_gas < co2_led_config.boundary_end3){
				leds_on(LEDS_YELLOW);
			}else if(co2_gas < co2_led_config.boundary_end4){
				leds_on(LEDS_MAGENTA);
			}else{
				leds_on(LEDS_RED);
			}
		}
	}else{
		if(flag_co2_led){
			leds_on(LEDS_WHITE);
		}
	}
	ctimer_set(&ledoff_ct, 3*CLOCK_SECOND, callback_ledoff, NULL);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
