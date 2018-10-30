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
#include "contiki.h"
#include "apros-packet-control.h"
#include "lib/crc16.h" // crc
#include <string.h>
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"

block_info_t block_info[MAX_NUM_MSG];
uint8_t block_info_index;

#define ADD_BASIC_BLOCK(TYPE) do{\
	add_basic_block_info(\
	BLK_TYPE_##TYPE,\
	BLK_LEN_##TYPE);\
	}while(0);\

#define ADD_CONT_BLOCK(TYPE) do{\
	add_cont_block_info(\
	BLK_TYPE_##TYPE,\
	BLK_REQ_LEN_##TYPE,\
	BLK_RSP_LEN_##TYPE);\
	}while(0);\
/*---------------------------------------------------------------------------*/
void add_all_block_info(){
	ADD_BASIC_BLOCK(ID);
	ADD_BASIC_BLOCK(BATTERY);
	ADD_BASIC_BLOCK(SEQNUM);
	ADD_BASIC_BLOCK(MOTION);
	ADD_BASIC_BLOCK(TEMP);
	ADD_BASIC_BLOCK(HUMID);
	ADD_BASIC_BLOCK(LIGHT);
	ADD_BASIC_BLOCK(CO2);
	ADD_BASIC_BLOCK(ACCEL_INDEX);
	ADD_BASIC_BLOCK(ACCEL_DATA);
	
	ADD_CONT_BLOCK(SET_BASECH);
	ADD_CONT_BLOCK(GET_BASECH);
	ADD_CONT_BLOCK(GET_BASEID);
	ADD_CONT_BLOCK(BASE_RESET);
	ADD_CONT_BLOCK(SET_BASECCA);
	ADD_CONT_BLOCK(GET_BASECCA);

	ADD_CONT_BLOCK(DEV_RESET);
	ADD_CONT_BLOCK(REPORT_PERIOD);
	ADD_CONT_BLOCK(PROMPT_REPORT);
	ADD_CONT_BLOCK(FTRY_RESET);
	ADD_CONT_BLOCK(REGISTER_NODE);
	ADD_CONT_BLOCK(SET_NODECH);
	ADD_CONT_BLOCK(GET_NODECH);
	ADD_CONT_BLOCK(SET_VENDOR);
	ADD_CONT_BLOCK(GET_VENDOR);
	
	ADD_CONT_BLOCK(SET_SAMPLING);
	ADD_CONT_BLOCK(GET_SAMPLING);
	ADD_CONT_BLOCK(CAPT_REQUEST);
	
#if APROS_OLD_DF
	ADD_CONT_BLOCK(DATA_FETCH);
#else
	ADD_CONT_BLOCK(DATA_FETCH_READY);
	ADD_CONT_BLOCK(DATA_FETCH_START);
	ADD_CONT_BLOCK(GET_NODEINFO);
	ADD_CONT_BLOCK(SET_NODECCA);
	ADD_CONT_BLOCK(GET_NODECCA);
#endif
	ADD_CONT_BLOCK(SET_BOUNDARY);
	ADD_CONT_BLOCK(GET_BOUNDARY);

	ADD_CONT_BLOCK(RETT);
	ADD_CONT_BLOCK(GET_OFFSET);
	ADD_CONT_BLOCK(SET_OFFSET);
	ADD_CONT_BLOCK(AUTO_OFFSET);
}
/*---------------------------------------------------------------------------*/
basic_block_info_t* find_basic_block_info(uint8_t type){
	for(uint8_t i=0; i<block_info_index; i++){
		if(block_info[i].basic_block.type == type){
			return &block_info[i].basic_block;
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
void add_basic_block_info(uint8_t type, uint8_t length){
	basic_block_info_t* b_info = find_basic_block_info(type);
	if(b_info == NULL){
		block_info[block_info_index].basic_block.type = type;
		block_info[block_info_index++].basic_block.length = length;	
	}
}
/*---------------------------------------------------------------------------*/
cont_block_info_t* find_cont_block_info(uint8_t type){
	for(uint8_t i=0; i<block_info_index; i++){
		if(block_info[i].cont_block.type == type){
			return &block_info[i].cont_block;
		}
	}
	return NULL;
}
/*---------------------------------------------------------------------------*/
void add_cont_block_info(uint8_t type, uint8_t req_len, uint8_t rsp_len){
	cont_block_info_t* c_info = find_cont_block_info(type);
	if(c_info == NULL){
		block_info[block_info_index].cont_block.type = type;
		block_info[block_info_index].cont_block.req_length = req_len;
		block_info[block_info_index++].cont_block.rsp_length = rsp_len;
	}
}
/*---------------------------------------------------------------------------*/
uint8_t
get_pkt_length(uint8_t* pkt){
	return pkt[1]+STX_LEN+ETX_LEN;
}
/*---------------------------------------------------------------------------*/
uint8_t
apros_insert_tlv(uint8_t* init_pkt, struct tlv_set* tlv){
	uint8_t* ptr;
	uint8_t frame_len = init_pkt[1];
		
	if(frame_len + STX_LEN + ETX_LEN + tlv->length + 2 > APROS_PKT_MAX_SIZE){  //2=frm_type + frm_length size
		return 0;
	}
	if(init_pkt[0] == STX && init_pkt[frame_len + STX_LEN] == ETX){
		memmove(init_pkt + frame_len + STX_LEN + tlv->length + 2, //2=frm_type + frm_length size
			init_pkt + frame_len + STX_LEN, 1); //ETX MOVE
			
		ptr = init_pkt + frame_len + STX_LEN; //Adding point
		
		ptr[0] = tlv->type;
		ptr[1] = tlv->length;
		if(tlv->value != NULL){
			memcpy(ptr+2, tlv->value, tlv->length);
		}
		init_pkt[1] += tlv->length+2; //2=frm_type + frm_length size
		
		return  1;
	}else{
		return 0;
	}
}
/*---------------------------------------------------------------------------*/
uint8_t //calculate and add CRC // Must call this function only when packet finalization
apros_complete_pkt(uint8_t* init_pkt){
	uint16_t crc;
	uint8_t* ptr;
	uint8_t frame_len = init_pkt[1];
	
	if(frame_len + STX_LEN + ETX_LEN + CRC_LEN > APROS_PKT_MAX_SIZE){
		return 0;
	}
	if(init_pkt[0] == STX && init_pkt[frame_len + STX_LEN] == ETX){
		init_pkt[1] += CRC_LEN; 
		crc = crc16_data(init_pkt + STX_LEN, frame_len, 0); // little-endian

		memmove(init_pkt + frame_len + STX_LEN + CRC_LEN, //2==crc size
			init_pkt + frame_len + STX_LEN, 1); // ETX MOVE
			
		ptr = init_pkt + frame_len + STX_LEN;
		
		ptr[0] = crc & 0xFF;
   		ptr[1] = (crc >> 8) & 0xFF;
		
		return 1;
	}else{
		return 0;
	}
	
}
/*---------------------------------------------------------------------------*/
uint8_t //make an empty packet // must be str_length > 128
apros_init_pkt(uint8_t* arr, uint8_t arr_max_length, uint8_t m_type){
	if(arr_max_length > 4){
		arr[0] = STX;
		arr[1] = 2; // length + m_type
		arr[2] = m_type;
		arr[3] = ETX;
		return 1;
	}else{
		return 0;
	}
}
/*---------------------------------------------------------------------------*/
uint8_t
apros_base_pkt_parser(uint8_t* p_buf, uint16_t p_buf_size, uint8_t* parsing_packet, uint8_t* parsing_length, uint8_t* msg_type)
{
  uint16_t start, length, end, crc;
  end = 0;
    
  for(start = 0; start < p_buf_size; start++) {
    if(p_buf[start] == STX) {
      // Data Length, Check
      length = p_buf[start + 1];
      length += 2; // STX + LENGTH
            
      if (start + length > p_buf_size) {
	  	 PRINTF("receive more\n");
        return 0; // receive more
      }
	  for( start = 0 ; start < p_buf_size ; start++){
			PRINTF("%d : %x ", start, p_buf[start]);
	  }
      PRINTF("\n");
	  start = 0;
	  PRINTF("start: %d etx : %d\n", start, start + length - 1);
	 
      if(p_buf[start + length - 1] == ETX) { // ETX
        crc = crc16_data(p_buf + start + 1, length - 4, 0); // CRC
        PRINTF("crc value : %x \n", crc);
        if(crc == (p_buf[start + length - 2] << 8) + (p_buf[start + length - 3])) {
		  *msg_type = p_buf[start + 2];
		  
          memcpy(parsing_packet, p_buf + start + 3, length - 6);
		  *parsing_length = length-6;
       
          end = start + length;
                    
          if(start + length == p_buf_size) {
            return 1; // OK
          }
          else {
		  	 PRINTF("search more\n");
            start = end - 1;
            continue; // search more packet
          }
        }
        else { // CRC ERROR
          PRINTF("crc error\n");
          continue; // search more
        }
      }
      else { // WRONG ETX
        PRINTF("wrong etx\n");
        continue; // search more
      }
    }
  }
  PRINTF("no stx\n");   
  return -1; // NO STX
}
/*---------------------------------------------------------------------------*/

uint8_t
apros_node_pkt_parser(uint8_t* p_buf, uint16_t p_buf_size, uint8_t* parsing_packet, uint8_t* parsing_length)
{
  uint16_t start, length, end, crc;
  end = 0;
    
  for(start = 0; start < p_buf_size; start++) {
    if(p_buf[start] == STX) {
      // Data Length, Check
      length = p_buf[start + 1];
      length += 2; // STX + LENGTH
            
      if (start + length > p_buf_size) {
	  	 PRINTF("receive more\n");
        return 0; // receive more
      }
	  for( start = 0 ; start < p_buf_size ; start++){
			PRINTF("%d : %x ", start, p_buf[start]);
	  }
      PRINTF("\n");
	  start = 0;
	  PRINTF("start: %d etx : %d\n", start, start + length - 1);
	 
      if(p_buf[start + length - 1] == ETX) { // ETX
        crc = crc16_data(p_buf + start + 1, length - 4, 0); // CRC
        PRINTF("crc value : %x \n", crc);
        if(crc == (p_buf[start + length - 2] << 8) + (p_buf[start + length - 3])) {
          // Control?
          if(p_buf[start + 2] == MSG_TYPE_CONTROL) {
            memcpy(parsing_packet, p_buf + start + 3, length - 6);
			*parsing_length = length-6;
          }

          end = start + length;
                    
          if(start + length == p_buf_size) {
            return 1; // OK
          }
          else {
		  	 PRINTF("search more\n");
            start = end - 1;
            continue; // search more packet
          }
        }
        else { // CRC ERROR
          PRINTF("crc error\n");
          continue; // search more
        }
      }
      else { // WRONG ETX
        PRINTF("wrong etx\n");
        continue; // search more
      }
    }
  }
  PRINTF("no stx\n");   
  return -1; // NO STX
}
/*---------------------------------------------------------------------------*/

