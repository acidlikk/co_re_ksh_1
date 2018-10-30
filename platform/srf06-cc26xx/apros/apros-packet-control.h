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
 * Packet process header filer
 * \author
 * Joonbum Kim
 * \modifier
 * Sanghyun Kim
 */
#ifndef APROS_PACKET_CONTROL_H_
#define APROS_PACKET_CONTROL_H_
#include "apros-command-scheduler.h"
/*---------------------------------------------------------------------------*/
/* Protocol */
#define STX                   0x40
#define ETX                   0x7D

#define MSG_TYPE_REPORT       0x00
#define MSG_TYPE_CONTROL      0x01
#define MSG_TYPE_RAWDATA	  0x02

//BLOCK_TYPE(BASIC)
#define BLK_TYPE_ID 		  0x01
#define BLK_LEN_ID		  	  0x08

#define BLK_TYPE_BATTERY	  0x02
#define BLK_LEN_BATTERY		  0x02

#define BLK_TYPE_SEQNUM		  0x06
#define BLK_LEN_SEQNUM	      0x01

#define BLK_TYPE_MOTION	      0x10
#define BLK_LEN_MOTION	      0x01

#define BLK_TYPE_TEMP		  0x11
#define BLK_LEN_TEMP		  0x02

#define BLK_TYPE_HUMID		  0x12
#define BLK_LEN_HUMID		  0x02

#define BLK_TYPE_LIGHT		  0x14
#define BLK_LEN_LIGHT		  0x02

#define BLK_TYPE_CO2		  0x21
#define BLK_LEN_CO2		      0x02

#define BLK_TYPE_ACCEL_DATA	  0xA0
#define BLK_LEN_ACCEL_DATA	  0x30 // Max Length

#define BLK_TYPE_ACCEL_INDEX  0xAF
#define BLK_LEN_ACCEL_INDEX   0x02

//BLOCK_TYPE(CONTROL)
#if APROS_OLD_DF //BASE - old
#define BLK_TYPE_SET_BASECH	 			  0x8A
#define BLK_REQ_LEN_SET_BASECH		  	  0x01
#define BLK_RSP_LEN_SET_BASECH		      0x01

#define BLK_TYPE_GET_BASECH         	  0x8B
#define BLK_REQ_LEN_GET_BASECH		      0x00
#define BLK_RSP_LEN_GET_BASECH		      0x01

#define BLK_TYPE_GET_BASEID	  	    	  0x8C
#define BLK_REQ_LEN_GET_BASEID		      0x00
#define BLK_RSP_LEN_GET_BASEID		      0x01

#define BLK_TYPE_BASE_RESET 			  0x8D
#define BLK_REQ_LEN_BASE_RESET			  0x00
#define BLK_RSP_LEN_BASE_RESET			  0x01
#else //BASE - new
#define BLK_TYPE_BASE_RESET 			  0x70
#define BLK_REQ_LEN_BASE_RESET			  0x00
#define BLK_RSP_LEN_BASE_RESET			  0x01

#define BLK_TYPE_SET_BASECH	 		 	  0x76
#define BLK_REQ_LEN_SET_BASECH		  	  0x01
#define BLK_RSP_LEN_SET_BASECH		      0x01

#define BLK_TYPE_GET_BASECH          	  0x77
#define BLK_REQ_LEN_GET_BASECH		      0x00
#define BLK_RSP_LEN_GET_BASECH		      0x01

#define BLK_TYPE_GET_BASEID	  	     	  0x78
#define BLK_REQ_LEN_GET_BASEID		      0x00
#define BLK_RSP_LEN_GET_BASEID		      0x01

#define BLK_TYPE_SET_BASECCA		      0x79
#define BLK_REQ_LEN_SET_BASECCA			  0x01
#define BLK_RSP_LEN_SET_BASECCA			  0x01

#define BLK_TYPE_GET_BASECCA			  0x7A
#define BLK_REQ_LEN_GET_BASECCA			  0x00
#define BLK_RSP_LEN_GET_BASECCA			  0x01
#endif

//NODE-Common
#define BLK_TYPE_DEV_RESET           	  0x80
#define BLK_REQ_LEN_DEV_RESET		      0x00
#define BLK_RSP_LEN_DEV_RESET		      0x01

#define BLK_TYPE_REPORT_PERIOD			  0x81
#define BLK_REQ_LEN_REPORT_PERIOD		  0x02
#define BLK_RSP_LEN_REPORT_PERIOD		  0x01

#define BLK_TYPE_PROMPT_REPORT			  0x82
#define BLK_REQ_LEN_PROMPT_REPORT		  0x00
#define BLK_RSP_LEN_PROMPT_REPORT		  0x01

#define BLK_TYPE_FTRY_RESET          	  0x84
#define BLK_REQ_LEN_FTRY_RESET 		   	  0x00
#define BLK_RSP_LEN_FTRY_RESET 		      0x01

#define BLK_TYPE_REGISTER_NODE       	  0x85
#define BLK_REQ_LEN_REGISTER_NODE         0x08
#define BLK_RSP_LEN_REGISTER_NODE	      0x01

#define BLK_TYPE_SET_NODECH	  		  	  0x86
#define BLK_REQ_LEN_SET_NODECH		      0x01
#define BLK_RSP_LEN_SET_NODECH		      0x01

#define BLK_TYPE_GET_NODECH	              0x87
#define BLK_REQ_LEN_GET_NODECH		      0x00
#define BLK_RSP_LEN_GET_NODECH		      0x01

#define VENDOR_MAX_LEN					  0x10

#define BLK_TYPE_SET_VENDOR               0x88
#define BLK_REQ_LEN_SET_VENDOR		      (VENDOR_MAX_LEN)
#define BLK_RSP_LEN_SET_VENDOR		      0x01

#define BLK_TYPE_GET_VENDOR   		      0x89
#define BLK_REQ_LEN_GET_VENDOR		      0x00
#define BLK_RSP_LEN_GET_VENDOR		      (VENDOR_MAX_LEN)

#if !APROS_OLD_DF //NODE-Common - new
#define BLK_TYPE_GET_NODEINFO 		      0x8A
#define BLK_REQ_LEN_GET_NODEINFO		  0x00
//#define BLK_RSP_LEN_GET_NODEINFO	      0x2B 
#define BLK_RSP_LEN_GET_NODEINFO	      0x2F //

#define BLK_TYPE_SET_NODECCA		      0x8B
#define BLK_REQ_LEN_SET_NODECCA			  0x01
#define BLK_RSP_LEN_SET_NODECCA			  0x01

#define BLK_TYPE_GET_NODECCA			  0x8C
#define BLK_REQ_LEN_GET_NODECCA			  0x00
#define BLK_RSP_LEN_GET_NODECCA			  0x01
#endif

#define BOUNDARY_MAX_LEN				  0x0C

#define BLK_TYPE_SET_BOUNDARY			  0x8D
#define BLK_REQ_LEN_SET_BOUNDARY          (BOUNDARY_MAX_LEN)
#define BLK_RSP_LEN_SET_BOUNDARY		  0x01

#define BLK_TYPE_GET_BOUNDARY             0x8E
#define BLK_REQ_LEN_GET_BOUNDARY		  0x00
#define BLK_RSP_LEN_GET_BOUNDARY		  (BOUNDARY_MAX_LEN)

//NODE-Accel
#define BLK_TYPE_SET_SAMPLING	 	      0xA1
#define BLK_REQ_LEN_SET_SAMPLING		  0x01
#define BLK_RSP_LEN_SET_SAMPLING		  0x01

#define BLK_TYPE_GET_SAMPLING		      0xA2
#define BLK_REQ_LEN_GET_SAMPLING		  0x00
#define BLK_RSP_LEN_GET_SAMPLING		  0x01

#define BLK_TYPE_CAPT_REQUEST 		      0xA3
#define BLK_REQ_LEN_CAPT_REQUEST		  0x00
#define BLK_RSP_LEN_CAPT_REQUEST		  0x01

#if APROS_OLD_DF //Node-Accel - old
#define BLK_TYPE_DATA_FETCH               0xA4
#define BLK_REQ_LEN_DATA_FETCH		      0x01
#define BLK_RSP_LEN_DATA_FETCH		      0x06
#define BLK_RSP2_LEN_DATA_FETCH			  0x01

#define BLK_TYPE_RETT		              0xA5
#define BLK_REQ_LEN_RETT		          0x02
#define BLK_RSP_LEN_RETT		          0x06
#else //Node-Accel - new
#define BLK_TYPE_DATA_FETCH_READY         0xA4
//#define BLK_REQ_LEN_DATA_FETCH_READY 	  0x01
#define BLK_REQ_LEN_DATA_FETCH_READY 	  0x02 //170916
//#define BLK_RSP_LEN_DATA_FETCH_READY 	  0x06
//#define BLK_RSP_LEN_DATA_FETCH_READY 	  0x07 //170916
#define BLK_RSP_LEN_DATA_FETCH_READY 	  0x0B //170925

#define BLK_TYPE_DATA_FETCH_START 		  0xA5
//#define BLK_REQ_LEN_DATA_FETCH_START    0x00
#define BLK_REQ_LEN_DATA_FETCH_START      0x01 //170916
//#define BLK_RSP_LEN_DATA_FETCH_START 	  0x01
#define BLK_RSP_LEN_DATA_FETCH_START 	  0x02 //170916


#define BLK_TYPE_RETT		          	  0xA6
#define BLK_REQ_LEN_RETT		          0x02
#define BLK_RSP_LEN_RETT		          0x06
#endif

#define BLK_TYPE_SET_OFFSET	              0xA7
#define BLK_REQ_LEN_SET_OFFSET		      0x04
#define BLK_RSP_LEN_SET_OFFSET		      0x01

#define BLK_TYPE_GET_OFFSET	              0xA8
#define BLK_REQ_LEN_GET_OFFSET			  0x00
#define BLK_RSP_LEN_GET_OFFSET		      0x05

#define BLK_TYPE_AUTO_OFFSET   	          0xA9
#define BLK_REQ_LEN_AUTO_OFFSET 		  0x00
#define BLK_RSP_LEN_AUTO_OFFSET 		  0x01

//RSP_TYPE
#define RESPONSE_SUCCESS     			 0x00
#define RESPONSE_INVALID_ARGU 			 0xFF
#define RESPONSE_TIMEOUT      			 0xFE
#define RESPONSE_ALREADY_EXIST 			 0xFD
#define RESPONSE_NODE_BUSY	  			 0xFC
#define RESPONSE_NOT_EXIST_CAPT			 0xFB
#define RESPONSE_NOT_FETCH_RDY			 0xFA
#define RESPONSE_NOT_SURPT_CH			 0xF9
#define RESPONSE_NOT_SUPRT_BLOCK_TYPE 	 0xF8
#define RESPONSE_FLASH_ERROR 			 0xF7

//ADDR_TYPE
#define	BR_ADDR		 	 	  0x01
#define MY_ADDR	 		  	  0x02
#define OTHER_ADDR	      	  0x03

typedef struct tlv_set{
	uint8_t type;
	uint8_t length;
	uint8_t *value;
}tlv_set_t;

typedef struct tlv_buf_hdr{
	uint8_t tlv_num;
	tlv_set_t* tlv_p;
}tlv_buf_hdr_t;

typedef struct cont_block_info{
	uint8_t type;
	uint8_t req_length;
	uint8_t rsp_length;
}cont_block_info_t;

typedef struct basic_block_info{
	uint8_t type;
	uint8_t length;
}basic_block_info_t;

typedef union block_info{
	cont_block_info_t cont_block;
	basic_block_info_t basic_block;
}block_info_t;

#define CRC_LEN 2
#define STX_LEN 1
#define ETX_LEN 1
#define MTYPE_LEN 1
#define FRMLEN_LEN 1
#define APROS_HDR_LEN (STX_LEN+FRMLEN_LEN+MTYPE_LEN)
#define APROS_FOOT_LEN (CRC_LEN+ETX_LEN)

#define APROS_PKT_MIN_SIZE (BLK_LEN_ID+APROS_HDR_LEN+APROS_FOOT_LEN)
#define APROS_PKT_MAX_SIZE 100
#define APROS_PL_MAX_SIZE (APROS_PKT_MAX_SIZE-APROS_HDR_LEN-APROS_FOOT_LEN)
/*---------------------------------------------------------------------------*/
void add_all_block_info();
basic_block_info_t* find_basic_block_info(uint8_t type);
void add_basic_block_info(uint8_t type, uint8_t length);

cont_block_info_t* find_cont_block_info(uint8_t type);
void add_cont_block_info(uint8_t type, uint8_t req_len, uint8_t rsp_len);
/*---------------------------------------------------------------------------*/
uint8_t get_pkt_length(uint8_t* pkt);
uint8_t apros_insert_tlv(uint8_t* init_pkt, struct tlv_set* tlv);

uint8_t apros_complete_pkt(uint8_t* pkt);
uint8_t apros_init_pkt(uint8_t* arr, uint8_t arr_max_length, uint8_t m_type);
/*---------------------------------------------------------------------------*/
uint8_t apros_base_pkt_parser(uint8_t* p_buf, uint16_t p_buf_size, uint8_t* parsing_packet, uint8_t* parsing_length, uint8_t* msg_type);
uint8_t apros_node_pkt_parser(uint8_t* p_buf, uint16_t p_buf_size, uint8_t* parsing_packet, uint8_t* parsing_length);
/*---------------------------------------------------------------------------*/
#endif /* APROS_PACKET_CONTROL_H_ */
