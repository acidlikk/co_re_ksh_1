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
  * 
  * \author
  * Adam Dunkels
  * Sanghyun Kim
  *
  */
#ifndef APROS_FLASH_CONTROL_H_
#define APROS_FLASH_CONTROL_H_
#include "contiki.h"
#include "ext-flash.h"

typedef struct apros_node_config {
	uint32_t dev_name;
 	uint8_t base_id[8];
  	uint8_t channel; 
	uint8_t cca_th;
#if APROS_ACCEL
	uint8_t sampling_exp; 
	//int32_t offset_val; //soft offset
	uint8_t ofc[3];
#endif
#if APROS_CO2_ENABLE
	int16_t boundary[6]; 
#endif
#if APROS_PERIODIC_PKT
	uint16_t pkt_period;
#endif
	uint8_t __vendor[16];
} apros_node_config_t;

typedef struct apros_base_config {
	uint32_t dev_name;
	uint8_t channel; 
	uint8_t cca_th;
}apros_base_config_t;
/*---------------------------------------------------------------------------*/
uint8_t apros_node_config_load(apros_node_config_t* conf);
uint8_t apros_node_config_save(apros_node_config_t* conf);
uint8_t apros_base_config_load(apros_base_config_t* conf);
uint8_t apros_base_config_save(apros_base_config_t* conf);

uint8_t apros_read_flash(size_t r_point, uint8_t* r_data, size_t r_len);
uint8_t apros_write_flash(size_t w_point, uint8_t* w_data, size_t w_len);

#endif

