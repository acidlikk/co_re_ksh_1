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

#include "apros-flash-control.h"

#define APROS_NODE_CONFIG_P  0
#define APROS_BASE_CONFIG_P  0

#define APROS_ACCEL_DATA_P   64 // TODO
#define APROS_ACCEL_LAST_DATA_INDEX_P 0xFF //TODO
#define APROS_ACCEL_TOTAL_DATA_NUM_P 0xFF // TODO
/*---------------------------------------------------------------------------*/
/* Ext_Flash */ 
/*---------------------------------------------------------------------------*/
uint8_t apros_read_flash(size_t r_point, uint8_t* r_data, size_t r_len)
{
  uint8_t rv = ext_flash_open();

  if(!rv) {
    ext_flash_close();
    return 0;
  }

  rv = ext_flash_read(r_point, r_len, r_data);
  ext_flash_close();

  if(!rv) {
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t apros_write_flash(size_t w_point, uint8_t* w_data, size_t w_len)
{
  uint8_t rv = ext_flash_open();

  if(!rv) {
    ext_flash_close();
    return 0;
  }

  rv = ext_flash_erase(w_point, w_len);

  if(!rv) {
  	ext_flash_close();
    return 0;
  }
  
  rv = ext_flash_write(w_point, w_len, w_data);
  ext_flash_close();
  
  if(!rv) {
  	return 0;
  }
  
  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t apros_node_config_load(apros_node_config_t* conf){
	return apros_read_flash(APROS_NODE_CONFIG_P, (uint8_t*)conf, sizeof(apros_node_config_t));
}
/*---------------------------------------------------------------------------*/
uint8_t apros_node_config_save(apros_node_config_t* conf){
	return apros_write_flash(APROS_NODE_CONFIG_P, (uint8_t*)conf, sizeof(apros_node_config_t));
}
/*---------------------------------------------------------------------------*/
uint8_t apros_base_config_load(apros_base_config_t* conf){
	return apros_read_flash(APROS_BASE_CONFIG_P, (uint8_t*)conf, sizeof(apros_base_config_t));
}
/*---------------------------------------------------------------------------*/
uint8_t apros_base_config_save(apros_base_config_t* conf){
	return apros_write_flash(APROS_BASE_CONFIG_P, (uint8_t*)conf, sizeof(apros_base_config_t));
}
/*---------------------------------------------------------------------------*/

//TODO::
uint8_t apros_accel_data_last_data_index(){
	static uint8_t index;
	if(apros_read_flash(APROS_ACCEL_LAST_DATA_INDEX_P, &index, 1))
		return index;
	else
		return 0;
}
/*---------------------------------------------------------------------------*/
//TODO::
uint8_t apros_accel_data_write(uint8_t* wdata, size_t length, uint8_t sampling){
	return apros_write_flash(APROS_ACCEL_DATA_P, wdata, length);
}
/*---------------------------------------------------------------------------*/
//TODO::
uint8_t apros_accel_data_read(uint8_t* rdata, size_t length, uint8_t sampling, uint8_t index){
	return apros_read_flash(APROS_ACCEL_DATA_P, rdata, length);
}
/*---------------------------------------------------------------------------*/



