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

 
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ads127l01-spi-control.h"
#include "ti-lib.h"
#include "ads127l01-board-spi.h"
#include <stdint.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
/**
 * Clear CSN line
 */
static void
select_on_bus(void)
{
  ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_SPI_CS);
}
static void 
deselect(void){
   ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_SPI_CS);
}
/*---------------------------------------------------------------------------*/
bool
ads127l01_spi_open()
{
  ads127l01_board_spi_open(ADS127L01_SPI_BITRATE_CONF, BOARD_IOID_ADS127L01_SPI_SCK); 
  /* Default output to clear chip select */
  return 1;
}
/*---------------------------------------------------------------------------*/
void
ads127l01_spi_close()
{
  ads127l01_board_spi_close();
}
/*---------------------------------------------------------------------------*/
bool ads127l01_spi_read(uint8_t command, uint8_t register_addr, uint8_t *output_buf, size_t value_length)
{
  uint8_t wbuf[2];
  bool ret; 
  /* Wait till previous operation completes */

  wbuf[0] = (register_addr&0x0f) | (command); 
  select_on_bus();
  if(value_length <= 0 || command == 0x12){
  
	  if(ads127l01_board_spi_write(wbuf, 1) == false) { //if value_length == 0, we don't input second byte
	    return false;
	  }
  }else{
      wbuf[1] = value_length-1; // nnnn-1 : number of registers to be read - 1
      
	  if(ads127l01_board_spi_write(wbuf, sizeof(wbuf)) == false) {
	    return false;
	  }
  }

  ret = ads127l01_board_spi_read(output_buf, value_length);
  deselect();
  return ret;
}
/*---------------------------------------------------------------------------*/
bool ads127l01_spi_write(uint8_t command, uint8_t register_addr, uint8_t* input_buf, size_t value_length)
{
  uint8_t wbuf[9];
  uint8_t i;
  bool ret;

  if(value_length > 5){
	return false;
  }
  
  
  wbuf[0] = (register_addr&0x0f) | (command); 
  select_on_bus();

  if(value_length <= 0){
	  if(ads127l01_board_spi_write(wbuf, value_length+2) == false) { // wbuf[0:1] + value
    	   return false;
  	  }
  }else{	
	   wbuf[1] = value_length-1; // nnnn-1 : number of registered to be written -1 
	   
	   for(i = 0 ; i < value_length ; i++ ){
			wbuf[i+2]=*input_buf;
			input_buf++;
  	   }
	   if(ads127l01_board_spi_write(wbuf, value_length+2) == false) { // wbuf[0:1] + value
    		return false;
  	   }
  }
  deselect();
  return ret;
}
/*---------------------------------------------------------------------------*/
void
ads127l01_spi_init()
{
  ads127l01_spi_open();
  ads127l01_spi_close();
}
/*---------------------------------------------------------------------------*/
/** @} */

