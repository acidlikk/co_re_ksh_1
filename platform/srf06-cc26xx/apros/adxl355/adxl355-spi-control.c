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
#include "adxl355-spi-control.h"
#include "ti-lib.h"
#include "adxl355-board-spi.h"

#include <stdint.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
/**
 * Clear CSN line
 */
static void
select_on_bus(void)
{
  ti_lib_gpio_clear_dio(BOARD_IOID_ADXL355_ADXL_CS);
}
/*---------------------------------------------------------------------------*/
/**
 * Set CSN line
 */
static void
deselect(void)
{
  ti_lib_gpio_set_dio(BOARD_IOID_ADXL355_ADXL_CS);
}
/*---------------------------------------------------------------------------*/
//static bool
//capt_start_signal(void){
//	ti_lib_gpio_set_dio(BOARD_IOID_IRQ2);
//	
//	ti_lib_gpio_clear_dio(BOARD_IOID_IRQ2);
//}
/*---------------------------------------------------------------------------*/
bool
adxl355_spi_open()
{
ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADXL355_SPI_SCK);
		
ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADXL355_SPI_MOSI);

ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADXL355_SPI_MISO);
//ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL355_SPI_MISO, IOC_IOPULL_UP);

ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADXL355_ADXL_CS);

  adxl355_board_spi_open(ADXL355_SPI_BITRATE_CONF, BOARD_IOID_ADXL355_SPI_SCK); 

  /* Put the part is standby mode */
  //power_standby();
  return 1;
}
/*---------------------------------------------------------------------------*/
void
adxl355_spi_close()
{
  /* Put the part in low power mode */
//  power_down();

  adxl355_board_spi_close();
}
/*---------------------------------------------------------------------------*/
bool adxl355_spi_read(uint8_t register_addr, uint8_t *buf, size_t length)
{
  uint8_t wbuf;
  bool ret; 
  /* Wait till previous operation completes */

  wbuf = register_addr;

  //wait_ready();
  
  select_on_bus();

  if(adxl355_board_spi_write(&wbuf, 1) == false) {
    /* failure */
    deselect();
    return false;
  }
  //wait_ready();
  //wait_ready(); //To make a litte time gap between read and write 
  
  ret = adxl355_board_spi_read(buf, length);

  deselect();
  
  return ret;
}
/*---------------------------------------------------------------------------*/
bool adxl355_spi_write(uint8_t register_addr, uint8_t bit_value)
{
  uint8_t wbuf[2];
  bool ret;
  /* Wait till previous operation completes */

  wbuf[0] = register_addr;
  wbuf[1] = bit_value;

//  wait_ready();

  select_on_bus();

  if(adxl355_board_spi_write(wbuf, sizeof(wbuf)) == false) {
    /* failure */
    deselect();
    return false;
  }
  
  deselect();
  return ret;
}
/*---------------------------------------------------------------------------*/
void
adxl355_spi_init()
{
  adxl355_spi_open();
  adxl355_spi_close();
}
/*---------------------------------------------------------------------------*/
/** @} */

