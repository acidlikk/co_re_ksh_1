/*
 * Copyright (c) 2010, Swedish Institute of Computer Science.
 * Copyright (c) 2016, Zolertia <http://www.zolertia.com>
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
/*---------------------------------------------------------------------------*/
#include "adxl355.h"
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif  

#if USE_COMM==I2C__
#define SENSOR_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, ADXL355_ADDR_ASEL_L)
#define SENSOR_DESELECT()   board_i2c_deselect()
#endif

static uint8_t enabled;

/*---------------------------------------------------------------------------*/
static uint8_t
adxl355_is_present(void)
{
  uint8_t is_present=0;
  uint8_t type_addr = (ADXL355_DEVID_AD << 1 | 0x01);

#if USE_COMM==I2C__  
  SENSOR_SELECT();
  board_i2c_write_read(&type_addr, 1, &is_present, 1);
  SENSOR_DESELECT();
#endif
#if USE_COMM==SPI__
  adxl355_spi_open();
  adxl355_spi_read(type_addr, &is_present , 1);
  adxl355_spi_close();
#endif
  return is_present == ADXL355_DEVID_VALUE;
}
/*---------------------------------------------------------------------------*/
static uint32_t
adxl355_read_accel(uint8_t addr1, uint8_t addr2, uint8_t addr3)
{
  uint8_t acceleration[3];
  
  uint32_t result;
#if USE_COMM==I2C__ 
  SENSOR_SELECT();
  board_i2c_write_read(&addr1, 1, &acceleration[0], 1);
  board_i2c_write_read(&addr2, 1, &acceleration[1], 1);
  board_i2c_write_read(&addr3, 1, &acceleration[2], 1);
  SENSOR_DESELECT();
#endif
#if USE_COMM==SPI__
  adxl355_spi_open();
  adxl355_spi_read(ADXL355_READ_ADDR(addr1), &acceleration[0], 1);
  adxl355_spi_read(ADXL355_READ_ADDR(addr2), &acceleration[1], 1);
  adxl355_spi_read(ADXL355_READ_ADDR(addr3), &acceleration[2], 1);
  adxl355_spi_close();
#endif
  result = (acceleration[0] << 12) | (acceleration[1] << 4) | (acceleration[2] >> 4) ;

  return result;
}
/*---------------------------------------------------------------------------*/
static void
adxl355_write_reg(uint8_t register_addr, uint8_t value){
#if USE_COMM==SPI__
  adxl355_spi_open();
  adxl355_spi_write(ADXL355_WRITE_ADDR(register_addr), value); 
  adxl355_spi_close();
#endif
}
/*---------------------------------------------------------------------------*/
static void
adxl355_init(void)
{

#if USE_COMM==I2C__  
  uint8_t type_addr[2];

  type_addr[0] = ADXL355_POWER_CTL;
  type_addr[1] = 0;

  SENSOR_SELECT();
  board_i2c_write(type_addr, 2);
  SENSOR_DESELECT();
#endif
#if USE_COMM==SPI__
  adxl355_write_reg(ADXL355_POWER_CTL, 0);
#endif
/*
  type_addr[0] = ADXL355_SELF_TEST;
  type_addr[1] = 0;
  
  SENSOR_SELECT();
  board_i2c_write(type_addr, 2);
  SENSOR_DESELECT();

  type_addr[0] = ADXL355_FILTER;
  type_addr[1] = 0x00;
  
  SENSOR_SELECT();
  board_i2c_write(type_addr, 2);
  SENSOR_DESELECT();
  */
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  if(!enabled) {
    PRINTF("ADXL355: sensor not started\n");
    return ADXL355_ERROR;
  }

  if(type == ADXL355_READ_X) {
    return adxl355_read_accel(ADXL355_XDATA3, ADXL355_XDATA2, ADXL355_XDATA1);
  } else if(type == ADXL355_READ_Y) {
    return adxl355_read_accel(ADXL355_YDATA3, ADXL355_YDATA2, ADXL355_YDATA1);
  } else if(type == ADXL355_READ_Z) {
    return adxl355_read_accel(ADXL355_ZDATA3, ADXL355_ZDATA2, ADXL355_ZDATA1);
  } else {
    PRINTF("ADXL355: invalid value requested\n");
    return ADXL355_ERROR;
  }

  return ADXL355_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  if(type == SENSORS_ACTIVE) {
    if(!adxl355_is_present()) {
      PRINTF("ADXL355: is not present\n");
      enabled = 0;
      return ADXL355_ERROR;
    } else {
      adxl355_init();
      enabled = 1;
      return ADXL355_SUCCESS;
    }
  }
  if(type == ADXL355_RANGE){
	 if(!adxl355_is_present()){
		return ADXL355_ERROR;
	 }else{
		adxl355_write_reg(ADXL355_RANGE, value);
	 }
  }

  return ADXL355_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adxl355, ADXL355_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */


