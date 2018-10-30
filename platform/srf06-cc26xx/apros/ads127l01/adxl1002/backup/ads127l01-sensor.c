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
 * \addtogroup accel-sensor
 * @{
 *
 * \file
 *  Driver for the ADS127L01 sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "ads127l01-sensor.h"
#include "ads127l01-spi-control.h"
#include "sensor-common.h"
#include "ti-lib.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 0
#if DEBUG
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif
/*---------------------------------------------------------------------------*/
/* Sensor selection/deselection */
//#define SENSOR_SELECT()     apros_adis_spi_open()
//#define SENSOR_DESELECT()   apros_adis_spi_close()
/*---------------------------------------------------------------------------*/
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define SWAP(v) ((LO_UINT16(v) << 8) | HI_UINT16(v))
/*---------------------------------------------------------------------------*/

// */
//static int16_t
//ads127l01_convert(uint8_t variable, uint16_t value)
//{
//  int16_t rd;
//  uint32_t buff;
//}
/*---------------------------------------------------------------------------*/
void
ads127l01_enable(){
	ads127l01_spi_open();	
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_FILTER0);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_FILTER0, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_FILTER1);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_FILTER1, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_OSR0);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_OSR0, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_OSR1);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_OSR1, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADXL1002_STANDBY);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL1002_STANDBY, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADXL1002_ST);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL1002_ST, IOC_IOPULL_DOWN);
}
/*---------------------------------------------------------------------------*/
void
ads127l01_disable(){
	ads127l01_spi_close();	
}
/*---------------------------------------------------------------------------*/
void
ads127l01_register_read_8(uint8_t register_addr, uint8_t* rd)
{
  uint8_t command = ADS127L01_COMMAND_RREG;
  ads127l01_spi_read(command, register_addr, rd, 1);
}
/*---------------------------------------------------------------------------*/
uint8_t
ads127l01_register_write_8(uint8_t register_addr, uint8_t bit_value)
{
 uint8_t command = ADS127L01_COMMAND_WREG;
  if(ads127l01_spi_write(command, register_addr, &bit_value, 1)){
		return ADS127L01_SUCCESS;
  }else{
	  PRINTF("ADS127L01: failed to read sensor\n");
	  return ADS127L01_ERROR;
  }
}
/*---------------------------------------------------------------------------*/
/* 
\	
*/
void
ads127l01_read_data(uint8_t* rd){
	ads127l01_spi_read(ADS127L01_COMMAND_RDATA, 0, rd, 4);
}
/*---------------------------------------------------------------------------*/
/* 
\	
*/
void
ads127l01_start(){
	ads127l01_spi_write(ADS127L01_COMMAND_START1, 0, NULL, 0);
}
/*---------------------------------------------------------------------------*/
/* 
\	
*/
void
ads127l01_stop(){
	ads127l01_spi_write(ADS127L01_COMMAND_STOP1, 0, NULL, 0);
}
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* 
\	Resets both the digital filter and register contents to default settings.
*/
void
ads127l01_reset(){
		//BUG :: we can not set the reset pin
}
/*---------------------------------------------------------------------------*/
/*
\	Enters power-down mode where both the analog and digital circuitry is completely deactivated. 
\	The digital inputs are internally disabled so there is no concen in driving the pins.
*/
void
ads127l01_powerdown(){
		//BUG :: we can not set the reset pin
}
/*---------------------------------------------------------------------------*/
/*
\	two Wideband filter options, and a Low-latency filter.		
\   BUT we can not deal with this option because the start pin is always set to high.
*/
void 
ads127l01_set_filter(const ads127l01_filter filter){
	switch(filter){
		case WFILTER1_512K:
  			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER1_256K:
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER1_128K:
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER1_64K:
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER2_512K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;	
			
		case WFILTER2_256K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER2_128K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case WFILTER2_64K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case LLFILTER_512K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case LLFILTER_128K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case LLFILTER_32K:
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);
			break;
			
		case LLFILTER_8K:
			ti_lib_gpio_clear_dio(BOARD_IOID_ADS127L01_FILTER0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_FILTER1);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR0);
			ti_lib_gpio_set_dio(BOARD_IOID_ADS127L01_OSR1);			
			break;
	}
}
/*---------------------------------------------------------------------------*/
/*static int
value(uint8_t type)
{
  uint16_t val;

  if(ads127l01_status == ADS127L01_SENSOR_STATUS_DISABLED) {
    PRINTF("ADS127L01: sensor not started\n");
    return ADS127L01_ERROR;
  }

  if((type != ADS127L01_VAL_SUPPLY) && 
  	(type != ADS127L01_VAL_TEMP) && 
  	(type != ADS127L01_VAL_X_AXIS) &&
  	(type != ADS127L01_VAL_Y_AXIS) &&
  	(type != ADS127L01_VAL_Z_AXIS)) {
    PRINTF("ADS127L01: invalid value requested\n");
    return ADS127L01_ERROR;
  }

  if(ads127l01_read_16(type, &val) == ADS127L01_SUCCESS) {
      return (int)val;
  }
  return ADS127L01_ERROR;
}*/
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the SHT sensor.
 *
 * \param type Activate, enable or disable the sensor. See below
 * \param enable
 *
 * When type == SENSORS_HW_INIT we turn on the hardware
 * When type == SENSORS_ACTIVE and enable==1 we enable the sensor
 * When type == SENSORS_ACTIVE and enable==0 we disable the sensor
 */
//static int
//configure(int type, int value)
//{
//  uint8_t buf[2];
//
//  if((type != ADS127L01_ACTIVE) && (type != ADS127L01_RESET) &&
//     (type != ADS127L01_RESOLUTION)) {
//    PRINTF("ADS127L01: option not supported\n");
//    return ADS127L01_ERROR;
//  }
//
//  switch(type) {
//  case ADS127L01_ACTIVE:
//    if(value) {
//      SENSOR_SELECT();

//      /* Read the user config register */
//      if(sensor_common_read_reg(SHT_UREG_READ, &user_reg, 1) == SHT_SUCCESS) {
//        enabled = value;
//        return SHT_SUCCESS;
//      }
//    }
//
//  case SHT_RESET:
//    SENSOR_SELECT();
//    if(sensor_common_write_reg(SHT_SOFT_RESET, NULL, 0) != SHT_SUCCESS) {
//      PRINTF("SHT: failed to reset the sensor\n");
//      return SHT_ERROR;
//    }
//    clock_delay_usec(SHT_RESET_DELAY);
//    return SHT_SUCCESS;
//
//  case SHT_RESOLUTION:
//    if((value != SHT_RES_14T_12RH) && (value != SHT_RES_12T_08RH) &&
//       (value != SHT_RES_13T_10RH) && (value != SHT_RES_11T_11RH)) {
//      PRINTF("SHT: invalid resolution value\n");
//      return SHT_ERROR;
//    }
//
//    user_reg &= ~SHT_RES_11T_11RH;
//    user_reg |= value;
//    buf[0] = user_reg;

//    SENSOR_SELECT();
//    if(sensor_common_write_reg(SHT_UREG_WRITE, buf, 1) == SHT_SUCCESS) {
//      PRINTF("SHT: new user register value 0x%02X\n", user_reg);
//      return SHT_SUCCESS;
//    }
//
//  default:
//    return SHT_ERROR;
//  }
//
//  return SHT_ERROR;
//}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return One of the SENSOR_STATUS_xyz defines
 */
/*static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return ads127l01_status;
    break;
  default:
    break;
  }
  return ADS127L01_SENSOR_STATUS_DISABLED;
}*/
/*---------------------------------------------------------------------------*/
//SENSORS_SENSOR(ads127l01_sensor, "ADS127L01", value, configure, status);
/*---------------------------------------------------------------------------*/

