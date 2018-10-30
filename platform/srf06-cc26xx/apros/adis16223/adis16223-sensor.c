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
 * \addtogroup sht-sensor
 * @{
 *
 * \file
 *  Driver for the Adis16223 sensor
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/ctimer.h"
#include "lib/sensors.h"
#include "adis16223-sensor.h"
#include "adis16223-spi-control.h"
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

/*---------------------------------------------------------------------------*/
static int adis16223_status = ADIS16223_SENSOR_STATUS_DISABLED;
static uint8_t user_reg;
/*---------------------------------------------------------------------------*/
/**
 * \brief       Convert raw data to temperature and humidity
 * \param       temp - converted temperature
 * \param       hum - converted humidity
// */
//static int16_t
//adis16223_convert(uint8_t variable, uint16_t value)
//{
//  int16_t rd;
//  uint32_t buff;
//}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_enable(){
	adis16223_spi_open();	
	adis16223_status = ADIS16223_SENSOR_STATUS_ENABLED;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_disable(){
	adis16223_spi_close();	
	adis16223_status = ADIS16223_SENSOR_STATUS_DISABLED;
}
/*---------------------------------------------------------------------------*/
void
adis16223_read_8(uint8_t variable, uint8_t rd[][2])
{
  adis16223_spi_read(variable, rd, 2);
  	//== ADIS16223_SUCCESS) {
    //*rd = adis16223_convert(variable, raw);
    //return ADIS16223_SUCCESS;
  //}
 // PRINTF("ADIS16223: failed to read sensor\n");
  //return ADIS16223_ERROR;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_read_16(uint8_t variable, uint16_t *rd)
{
  uint8_t buf[2];

//  if((variable != ADIS16223_VAL_SUPPLY) && 
//  	(variable != ADIS16223_VAL_TEMP) && 
//  	(variable != ADIS16223_VAL_X_AXIS) &&
//  	(variable != ADIS16223_VAL_Y_AXIS) &&
//  	(variable != ADIS16223_VAL_Z_AXIS)) {
//    PRINTF("ADIS16223: invalid sensor requested\n");
//    return ADIS16223_ERROR;
//  }
  
  if(adis16223_spi_read(variable, buf, 2) == ADIS16223_SUCCESS) {
    *rd = (buf[0] << 8) + buf[1];
    //*rd = adis16223_convert(variable, raw);
    return ADIS16223_SUCCESS;
  }

  PRINTF("ADIS16223: failed to read sensor\n");
  return ADIS16223_ERROR;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_write(uint8_t variable, uint8_t ul_byte_addr, uint8_t bit_value)
{
  uint8_t buf[2];

//  if((variable != ADIS16223_VAL_SUPPLY) && 
//  	(variable != ADIS16223_VAL_TEMP) && 
//  	(variable != ADIS16223_VAL_X_AXIS) &&
//  	(variable != ADIS16223_VAL_Y_AXIS) &&
//  	(variable != ADIS16223_VAL_Z_AXIS)) {
//    PRINTF("ADIS16223: invalid sensor requested\n");
//    return ADIS16223_ERROR;
//  }
  switch(ul_byte_addr){
	case ADIS16223_LOWER_BYTE:
		if(adis16223_spi_write(variable, bit_value) == ADIS16223_SUCCESS) {
		    return ADIS16223_SUCCESS;
	  	}
		break;
	case ADIS16223_UPPER_BYTE:
		 if(adis16223_spi_write(variable+1, bit_value) == ADIS16223_SUCCESS) {
   			return ADIS16223_SUCCESS;
 		 }
		break;
	default:
		break;
  }
 

  PRINTF("ADIS16223: failed to read sensor\n");
  return ADIS16223_ERROR;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_dio_configure(uint8_t dio1_pol, uint8_t dio1_func, uint8_t dio2_pol, uint8_t dio2_func){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_DIO_CTRL, ADIS16223_LOWER_BYTE, dio1_pol|dio1_func|dio2_pol|dio2_func);
	return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t 
adis16223_avg_cnt_configure(uint8_t d_value){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_AVG_CNT, ADIS16223_LOWER_BYTE, d_value);
}
/*---------------------------------------------------------------------------*/
void
adis16223_capture_offset_reset(){
	adis16223_write(ADIS16223_NULL_X, ADIS16223_LOWER_BYTE, 0);
	adis16223_write(ADIS16223_NULL_X, ADIS16223_UPPER_BYTE, 0);
	adis16223_write(ADIS16223_NULL_Y, ADIS16223_LOWER_BYTE, 0);
	adis16223_write(ADIS16223_NULL_Y, ADIS16223_UPPER_BYTE, 0);
	adis16223_write(ADIS16223_NULL_Z, ADIS16223_LOWER_BYTE, 0);
	adis16223_write(ADIS16223_NULL_Z, ADIS16223_UPPER_BYTE, 0);
}
/*---------------------------------------------------------------------------*/

uint8_t 
adis16223_capture_configure(uint8_t power_ctrl, uint8_t mode, uint8_t evt_sample, uint8_t auto_flash, uint8_t band_filter, uint8_t ext_ch){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_CAPT_CTRL, ADIS16223_LOWER_BYTE, power_ctrl|mode|evt_sample|auto_flash|band_filter);
	if(ret){
		ret = adis16223_write(ADIS16223_CAPT_CTRL, ADIS16223_UPPER_BYTE, ext_ch);
	}
	return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_capture_clear_buffer(){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_GLOB_CMD, ADIS16223_UPPER_BYTE, ADIS16223_GLOB_CMD_CLEAR_CB);
	return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_capture_trigger(){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_GLOB_CMD, ADIS16223_UPPER_BYTE, ADIS16223_GLOB_CMD_CAPT_TR);
	return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_capture_pntrmove(uint16_t index){
	//uint8_t ret;
	adis16223_write(ADIS16223_CAPT_PNTR, ADIS16223_LOWER_BYTE, index&0x00ff);
	adis16223_write(ADIS16223_CAPT_PNTR, ADIS16223_UPPER_BYTE, index>>8);

	//return ret;
}
/*---------------------------------------------------------------------------*/
uint8_t
adis16223_capture_pntrreset(){
	uint8_t ret;
	ret = adis16223_write(ADIS16223_GLOB_CMD, ADIS16223_UPPER_BYTE, ADIS16223_GLOB_CMD_RESET_PNTR);
	return ret;
}
/*---------------------------------------------------------------------------*/
static int
value(uint8_t type)
{
  uint16_t val;

  if(adis16223_status == ADIS16223_SENSOR_STATUS_DISABLED) {
    PRINTF("ADIS16223: sensor not started\n");
    return ADIS16223_ERROR;
  }

  if((type != ADIS16223_VAL_SUPPLY) && 
  	(type != ADIS16223_VAL_TEMP) && 
  	(type != ADIS16223_VAL_X_AXIS) &&
  	(type != ADIS16223_VAL_Y_AXIS) &&
  	(type != ADIS16223_VAL_Z_AXIS)) {
    PRINTF("ADIS16223: invalid value requested\n");
    return ADIS16223_ERROR;
  }

  if(adis16223_read_16(type, &val) == ADIS16223_SUCCESS) {
      return (int)val;
  }
  return ADIS16223_ERROR;
}
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
static int
configure(int type, int value)
{
//  uint8_t buf[2];
//
//  if((type != ADIS16223_ACTIVE) && (type != ADIS16223_RESET) &&
//     (type != ADIS16223_RESOLUTION)) {
//    PRINTF("ADIS16223: option not supported\n");
//    return ADIS16223_ERROR;
//  }
//
//  switch(type) {
//  case ADIS16223_ACTIVE:
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
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Returns the status of the sensor
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return One of the SENSOR_STATUS_xyz defines
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return adis16223_status;
    break;
  default:
    break;
  }
  return ADIS16223_SENSOR_STATUS_DISABLED;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(adis16223_sensor, "ADIS16223", value, configure, status);
/*---------------------------------------------------------------------------*/

