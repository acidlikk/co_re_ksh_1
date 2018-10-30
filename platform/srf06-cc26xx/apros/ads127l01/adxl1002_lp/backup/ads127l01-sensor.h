/*
 * Copyright (c) 2017, APROS
 * All rights reserved.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * @{
 *
 * \file
 * Header file for the ADS127L01 sensor
 */
/*---------------------------------------------------------------------------*/
#ifndef ADS127L01_SENSOR_H
#define ADS127L01_SENSOR_H
/*---------------------------------------------------------------------------*/

/* Registers */
#define ADS127L01_REG_ID					0x00
#define ADS127L01_REG_CONFIG				0x01
#define ADS127L01_REG_OFC0					0x02
#define ADS127L01_REG_OFC1					0x03
#define ADS127L01_REG_OFC2					0x04
#define ADS127L01_REG_FSC0					0x05
#define ADS127L01_REG_FSC1					0x06
#define ADS127L01_REG_MODE					0x07


/* Commands */
#define ADS127L01_COMMAND_RESET1			0x07
#define ADS127L01_COMMAND_RESET2			0x06
#define ADS127L01_COMMAND_START1			0x09		
#define ADS127L01_COMMAND_START2			0x08
#define ADS127L01_COMMAND_STOP1				0x0B
#define ADS127L01_COMMAND_STOP2				0x0A
#define ADS127L01_COMMAND_RDATA				0x12
#define ADS127L01_COMMAND_RREG				0x20
#define ADS127L01_COMMAND_WREG				0x40


/* Return Values */
#define ADS127L01_ERROR           		  (-1)
#define ADS127L01_SUCCESS            	  true
//#define ADS127L01_RESET_DELAY       15000
//#define ADS127L01_STATUS_BITS_MASK  0x0003
/*---------------------------------------------------------------------------*/
/**
 * \name ADS127L01 driver states
 * @{
 */
#define ADS127L01_SENSOR_STATUS_DISABLED	 	  0
#define ADS127L01_SENSOR_STATUS_ENABLED	  	  1

#define ADS127L01_SENSOR_STATUS_IDLE     	  2
#define ADS127L01_SENSOR_STATUS_BUSY      	  3
/** @} */

typedef enum
{
	WFILTER1_512K, 
	WFILTER1_256K, 
	WFILTER1_128K,
	WFILTER1_64K,
	WFILTER2_512K,
	WFILTER2_256K,
	WFILTER2_128K,
	WFILTER2_64K,
	LLFILTER_512K,
	LLFILTER_128K,
	LLFILTER_32K,
	LLFILTER_8K
} ads127l01_filter;
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor ads127l01_sensor;
/*---------------------------------------------------------------------------*/
void ads127l01_enable();
	
void ads127l01_disable();

void ads127l01_register_read_8(uint8_t variable, uint8_t* rd);

uint8_t ads127l01_register_write_8(uint8_t variable, uint8_t bit_value);
void 
ads127l01_set_filter(const ads127l01_filter filter);
void ads127l01_read_data(uint8_t* rd);

void ads127l01_start();
void
ads127l01_stop();

	
#endif /* ADS127L01_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

