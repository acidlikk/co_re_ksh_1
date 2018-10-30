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
 * Header file for the ADIS16223 sensor
 */
/*---------------------------------------------------------------------------*/
#ifndef ADIS16223_SENSOR_H
#define ADIS16223_SENSOR_H
/*---------------------------------------------------------------------------*/

/* Registers */
#define ADIS16223_FLASH_CNT 			  0x00

#define ADIS16223_NULL_X				  0x02
#define ADIS16223_NULL_Y				  0x04
#define ADIS16223_NULL_Z				  0x06

#define ADIS16223_CAPT_SUPPLY		 	  0x0A
#define ADIS16223_CAPT_TEMP			      0x0C
#define ADIS16223_CAPT_PEAKX			  0x0E
#define ADIS16223_CAPT_PEAKY			  0x10
#define ADIS16223_CAPT_PEAKZ			  0x12
#define ADIS16223_CAPT_BUFFX			  0x14
#define ADIS16223_CAPT_BUFFY			  0x16
#define ADIS16223_CAPT_BUFFZ		      0x18
#define ADIS16223_CAPT_PNTR			      0x1A
#define ADIS16223_CAPT_CTRL     		  0x1C
#define ADIS16223_CAPT_PRD			      0x1E

#define ADIS16223_ALM_MAGX			      0x20
#define ADIS16223_ALM_MAGY			  	  0x22
#define ADIS16223_ALM_MAGZ			  	  0x24
#define ADIS16223_ALM_MAGS			  	  0x26
#define ADIS16223_ALM_CTRL			  	  0x28

#define ADIS16223_GPIO_CTRL			  	  0x32
#define ADIS16223_MSC_CTRL			  	  0x34
#define ADIS16223_DIO_CTRL			  	  0x36
#define ADIS16223_AVG_CNT 			 	  0x38

#define ADIS16223_DIAG_STAT			  	  0x3A
#define ADIS16223_GLOB_CMD			  	  0x3E

#define ADIS16223_LOT_ID1			  	  0x52
#define ADIS16223_LOT_ID2			  	  0x54
#define ADIS16223_PROD_ID			  	  0x56
#define ADIS16223_SERIAL_NUM			  0x58

/* Bit Description */
/*
 * This description can be replaced with bit structures.
 */
/* CAPT_CTRL */
/* LOWER_BYTE */
#define ADIS16223_CAPT_CTRL_PD_ENABLE 	  	0x02 // 0000 0010 Power-down beteen capture events 

#define ADIS16223_CAPT_CTRL_MODE_MANUAL   	0x00 // 0000 0000 Manual Capture
#define ADIS16223_CAPT_CTRL_MODE_AUTO     	0x04 // 0000 0100 Automatic Capture
#define ADIS16223_CAPT_CTRL_MODE_EVENT    	0x08 // 0000 1000 Event Capture
#define ADIS16223_CAPT_CTRL_MODE_EXTENDED 	0x0C // 0000 1100 EXtended Capture

#define ADIS16223_CAPT_CTRL_EVT_SAMPLE_64 	0x00 // 0000 0000  Pre-event Capture Length == 64 for event mode 
#define ADIS16223_CAPT_CTRL_EVT_SAMPLE_128  0x10 // 0001 0000 
#define ADIS16223_CAPT_CTRL_EVT_SAMPLE_256  0x20 // 0010 0000 
#define ADIS16223_CAPT_CTRL_EVT_SAMPLE_512  0x30 // 0011 0000 

#define ADIS16223_CAPT_CTRL_STORE_FLASH_ALM 0x40 // 0100 0000  automatically store capture buffers to flash upon alarm trigger

#define ADIS16223_CAPT_CTRL_FILTER			0x80 // 1000 0000  Band-pass filter

/* UPPER_BYTE */
#define ADIS16223_CAPT_CTRL_EXT_CH_X		0x00 // 0000 0000  Extended mode channel selection : x-axis
#define ADIS16223_CAPT_CTRL_EXT_CH_Y		0x01 // 0000 0001  
#define ADIS16223_CAPT_CTRL_EXT_CH_Z		0x02 // 0000 0010 

/* CAPT_PRD */ 
/*
 * CAPT_PRD provides configuration for automatic mode 
 *
 */
/* LOWER_BYTE */
#define ADIS16223_CAPT_PRD_DATA_BIT 	    0x01 /* xxxx xxxx    
												  * usage : ADIS16223_CAPT_PRD_DATA_BITS * x
												  * x min : 1,  max : 0xff == 255 
												  */

/* UPPER BYTE */
#define ADIS16223_CAPT_PRD_SCALE_SECOND		0x00 // 0000 0000  second scale
#define ADIS16223_CAPT_PRD_SCALE_MINUTE		0x01 // 0000 0001  minute scale
#define ADIS16223_CAPT_PRD_SCALE_HOUR		0x02 // 0000 0010  hour scale

/* ALM_MAGX */
//TODO
/* ALM_MAGY */
//TODO
/* ALM_MAGZ */
//TODO

/* ALM_MAGS */
//TODO

/* ALM_CTRL */
//TODO

/* GLOB_CMD */ 
/*
 * GLOB_CMD provides an array of single-write commands for convenience. 
 */ 
/* LOWER_BYTE */
#define ADIS16223_GLOB_CMD_AUTONULL		  0x01  // 0000 0001 autonull ,execution time 936ms
#define ADIS16223_GLOB_CMD_PD			  0x02	// 0000 0010 power-down
#define ADIS16223_GLOB_CMD_SELF_TEST	  0x04	// 0000 0100 self-test, result in DIAG_STAT[5], 33ms
#define ADIS16223_GLOB_CMD_FACTORY		  0x08  // 0000 1000 Restore factory register settings and clear the capture buffers, 339ms
#define ADIS16223_GLOB_CMD_CLEAR_STAT	  0x10  // 0001 0000 Clear DIAG_STAT register, 0.035ms
#define ADIS16223_GLOB_CMD_FLASH_TEST	  0x20  // 0010 0000 Flash test, compare sum of flash memory with factory value, 10.5ms
#define ADIS16223_GLOB_CMD_SW_RESET		  0x80 	// 1000 0000 54ms

/* UPPER_BYTE */
#define ADIS16223_GLOB_CMD_CLEAR_CB		  0x01	// 0000 0001 Clear capture buffers 0.84ms 
#define ADIS16223_GLOB_CMD_RESET_PNTR	  0x04	// 0000 0100 Set CAPT_PNTR = 0x0000, 0.035ms
#define ADIS16223_GLOB_CMD_CAPT_TR		  0x08	// 0000 1000 Capture mode start/stop
#define ADIS16223_GLOB_CMD_TO_FLASH		  0x10	// 0001 0000 Copy capture data and settings to flash memory, 339ms(no capture), 509ms(capture)
#define ADIS16223_GLOB_CMD_FROM_FLASH	  0x20  // 0010 0000 Restroe capture data ans settings from flash memory, 0.98ms(no capture), 7.0ms(capture)

/* DIO_CTRL */
/* LOWER_BYTE */
#define ADIS16223_DIO_CTRL_DIO1_POL_L	  0x00 // 0000 0000 DIO1 line polarity active LOW
#define ADIS16223_DIO_CTRL_DIO1_POL_H	  0x01 // 0000 0001 HIGH 

#define ADIS16223_DIO_CTRL_DIO2_POL_L     0x00 // 0000 0000 DIO2 line polarity active LOW
#define ADIS16223_DIO_CTRL_DIO2_POL_H	  0x02 // 0000 0010 HIGH

#define ADIS16223_DIO_CTRL_DIO1_FUNC_GPIO		0x00 // 0000 0000 DIO1 GPIO
#define ADIS16223_DIO_CTRL_DIO1_FUNC_ARM		0x04 // 0000 0100 DIO1 Alarm indicator output
#define ADIS16223_DIO_CTRL_DIO1_FUNC_CAPT_TR	0x08 // 0000 1000	DIO1 capture trigger input
#define ADIS16223_DIO_CTRL_DIO1_FUNC_BUSY_IND 	0x0C // 0000 1100 DIO1 busy indicator output

#define ADIS16223_DIO_CTRL_DIO2_FUNC_GPIO 		0x00 // 0000 0000 DIO2 GPIO
#define ADIS16223_DIO_CTRL_DIO2_FUNC_ARM		0x10 // 0001 0000 DIO2 Alarm indicator output
#define ADIS16223_DIO_CTRL_DIO2_FUNC_CAPT_TR	0x20 // 0010 0000 DIO2 capture trigger input
#define ADIS16223_DIO_CTRL_DIO2_FUNC_BUSY_IND   0x30 // 0011 0000 DIO2 busy indicator output

/* GPIO_CTRL */
//TODO

/* DIAG_STAT */
//TODO

/* MSC_CTRL */
//TODO

/* AVG_CNT */
/* LOWER_BYTE */
#define ADIS16223_AVG_CNT_D0 		0x00
#define ADIS16223_AVG_CNT_D1 		0x01
#define ADIS16223_AVG_CNT_D2 		0x02
#define ADIS16223_AVG_CNT_D3 		0x03
#define ADIS16223_AVG_CNT_D4 		0x04
#define ADIS16223_AVG_CNT_D5 		0x05
#define ADIS16223_AVG_CNT_D6 		0x06
#define ADIS16223_AVG_CNT_D7 		0x07
#define ADIS16223_AVG_CNT_D8 		0x08
#define ADIS16223_AVG_CNT_D9 		0x09
#define ADIS16223_AVG_CNT_D10 		0x0A

/* Status & Configure */
#define ADIS16223_UPPER_BYTE			  0x01
#define ADIS16223_LOWER_BYTE			  0x00

#define ADIS16223_ACTIVE            	  SENSORS_ACTIVE
#define ADIS16223_RESET             	  0x01
#define ADIS16223_RESOLUTION        	  0x02

#define ADIS16223_VAL_TEMP          	  ADIS16223_CAPT_TEMP
#define ADIS16223_VAL_SUPPLY			  ADIS16223_CAPT_SUPPLY
#define ADIS16223_VAL_X_AXIS              ADIS16223_CAPT_BUFFX
#define ADIS16223_VAL_Y_AXIS			  ADIS16223_CAPT_BUFFY
#define ADIS16223_VAL_Z_AXIS			  ADIS16223_CAPT_BUFFZ

#define ADIS16223_ERROR           		  (-1)
#define ADIS16223_SUCCESS            	  true
//#define ADIS16223_RESET_DELAY       15000
//#define ADIS16223_STATUS_BITS_MASK  0x0003
/*---------------------------------------------------------------------------*/
/**
 * \name ADIS16223 driver states
 * @{
 */
#define ADIS16223_SENSOR_STATUS_DISABLED	 	  0
#define ADIS16223_SENSOR_STATUS_ENABLED	  	  1

#define ADIS16223_SENSOR_STATUS_IDLE     	  2
#define ADIS16223_SENSOR_STATUS_BUSY      	  3
/** @} */
/*---------------------------------------------------------------------------*/
extern const struct sensors_sensor adis16223_sensor;
/*---------------------------------------------------------------------------*/
uint8_t adis16223_enable();
	
uint8_t adis16223_disable();

void adis16223_read_8(uint8_t variable, uint8_t rd[][2]);

uint8_t adis16223_read_16(uint8_t variable, uint16_t *rd);

uint8_t adis16223_write(uint8_t variable, uint8_t ul_byte_addr, uint8_t bit_value);

uint8_t adis16223_dio_configure(uint8_t dio1_pol, uint8_t dio1_func, uint8_t dio2_pol, uint8_t dio2_func);

uint8_t adis16223_avg_cnt_configure(uint8_t d_value);

void adis16223_capture_offset_reset();

uint8_t adis16223_capture_configure(uint8_t power_ctrl, uint8_t mode, uint8_t evt_sample, uint8_t auto_flash, uint8_t band_filter, uint8_t ext_ch);

uint8_t adis16223_capture_clear_buffer();
	
uint8_t adis16223_capture_trigger();
	
uint8_t adis16223_capture_pntrmove(uint16_t index);
	
uint8_t adis16223_capture_pntrreset();
	
#endif /* ADIS16223_SENSOR_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */

