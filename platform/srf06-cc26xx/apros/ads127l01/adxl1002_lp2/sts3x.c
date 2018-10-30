/*---------------------------------------------------------------------------*/
/**
 * \addtogroup thermometer
 * @{
 *
 * \file
 *  Driver for sts3x
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/ctimer.h"

#include "sts3x.h"
#include "board-i2c.h"
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
/* selection/deselection */
#define STS3X_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, STS3X_ADDR)
#define STS3X_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define SWAP(v) ((LO_UINT16(v) << 8) | HI_UINT16(v))
/*---------------------------------------------------------------------------*/
/*uint8_t crc8( uint8_t *addr, uint8_t len)
{
     uint8_t crc=0;
     
     for (uint8_t i=0; i<len;i++) 
     {
           uint8_t inbyte = addr[i];
           for (uint8_t j=0;j<8;j++) 
           {
                 uint8_t mix = (crc ^ inbyte) & 0x01;
                 crc >>= 1;
                 if (mix) 
                       crc ^= 0x8C;
                 
                 inbyte >>= 1;
           }
     }
     return crc;
}*/
/*---------------------------------------------------------------------------*/
uint8_t
sts3x_single_shot(uint8_t *rd, uint16_t option){
	static uint8_t wbuf[2], rbuf[3];

	wbuf[0] = (option>>8)&0xff;
	wbuf[1] = option&0xff;

	STS3X_SELECT();

	/*if(board_i2c_write_read(wbuf, 2, rbuf, 3) == STS3X_SUCCESS){
		leds_toggle(LEDS_BLUE);
		memcpy(rd, rbuf, 2); //except CRC 
		return STS3X_SUCCESS;
	}else{
		leds_toggle(LEDS_RED);
	}*/
	if(board_i2c_write(wbuf, 2) == STS3X_SUCCESS) {
		switch(option){
			case STS3X_SS_CS_HIGH:
			case STS3X_SS_CS_MEDIUM:
			case STS3X_SS_CS_LOW:
				if(board_i2c_read(rbuf, 3) == STS3X_SUCCESS){
					memcpy(rd, rbuf, 2); //except CRC 
					STS3X_DESELECT();

			    	return STS3X_SUCCESS;
				}
			break;
			case STS3X_SS_HIGH:
			case STS3X_SS_MEDIUM:
			case STS3X_SS_LOW:
				//TODO
				//board_i2c_read(rbuf, 3);
				
			break;
		}
	}
	
	STS3X_DESELECT();
	return STS3X_ERROR;
}
/*---------------------------------------------------------------------------*/
//TODO::periodic measurement

/*---------------------------------------------------------------------------*/
uint8_t
sts3x_reg_read(uint16_t command, uint8_t *rd)
{
  	static uint8_t wbuf[2], rbuf[3];
	
	//wbuf[0] = reg_addr;
	wbuf[0] = (command>>8)&0xff; 
	wbuf[1] = command&0xff;
		
   	STS3X_SELECT();

  	if(board_i2c_write_read(wbuf, 2, rbuf, 3) == STS3X_SUCCESS) {
		memcpy(rd, rbuf, 2); //except CRC 
		STS3X_DESELECT();
    	return STS3X_SUCCESS;
  	}
  	STS3X_DESELECT();

  	return STS3X_ERROR;
}
/*---------------------------------------------------------------------------*/
uint8_t
sts3x_reg_write(uint16_t command)
{
	static uint8_t wbuf[2];
		
	//buf[0] = reg_addr;
	wbuf[0] = (command>>8)&0xff; 
	wbuf[1] = command&0xff;

	STS3X_SELECT();

	if(board_i2c_write(wbuf, 2) == STS3X_SUCCESS){
		STS3X_DESELECT();
		return STS3X_SUCCESS;
	}
	STS3X_DESELECT();

	return STS3X_ERROR;
}
/** @} */

void sts3x_init(){
	STS3X_SELECT();
}
void sts3x_close(){
	STS3X_DESELECT();
}

