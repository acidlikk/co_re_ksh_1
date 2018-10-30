/*
 * Copyright (c) 2016, APROS
 * All rights reserved.
 *
 */
/*---------------------------------------------------------------------------*/
/**

 * \file
 * Header file for the M41T64
 */
/*---------------------------------------------------------------------------*/
#ifndef M41T64_M41T64_H
#define M41T64_M41T64_H
/*---------------------------------------------------------------------------*/

/* Sensor I2C address */
#define M41T64_ADDR              0x68

/* Registers */
// All time value is described by BCD format.
#define M41T64_REG_00			  0x00   //[7:4] 0.1s [3:0] 0.01s
#define M41T64_REG_01			  0x01   //[7]ST [6:4] 10s [3:0] 1s
#define M41T64_REG_02		      0x02   //[6:4]10min [3:0] 1min
#define M41T64_REG_03			  0x03   //[7]RS3 [6]RS2 [5]RS1 [4]RS0 [2:0] day of week
#define M41T64_REG_04			  0x04   
#define M41T64_REG_05			  0x05	
#define M41T64_REG_06			  0x06
#define M41T64_REG_07			  0x07
#define M41T64_REG_08			  0x08
#define M41T64_REG_09			  0x09
#define M41T64_REG_0A			  0x0A
#define M41T64_REG_0B			  0x0B
#define M41T64_REG_0C			  0x0C
#define M41T64_REG_0D			  0x0D
#define M41T64_REG_0E			  0x0E
#define M41T64_REG_0F			  0x0F

/* Status */
#define M41T64_ERROR             (-1)
#define M41T64_SUCCESS           0x01
#define M41T64_RESET_DELAY       15000
#define M41T64_STATUS_BITS_MASK  0x0003

/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
void m41t64_init();
void m41t64_close();
int m41t64_reg_read(uint8_t reg_address, uint8_t *rd);
int m41t64_reg_write(uint8_t reg_address, uint8_t value);
/*---------------------------------------------------------------------------*/
#endif /* M41T64_M41T64_H */

