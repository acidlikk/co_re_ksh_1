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
/**
 * \addtogroup openmote-sensors
 * @{
 *
 * \defgroup openmote-adxl346-sensor ADXL346 acceleration sensor
 * @{
 *
 * \file
 * ADXL346 acceleration sensor driver header file
 *
 * \author
 * Pere Tuset <peretuset@openmote.com>
 */
 // \mod Sanghyun KIM for ADXL355

/*---------------------------------------------------------------------------*/
#ifndef ADXL355_H_
#define ADXL355_H_
#include <stdio.h>
#include "lib/sensors.h"
#if USE_COMM==I2C__
#include "board-i2c.h"
#endif
#if USE_COMM==SPI__
#include "adxl355-spi-control.h"
#endif
/*---------------------------------------------------------------------------*/
/* Used in accm_read_axis(), eg accm_read_axis(X_AXIS) */
enum ADXL355_AXIS {
  X_AXIS = 0,
  Y_AXIS = 3,
  Z_AXIS = 6,
};
/* -------------------------------------------------------------------------- */
/* Reference definitions, should not be changed */
/* adxl355 slave address */
#define ADXL355_ADDR_ASEL_L		0x1D	  
//#define ADXL355_ADDR_ASEL_H     0x53

#define ADXL355_DEVID_VALUE     0xAD

#define ADXL355_READ_ADDR(x) ((x << 1) | 0x01)
#define ADXL355_WRITE_ADDR(x) ((x << 1))
/* ADXL355 registers */
// Read Only
#define ADXL355_DEVID_AD        0x00
#define ADXL355_DEVID_MST		0x01
#define ADXL355_PARTID			0x02
#define ADXL355_REVID			0x03
#define ADXL355_STATUS			0x04
#define ADXL355_FIFO_ENTRIES	0x05
#define ADXL355_TEMP2			0x06
#define ADXL355_TEMP1			0x07
#define ADXL355_XDATA3			0x08
#define ADXL355_XDATA2			0x09
#define ADXL355_XDATA1			0x0A
#define ADXL355_YDATA3			0x0B
#define ADXL355_YDATA2			0x0C
#define ADXL355_YDATA1			0x0D
#define ADXL355_ZDATA3			0x0E
#define ADXL355_ZDATA2			0x0F
#define ADXL355_ZDATA1			0x10
#define ADXL355_FIFO_DATA		0x11

//Read and Write 
#define ADXL355_OFFSET_X_H		0x1E
#define ADXL355_OFFSET_X_L		0x1F
#define ADXL355_OFFSET_Y_H		0x20
#define ADXL355_OFFSET_Y_L		0x21
#define ADXL355_OFFSET_Z_H		0x22
#define ADXL355_OFFSET_Z_L		0x23
#define ADXL355_ACT_EN			0x24
#define ADXL355_ACT_THRESH_H	0x25
#define ADXL355_ACT_THRESH_L	0x26
#define ADXL355_ACT_COUNT		0x27
#define ADXL355_FILTER			0x28
#define ADXL355_FIFO_SAMPLES	0x29
#define ADXL355_INT_MAP			0x2A
#define ADXL355_SYNC			0x2B
#define ADXL355_RANGE			0x2C
#define ADXL355_POWER_CTL		0x2D
#define ADXL355_SELF_TEST		0x2E

//Write Only
#define	ADXL355_RESET			0x2F

/* g-range for DATA_FORMAT register */
#define ADXL355_RANGE_2G    	0x01
#define ADXL355_RANGE_4G    	0x02
#define ADXL355_RANGE_8G  		0x03

#define ADXL355_READ_X          0x00
#define ADXL355_READ_Y          0x03
#define ADXL355_READ_Z          0x06
/* -------------------------------------------------------------------------- */
#define ADXL355_SUCCESS         0x00
#define ADXL355_ERROR           (-1)
/* -------------------------------------------------------------------------- */
#define ADXL355_SENSOR         "ADXL355 sensor"
/* -------------------------------------------------------------------------- */
extern const struct sensors_sensor adxl355;
/* -------------------------------------------------------------------------- */
#endif /* ifndef ADXL355_H_ */

