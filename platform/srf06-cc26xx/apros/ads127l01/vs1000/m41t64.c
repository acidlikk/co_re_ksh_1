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
 * \addtogroup external-rtc
 * @{
 *
 * \file
 *  Driver for m41t64
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "sys/ctimer.h"

#include "m41t64.h"
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
#define M41T64_SELECT()     board_i2c_select(BOARD_I2C_INTERFACE_0, M41T64_ADDR)
#define M41T64_DESELECT()   board_i2c_deselect()
/*---------------------------------------------------------------------------*/
/* Byte swap of 16-bit register value */
#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

#define SWAP(v) ((LO_UINT16(v) << 8) | HI_UINT16(v))
/*---------------------------------------------------------------------------*/
int
m41t64_reg_read(uint8_t reg_addr, uint8_t *rd)
{
  static uint8_t buf;
	
  M41T64_SELECT();

  if(board_i2c_write_read(&reg_addr, 1, &buf, 1) == M41T64_SUCCESS) {
    *rd = buf;
    return M41T64_SUCCESS;
  }

  return M41T64_ERROR;
}
/*---------------------------------------------------------------------------*/
int
m41t64_reg_write(uint8_t reg_addr, uint8_t value)
{
	static uint8_t buf[2];
		
	buf[0] = reg_addr;
	buf[1] = value;

	M41T64_SELECT();

	return board_i2c_write(buf, 2);
}
/** @} */

void m41t64_init(){
	M41T64_SELECT();
}
void m41t64_close(){
	M41T64_DESELECT();
}
