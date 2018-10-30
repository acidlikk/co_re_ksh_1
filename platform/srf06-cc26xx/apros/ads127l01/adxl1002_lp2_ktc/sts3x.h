/*
 * Copyright (c) 2016, APROS
 * All rights reserved.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 * Header file for the sts3x
 */
/*---------------------------------------------------------------------------*/
#ifndef STS3X_H
#define STS3X_H
/*---------------------------------------------------------------------------*/

/* Sensor I2C address */
#define STS3X_ADDR              0x4A // ADDR is low
#define STS3X_GEN_ADDR			0x00 // when gen_reset is called

/* Register */
//Single Shot
//Clock Stretching Enable
#define STS3X_SS_CS_HIGH	    0x2C06
#define STS3X_SS_CS_MEDIUM		0x2C0D
#define STS3X_SS_CS_LOW			0x2C10
//Clock Stretching DISABLE
#define STS3X_SS_HIGH			0x2400
#define STS3X_SS_MEDIUM			0x240B
#define STS3X_SS_LOW			0x2416

//Periodic
//0.5mps measurement
#define STS3X_P_0P5_HIGH		0x2032
#define STS3X_P_0P5_MEDIUM		0x2024
#define STS3X_P_0P5_LOW			0x202F

#define STS3X_P_1_HIGH			0x2130
#define STS3X_P_1_MEDIUM		0x2126
#define STS3X_P_1_LOW			0x212D

#define STS3X_P_2_HIGH			0x2236
#define STS3X_P_2_MEDIUM		0x2220
#define STS3X_P_2_LOW			0x222B

#define STS3X_P_4_HIGH			0x2334
#define STS3X_P_4_MEDIUM		0x2322
#define STS3X_P_4_LOW			0x2329

#define STS3X_P_10_HIGH			0x2737
#define STS3X_P_10_MEDIUM		0x2721
#define STS3X_P_10_LOW			0x272A

#define STS3X_FETCH_DATA		0xE000
#define STS3X_BREAK 			0x3093

//Command
#define STS3X_RESET				0x30A2
#define STS3X_GEN_RESET			0x06
#define STS3X_HEATER_ENABLE	    0x306D
#define STS3X_HEATER_DISABLE	0x3066
#define STS3X_STATUS		    0xF32D
#define STS3X_CLEAR_STATUS		0x3041

/* Status */
#define STS3X_ERROR             (-1)
#define STS3X_SUCCESS           0x01


/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
void sts3x_init();
void sts3x_close();
uint8_t sts3x_single_shot(uint8_t *rd, uint16_t option);
uint8_t sts3x_reg_read(uint16_t command, uint8_t *rd);
uint8_t sts3x_reg_write(uint16_t command);
/*---------------------------------------------------------------------------*/
#endif /* STS3X_H */

