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
/** \addtogroup cc26xx-srf-tag
 * @{
 *
 * \defgroup apros-cc13xx-peripherals
 *
 * Defines related to the CC1310 Apros KETI KDHC Smart Sensor
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other peripherals
 *
 * This file can be used as the basis to configure other boards using the
 * CC13xx/CC26xx code as their basis.
 *
 * This file is not meant to be modified by the user.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the TI
 * CC1310 Apros Smart Sensor
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_H_
#define BOARD_H_
/*---------------------------------------------------------------------------*/
#include "ioc.h"
/*---------------------------------------------------------------------------*/
/**
 * \name LED configurations
 *
 * Those values are not meant to be modified by the user
 * @{
 */
/* Some files include leds.h before us, so we need to get rid of defaults in
 * leds.h before we provide correct definitions */
#undef LEDS_GREEN
#undef LEDS_YELLOW
#undef LEDS_RED
#undef LEDS_CONF_ALL

#define LEDS_RED       1
#define LEDS_BLUE      2
#define LEDS_GREEN     4
#define LEDS_CYAN      8  // BLUE + GREEN
#define LEDS_YELLOW    16 // GREEN + RED
#define LEDS_MAGENTA   32 // RED + BLUE
#define LEDS_WHITE     64 // RED + BLUE + GREEN
#define LEDS_ORANGE    16

#define LEDS_CONF_ALL  127

/* Notify various examples that we have LEDs */
#define PLATFORM_HAS_LEDS        1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name LED IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_LED_1          IOID_6 // RED:   LED_OUT_2
#define BOARD_IOID_LED_2          IOID_1 // GREEN: LED_OUT_1
#define BOARD_IOID_LED_3          IOID_7 // BLUE:  LED_OUT_3
#define BOARD_LED_1               (1 << BOARD_IOID_LED_1)
#define BOARD_LED_2               (1 << BOARD_IOID_LED_2)
#define BOARD_LED_3               (1 << BOARD_IOID_LED_3)
#define BOARD_LED_ALL             (BOARD_LED_1 | BOARD_LED_2 | BOARD_LED_3)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name UART IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_UART_RX        IOID_19
#define BOARD_IOID_UART_TX        IOID_3
#define BOARD_IOID_UART_CTS       IOID_UNUSED
#define BOARD_IOID_UART_RTS       IOID_UNUSED
#define BOARD_UART_RX             (1 << BOARD_IOID_UART_RX)
#define BOARD_UART_TX             (1 << BOARD_IOID_UART_TX)
#define BOARD_UART_CTS            (1 << BOARD_IOID_UART_CTS)
#define BOARD_UART_RTS            (1 << BOARD_IOID_UART_RTS)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Button IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_KEY_USER       IOID_18
#define BOARD_KEY_USER            (1 << BOARD_IOID_KEY_USER)
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Power IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SENSOR_POWER_ENABLE IOID_26
#define BOARD_SENSOR_POWER_ENABLE      (1 << BOARD_IOID_SENSOR_POWER_ENABLE)
/** @} */

/*---------------------------------------------------------------------------*/
/**
 * \name ADXL1002 IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ADXL1002_ST       		//
#define BOARD_IOID_ADXL1002_STANDBY       	IOID_22

#define BOARD_ADXL1002_ST            	 (1 << BOARD_IOID_ADXL1002_ST)
#define BOARD_ADXL1002_STANBY            (1 << BOARD_IOID_ADXL1002_STANDBY)


/*---------------------------------------------------------------------------*/
/**
 * \name ADS127L01 SPI IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ADS127L01_SPI_SCK        IOID_30
#define BOARD_IOID_ADS127L01_SPI_MOSI       IOID_29
#define BOARD_IOID_ADS127L01_SPI_MISO       IOID_28
#define BOARD_IOID_ADS127L01_SPI_CS		  	IOID_27
#define BOARD_IOID_ADS127L01_SPI_DRDY       IOID_25 //DATA READY PIN; it's pull up 
#define BOARD_IOID_ADS127L01_DAISYIN		//Always Low

#define BOARD_ADS127L01_SPI_SCK             (1 << BOARD_IOID_ADS127L01_SPI_SCK)
#define BOARD_ADS127L01_SPI_MOSI            (1 << BOARD_IOID_ADS127L01_SPI_MOSI)
#define BOARD_ADS127L01_SPI_MISO            (1 << BOARD_IOID_ADS127L01_SPI_MISO)
#define BOARD_ADS127L01_SPI_CS			  	(1 << BOARD_IOID_ADS127L01_SPI_CS)
#define BOARD_ADS127L01_SPI_DRDY			(1 << BOARD_IOID_ADS127L01_SPI_DRDY)

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ADS127L01 Control Logic IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ADS127L01_RESET			IOID_23
#define BOARD_IOID_ADS127L01_START			//Always Low
#define BOARD_IOID_ADS127L01_FILTER0		IOID_12
#define BOARD_IOID_ADS127L01_FILTER1		IOID_13
#define BOARD_IOID_ADS127L01_OSR0			IOID_14
#define BOARD_IOID_ADS127L01_OSR1			IOID_15

#define BOARD_ADS127L01_RESET				(1 << BOARD_IOID_ADS127L01_RESET)
#define BOARD_ADS127L01_FILTER0				(1 << BOARD_IOID_ADS127L01_FILTER0)
#define BOARD_ADS127L01_FILTER1				(1 << BOARD_IOID_ADS127L01_FILTER1)
#define BOARD_ADS127L01_OSR0				(1 << BOARD_IOID_ADS127L01_OSR0)
#define BOARD_ADS127L01_OSR1				(1 << BOARD_IOID_ADS127L01_OSR1)

/*---------------------------------------------------------------------------*/
/**
 * \name External RTC for ADS127L01 IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_ADS127L01_OSC			IOID_2

#define BOARD_ADS127L01_OSC				(1 << BOARD_IOID_ADS127L01_OSC)
/*---------------------------------------------------------------------------*/
/**
 * \name External RTC for CC1310 IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_RTC_32K 				   IOID_11
#define BOARD_IOID_RTC_INT				   IOID_24

#define BOARD_RTC_32K						(1 << BOARD_IOID_RTC_32K)
#define BOARD_RTC_INT						(1 << BOARD_IOID_RTC_INT)
/*---------------------------------------------------------------------------*/
/**
 * \name External Memory SPI IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SPI_SCK        IOID_10
#define BOARD_IOID_SPI_MOSI       IOID_9
#define BOARD_IOID_SPI_MISO       IOID_8
#define BOARD_SPI_SCK             (1 << BOARD_IOID_SPI_SCK)
#define BOARD_SPI_MOSI            (1 << BOARD_IOID_SPI_MOSI)
#define BOARD_SPI_MISO            (1 << BOARD_IOID_SPI_MISO)
/** @} */

/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name External flash IOID mapping
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_FLASH_ENABLE	  IOID_21
#define BOARD_IOID_FLASH_CS       IOID_20
#define BOARD_FLASH_CS            (1 << BOARD_IOID_FLASH_CS)
#define BOARD_IOID_SPI_CLK_FLASH  IOID_10
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief I2C IOID mappings
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_SCL            IOID_4
#define BOARD_IOID_SDA            IOID_5
#define BOARD_IOID_SCL_HP         IOID_UNUSED
#define BOARD_IOID_SDA_HP         IOID_UNUSED
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \brief Remaining pins
 *
 * Those values are not meant to be modified by the user
 * @{
 */
#define BOARD_IOID_DIO17          IOID_17
#define BOARD_IOID_DIO18          IOID_18
#define BOARD_IOID_DIO19          IOID_19
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "APROS ADXL1002_LP"
#define APROS_LP 1

/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_H_ */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
