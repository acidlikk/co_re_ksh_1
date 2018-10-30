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
#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_
/*---------------------------------------------------------------------------*/
/* Disable button shutdown functionality */
#define BUTTON_SENSOR_CONF_ENABLE_SHUTDOWN    0
/*---------------------------------------------------------------------------*/
/* Change to match your configuration */

#define FSK 0
#define LRM 1
#define SL_LR 2
#define HSM	3
#define OOK 4

#define BASE 11
#define NODE 12
/*---------------------------------------------------------------------------*/

#define ARPOS_CC1310_SMARTRF_SETTING FSK

#define APROS_DEV_TYPE NODE

//#define CCFG_FORCE_VDDR_HH 0x1

#define APROS_LED_EXIST 1
#define APROS_PERIODIC_PKT 1

#define APROS_TX_POWER_LEVEL 1

#define APROS_RADIO_OFF_MODE 1

#define APROS_UNICAST 0
#define APROS_EXT_FLASH 0
#define APROS_INIT_TIME 0

#define APROS_UART_END_CONF    0x0a
#define APROS_UART_START_CONF  0x20
#define APROS_UART_IGNORE_CHAR  1

#define APROS_CO2_ENABLE 1
#define APROS_TEMP_ENABLE  0
#define APROS_PIR_ENABLE   0

#define IEEE802154_CONF_PANID                 0xABCD

#define LINKADDR_CONF_SIZE                   2

#define RF_CORE_CONF_CHANNEL                  1

#if APROS_RADIO_OFF_MODE
#define NETSTACK_CONF_RDC     nullrdc_driver
#else
#define NETSTACK_CONF_RDC	  contikimac_driver
#endif
#define NETSTACK_CONF_MAC	  csma_driver

#define DOT_15_4G_CONF_FREQUENCY_BAND_ID DOT_15_4G_FREQUENCY_BAND_917

#define CC2650_FAST_RADIO_STARTUP             0

#define CC26XX_UART_CONF_BAUD_RATE    38400
/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
