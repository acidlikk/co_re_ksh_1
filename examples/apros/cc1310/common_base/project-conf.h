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
/*---------------------------------------------------------------------------*/
#define FSK 0
#define LRM 1
#define SL_LR 2//not operate
#define HSM	3//not operate
#define OOK 4 

#define BASE 11
#define NODE 12
/*---------------------------------------------------------------------------*/

#define ARPOS_CC1310_SMARTRF_SETTING FSK
#define APROS_CC1310_SMARTRF_EXT_BIAS 0x0

#define APROS_DEV_TYPE BASE
#define CCFG_FORCE_VDDR_HH 0x1
#define APROS_TX_POWER_LEVEL 0
#define APROS_LED_EXIST 1
#define APROS_ACCEL 1

#define APROS_OLD_DF 0

#define APROS_TIMEOUT_ENABLE 0
#define APROS_BASE_BYPASS_FROM_WIRELESS 1

#define APROS_BASE_AUTO_REBOOT 1 // Not Recommend to use this feature.
#if APROS_BASE_AUTO_REBOOT
#define	APROS_BASE_AUTO_REBOOT_PERIOD 		300
#endif
#define PROP_MODE_CONF_RSSI_THRESHOLD 0xBA

#define IEEE802154_CONF_PANID                 0xABCD

#define LINKADDR_CONF_SIZE                   2

//#define RF_CORE_CONF_CHANNEL                  1

#define NETSTACK_CONF_RDC       nullrdc_driver
//#define NETSTACK_CONF_RDC		contikimac_driver
#define NETSTACK_CONF_MAC		csma_driver
//#define NETSTACK_CONF_MAC		nullmac_driver 
#define DOT_15_4G_CONF_FREQUENCY_BAND_ID DOT_15_4G_FREQUENCY_BAND_917

#define CC2650_FAST_RADIO_STARTUP             0

#define CC26XX_UART_CONF_BAUD_RATE    115200

//#define NETSTACK_CONF_MAC		nullmac_driver
//#define NETSTACK_CONF_MAC		tschmac_driver
//#define NETSTACK_CONF_RDC		nordc_driver
//#define PROP_MODE_CONF_RSSI_THRESHOLD 0x9C

/*---------------------------------------------------------------------------*/
#endif /* PROJECT_CONF_H_ */
/*---------------------------------------------------------------------------*/
