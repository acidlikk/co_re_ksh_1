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
 * \addtogroup apros-common-peripherals
 * @{
 *
 * \file
 *  Board-initialisation for the apros.
 */
/*---------------------------------------------------------------------------*/
#include "contiki-conf.h"
#include "lib/sensors.h"
#include "ti-lib.h"
#include "lpm.h"
#include "prcm.h"
#include "hw_sysctl.h"
#include "board-peripherals.h"
#include "../ads127l01-sensor.h"
#include "../ads127l01-spi-control.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
/*---------------------------------------------------------------------------*/
static void
wakeup_handler(void)
{
  /* Turn on the PERIPH PD */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_PERIPH);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_PERIPH)
        != PRCM_DOMAIN_POWER_ON));
}
/*---------------------------------------------------------------------------*/
/*
 * Declare a data structure to register with LPM.
 * We don't care about what power mode we'll drop to, we don't care about
 * getting notified before deep sleep. All we need is to be notified when we
 * wake up so we can turn power domains back on
 */
LPM_MODULE(apros_module, NULL, NULL, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static void
configure_unused_pins(void)
{
//  uint32_t pins[] = {
//    BOARD_IOID_DIO26,
//    BOARD_IOID_DIO27, BOARD_IOID_DIO30,
//    IOID_UNUSED
//  }; 
//
//  uint32_t *pin;
//
//  for(pin = pins; *pin != IOID_UNUSED; pin++) {
//    ti_lib_ioc_pin_type_gpio_input(*pin);
//    ti_lib_ioc_io_port_pull_set(*pin, IOC_IOPULL_DOWN);
//  }
}
/*---------------------------------------------------------------------------*/
static void
configure_used_pins(void)
{
	/* SPI pin configuration */
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_SPI_SCK);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_SPI_SCK, IOC_IOPULL_DOWN);
		
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_SPI_MOSI);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_SPI_MOSI, IOC_IOPULL_DOWN);

	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADS127L01_SPI_MISO);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_SPI_MISO, IOC_IOPULL_UP);

	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_SPI_CS);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_SPI_CS, IOC_IOPULL_UP);

	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_SPI_DRDY);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_SPI_DRDY, IOC_IOPULL_UP);
	
	/* Control Logic pin configuration */
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_FILTER0);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_FILTER0, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_FILTER1);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_FILTER1, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_OSR0);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_OSR0, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_OSR1);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_OSR1, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_VS1000_ST);
	//ti_lib_ioc_io_port_pull_set(BOARD_IOID_VS1000_ST, IOC_IOPULL_UP);
	ti_lib_gpio_clear_dio(BOARD_IOID_VS1000_ST);
		
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_VS1000_RESET);
	//ti_lib_ioc_io_port_pull_set(BOARD_IOID_VS1000_RESET, IOC_IOPULL_UP);
	ti_lib_gpio_set_dio(BOARD_IOID_VS1000_RESET);
	
	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_RTC_32K);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_RTC_32K, IOC_IOPULL_DOWN);
	//ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_ADS127L01_OSC);
	//ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADS127L01_OSC, IOC_IOPULL_UP);
}
/*---------------------------------------------------------------------------*/
void
board_init()
{
  /* Disable global interrupts */
  uint8_t int_disabled = ti_lib_int_master_disable();

  /* Turn on relevant PDs */
  wakeup_handler();

  /* Enable GPIO peripheral */
  ti_lib_prcm_peripheral_run_enable(PRCM_PERIPH_GPIO);

  /* Apply settings and wait for them to take effect */
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());
  
  ads127l01_spi_init();
  ext_flash_init();
  
  lpm_register_module(&apros_module);

  /* For unsupported peripherals, select a default pin configuration */
  configure_unused_pins();
  configure_used_pins();

  /* Re-enable interrupt if initially enabled. */
  if(!int_disabled) {
    ti_lib_int_master_enable();
  }
}
/*---------------------------------------------------------------------------*/
/** @} */
