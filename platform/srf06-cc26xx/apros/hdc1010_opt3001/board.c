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
#include "board-i2c.h"

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
static void
shutdown_handler(uint8_t mode)
{
  if(mode == LPM_MODE_SHUTDOWN) {
#if APROS_LIGHT_ENABLE
    SENSORS_DEACTIVATE(opt_3001_sensor);
#endif
    SENSORS_DEACTIVATE(hdc_1010_sensor);
  }

  /* In all cases, stop the I2C */
  //board_i2c_shutdown();
}
/*---------------------------------------------------------------------------*/
 /*
 * Declare a data structure to register with LPM.
 * We don't care about what power mode we'll drop to, we don't care about
 * getting notified before deep sleep. All we need is to be notified when we
 * wake up so we can turn power domains back on for I2C and SSI, and to make
 * sure everything on the board is off before CM3 shutdown.
 */
LPM_MODULE(apros_module, NULL, shutdown_handler, wakeup_handler, LPM_DOMAIN_NONE);
/*---------------------------------------------------------------------------*/
static void
configure_unused_pins(void)
{
  uint32_t pins[] = {
  	BOARD_IOID_DIO11,
    BOARD_IOID_DIO12,
    BOARD_IOID_DIO15,
    BOARD_IOID_DIO16,
    BOARD_IOID_DIO17, 
    BOARD_IOID_DIO18,
    BOARD_IOID_DIO19,
    BOARD_IOID_DIO22,
    BOARD_IOID_DIO27,
    BOARD_IOID_DIO28,
    BOARD_IOID_DIO29,
    BOARD_IOID_DIO30,
    IOID_UNUSED
  };

  uint32_t *pin;

  for(pin = pins; *pin != IOID_UNUSED; pin++) {
    ti_lib_ioc_pin_type_gpio_input(*pin);
    ti_lib_ioc_io_port_pull_set(*pin, IOC_IOPULL_DOWN);
  }
}
/*---------------------------------------------------------------------------*/
static void
configure_used_pins(void){
	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_PIR_HI);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_PIR_HI, IOC_IOPULL_UP);
	
	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_PIR_LO);
	ti_lib_ioc_io_port_pull_set(BOARD_IOID_PIR_LO, IOC_IOPULL_DOWN);
	
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_BAT_ADC_ENABLE);
	ti_lib_gpio_clear_dio(BOARD_IOID_BAT_ADC_ENABLE);

	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_HDC1010_DRDY);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_HDC1010_DRDY, IOC_IOPULL_UP);

	ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_OPT3001_INT);
    ti_lib_ioc_io_port_pull_set(BOARD_IOID_OPT3001_INT, IOC_IOPULL_UP);
#if APROS_EXT_FLASH
#else
	ti_lib_ioc_pin_type_gpio_output(BOARD_IOID_FLASH_ENABLE);
	ti_lib_gpio_clear_dio(BOARD_IOID_FLASH_ENABLE);
#endif
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
  
  //board_i2c_wakeup();

  /* Make sure the external flash is in the lower power mode */
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

