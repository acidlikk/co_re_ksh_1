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
 * Header file for the apros PIR Motion Sensor - EKMC-1603113
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "ekmc-1603113-sensor.h"
#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "lpm.h"

#include "ti-lib.h"

#include <stdint.h>

/*---------------------------------------------------------------------------*/
#define PIR_EKMC_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP    | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_DISABLE | IOC_BOTH_EDGES    | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)

/*---------------------------------------------------------------------------*/
#define DEBOUNCE_DURATION (CLOCK_SECOND>>5)

struct pir_timer {
  struct timer debounce;
  clock_time_t start;
  clock_time_t duration;
};

static struct pir_timer ekmc_timer;
/*---------------------------------------------------------------------------*/
/**
 * \brief Handler for the pir sensor
 */
static void
pir_event_handler(uint8_t ioid)
{
  /*
  if(ioid == BOARD_IOID_PIR_HI) {
    if(ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI) == 0) {
      sensors_changed(&ekmc_1603113_sensor);
    }
  }
  */

  if(ioid == BOARD_IOID_PIR_HI) {
    if(!timer_expired(&ekmc_timer.debounce)) {
      return;
    }

    timer_set(&ekmc_timer.debounce, DEBOUNCE_DURATION);
	
    if(ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI) == 0) {
      ekmc_timer.start = clock_time();
      ekmc_timer.duration = 0;
    } else {
      ekmc_timer.duration = clock_time() - ekmc_timer.start;
      sensors_changed(&ekmc_1603113_sensor);
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the pir sensor.
 *
 * \param type This function does nothing unless type == SENSORS_ACTIVE
 * \param c 0: disable the button, non-zero: enable
 * \param ioid: One of BOARD_IOID_PIR_HI, BOARD_IOID_PIR_LO etc
 */
static void
config_pirs(int type, int c, uint32_t ioid)
{
  switch(type) {
  case SENSORS_HW_INIT:
    ti_lib_gpio_clear_event_dio(ioid);
    ti_lib_rom_ioc_pin_type_gpio_input(ioid);
    ti_lib_rom_ioc_port_configure_set(ioid, IOC_PORT_GPIO, PIR_EKMC_GPIO_CFG);
    gpio_interrupt_register_handler(ioid, pir_event_handler);
    break;
  case SENSORS_ACTIVE:
    if(c) {
      ti_lib_gpio_clear_event_dio(ioid);
      ti_lib_rom_ioc_pin_type_gpio_input(ioid);
      ti_lib_rom_ioc_port_configure_set(ioid, IOC_PORT_GPIO, PIR_EKMC_GPIO_CFG);
      ti_lib_rom_ioc_int_enable(ioid);
    } else {
      ti_lib_rom_ioc_int_disable(ioid);
    }
    break;
  default:
    break;
  }
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the pir hi.
 *
 * Parameters are passed onto config_buttons, which does the actual
 * configuration
 * Parameters are ignored. They have been included because the prototype is
 * dictated by the core sensor API. The return value is also required by
 * the API but otherwise ignored.
 *
 * \param type passed to config_pirs as-is
 * \param value passed to config_pirs as-is
 *
 * \return ignored
 */
static int
config(int type, int value)
{
  config_pirs(type, value, BOARD_IOID_PIR_HI);

  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Status function for the pir sensor.
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \param ioid BOARD_IOID_PIR_LO, BOARD_IOID_PIR_HI etc
 * \return 1 if the pir sensor's port interrupt is enabled (edge detect)
 *
 * This function will only be called by status_left, status_right and the
 * called will pass the correct key_io_id
 */
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    if(ti_lib_ioc_port_configure_get(BOARD_IOID_PIR_HI) & IOC_INT_ENABLE) {
      return 1;
    }
    break;
  default:
    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  if(type == PIR_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI) == 0 ?
            PIR_SENSOR_VALUE_OCCUPIED : PIR_SENSOR_VALUE_EMPTY;
  } else if(type == PIR_SENSOR_VALUE_DURATION) {
    return (int)ekmc_timer.duration;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(ekmc_1603113_sensor, PIR_SENSOR, value, config,
               status);
/*---------------------------------------------------------------------------*/
/** @} */

