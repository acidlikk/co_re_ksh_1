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
 * Header file for the apros PIR Motion Sensor - IRA-S410ST01
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "lib/sensors.h"
#include "ira-s410st01-sensor.h"
#include "gpio-interrupt.h"
#include "sys/timer.h"
#include "lpm.h"

#include "ti-lib.h"

#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define PIR_HI_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP | IOC_SLEW_DISABLE  | \
                                 IOC_HYST_ENABLE   | IOC_RISING_EDGE   | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP| IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
#define PIR_LO_GPIO_CFG         (IOC_CURRENT_2MA  | IOC_STRENGTH_AUTO | \
                                 IOC_IOPULL_UP| IOC_SLEW_DISABLE  | \
                                 IOC_HYST_ENABLE | IOC_RISING_EDGE   | \
                                 IOC_INT_ENABLE   | IOC_IOMODE_NORMAL | \
                                 IOC_NO_WAKE_UP   | IOC_INPUT_ENABLE)
/*---------------------------------------------------------------------------*/
#define DEBOUNCE_DURATION (CLOCK_SECOND>>5)

struct pir_timer {
  struct timer debounce;
  clock_time_t start;
  clock_time_t duration;
};

static struct pir_timer hi_timer, lo_timer;
/*---------------------------------------------------------------------------*/
/**
 * \brief Handler for the pir sensor
 */
static void
pir_event_handler(uint8_t ioid)
{
  if(ioid == BOARD_IOID_PIR_HI) {
   /* if(!timer_expired(&hi_timer.debounce)) {
      return;
    }*/

    //timer_set(&hi_timer.debounce, DEBOUNCE_DURATION);

    if(ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI) == 0) {

    /*  hi_timer.start = clock_time();
      hi_timer.duration = 0;
    } else {
      hi_timer.duration = clock_time() - hi_timer.start;*/
      sensors_changed(&pir_hi_sensor);
    }
  }

  if(ioid == BOARD_IOID_PIR_LO) {
  	
	/*if(!timer_expired(&lo_timer.debounce)) {
	  return;
	}*/

	//timer_set(&lo_timer.debounce, DEBOUNCE_DURATION);

	/*
	* Start press duration counter on press (falling), notify on release
	* (rising)
	*/
	if(ti_lib_gpio_read_dio(BOARD_IOID_PIR_LO) == 0) {

	  /*lo_timer.start = clock_time();
	  lo_timer.duration = 0;
	} else {
	  lo_timer.duration = clock_time() - lo_timer.start;*/
	  sensors_changed(&pir_lo_sensor);
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
    ti_lib_rom_ioc_port_configure_set(ioid, IOC_PORT_GPIO, PIR_HI_GPIO_CFG);
    gpio_interrupt_register_handler(ioid, pir_event_handler);
    break;
  case SENSORS_ACTIVE:
    if(c) {
      ti_lib_gpio_clear_event_dio(ioid);
      ti_lib_rom_ioc_pin_type_gpio_input(ioid);
      ti_lib_rom_ioc_port_configure_set(ioid, IOC_PORT_GPIO, PIR_LO_GPIO_CFG);
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
config_hi(int type, int value)
{
  config_pirs(type, value, BOARD_IOID_PIR_HI);

  return 1;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Configuration function for the pir lo.
 *
 * Parameters are passed onto config_buttons, which does the actual
 * configuration
 * Parameters are ignored. They have been included because the prototype is
 * dictated by the core sensor api. The return value is also required by
 * the API but otherwise ignored.
 *
 * \param type passed to config_pirs as-is
 * \param value passed to config_pirs as-is
 *
 * \return ignored
 */
static int
config_lo(int type, int value)
{
  config_pirs(type, value, BOARD_IOID_PIR_LO);

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
status(int type, uint32_t ioid)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    if(ti_lib_ioc_port_configure_get(ioid) & IOC_INT_ENABLE) {
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
value_hi(int type)
{
  if(type == PIR_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_PIR_HI) == 0 ?
            PIR_SENSOR_VALUE_OCCUPIED : PIR_SENSOR_VALUE_EMPTY;
  } else if(type == PIR_SENSOR_VALUE_DURATION) {
    return (int)hi_timer.duration;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
value_lo(int type)
{
  if(type == PIR_SENSOR_VALUE_STATE) {
    return ti_lib_gpio_read_dio(BOARD_IOID_PIR_LO) == 0 ?
            PIR_SENSOR_VALUE_OCCUPIED : PIR_SENSOR_VALUE_EMPTY;
  } else if(type == PIR_SENSOR_VALUE_DURATION) {
    return (int)lo_timer.duration;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Status function for the hi pir.
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the pir sensor's port interrupt is enabled (edge detect)
 *
 * This function will call status. It will pass type verbatim and it will also
 * pass the correct ioid
 */
static int
status_hi(int type)
{
  return status(type, BOARD_IOID_PIR_HI);
}
/*---------------------------------------------------------------------------*/
/**
 * \brief Status function for the lo pir.
 * \param type SENSORS_ACTIVE or SENSORS_READY
 * \return 1 if the pir sensor's port interrupt is enabled (edge detect)
 *
 * This function will call status. It will pass type verbatim and it will also
 * pass the correct ioid
 */
static int
status_lo(int type)
{
  return status(type, BOARD_IOID_PIR_LO);
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(pir_hi_sensor, PIR_SENSOR, value_hi, config_hi,
               status_hi);
SENSORS_SENSOR(pir_lo_sensor, PIR_SENSOR, value_lo, config_lo,
               status_lo);
/*---------------------------------------------------------------------------*/
/** @} */

