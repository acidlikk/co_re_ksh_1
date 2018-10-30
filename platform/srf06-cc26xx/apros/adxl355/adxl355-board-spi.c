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
 * \addtogroup sensortag-cc26xx-spi
 * @{
 *
 * \file
 * Board-specific SPI driver common to the Sensortag and LaunchPad
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "ti-lib.h"
#include "adxl355-board-spi.h"
#include "board.h"

#include <stdbool.h>
/*---------------------------------------------------------------------------*/
static bool
adxl355_accessible(void)
{
  /* First, check the PD */
  if(ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
     != PRCM_DOMAIN_POWER_ON) {
    return false;
  }

  /* Then check the 'run mode' clock gate */
  if(!(HWREG(PRCM_BASE + PRCM_O_SSICLKGR) & PRCM_SSICLKGR_CLK_EN_SSI1)) {
    return false;
  }

  return true;
}
/*---------------------------------------------------------------------------*/
bool
adxl355_board_spi_write(const uint8_t *buf, size_t len)
{
  if(adxl355_accessible() == false) {
  	//printf("fail\n");
    return false;
  }

  while(len > 0) {
    uint32_t ul;
	//printf("write %x\n", *buf);
    ti_lib_ssi_data_put(SSI1_BASE, *buf);
    ti_lib_rom_ssi_data_get(SSI1_BASE, &ul);
 
    len--;
    buf++;
  }
	//printf("%d\n", ti_lib_ssi_status(SSI1_BASE));
  return true;
}
/*---------------------------------------------------------------------------*/
bool
adxl355_board_spi_read(uint8_t *buf, size_t len)
{
  if(adxl355_accessible() == false) {
  	//printf("fail\n");
    return false;
  }

  while(len > 0) {
    uint32_t ul;

    if(!ti_lib_rom_ssi_data_put_non_blocking(SSI1_BASE, 0)) {
      return false;
    }
    ti_lib_rom_ssi_data_get(SSI1_BASE, &ul);
	//printf("read ul %x\n", ul);
    *buf = (uint8_t)ul;
    len--;
    buf++;
  }
  return true;
}
/*---------------------------------------------------------------------------*/
void
adxl355_board_spi_flush()
{
  if(adxl355_accessible() == false) {
    return;
  }

  uint32_t ul;
  while(ti_lib_rom_ssi_data_get_non_blocking(SSI1_BASE, &ul));
}
/*---------------------------------------------------------------------------*/
void
adxl355_board_spi_open(uint32_t bit_rate, uint32_t clk_pin)
{
  uint32_t buf;

  /* First, make sure the SERIAL PD is on */
  ti_lib_prcm_power_domain_on(PRCM_DOMAIN_SERIAL);
  while((ti_lib_prcm_power_domain_status(PRCM_DOMAIN_SERIAL)
         != PRCM_DOMAIN_POWER_ON));

  /* Enable clock in active mode */
  ti_lib_rom_prcm_peripheral_run_enable(PRCM_PERIPH_SSI1);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* SPI configuration */
  ti_lib_ssi_int_disable(SSI1_BASE, SSI_RXOR | SSI_RXFF | SSI_RXTO | SSI_TXFF);
  ti_lib_ssi_int_clear(SSI1_BASE, SSI_RXOR | SSI_RXTO);
  ti_lib_rom_ssi_config_set_exp_clk(SSI1_BASE, ti_lib_sys_ctrl_clock_get(),
                                    SSI_FRF_MOTO_MODE_0,
                                    SSI_MODE_MASTER, bit_rate, 8);
  ti_lib_rom_ioc_pin_type_ssi_master(SSI1_BASE, BOARD_IOID_ADXL355_SPI_MISO,
                                     BOARD_IOID_ADXL355_SPI_MOSI, IOID_UNUSED, clk_pin);
  ti_lib_ssi_enable(SSI1_BASE);

  /* Get rid of residual data from SSI port */
  while(ti_lib_ssi_data_get_non_blocking(SSI1_BASE, &buf));
}
/*---------------------------------------------------------------------------*/
void
adxl355_board_spi_close()
{
  /* Power down SSI1 */
  ti_lib_rom_prcm_peripheral_run_disable(PRCM_PERIPH_SSI1);
  ti_lib_prcm_load_set();
  while(!ti_lib_prcm_load_get());

  /* Restore pins to a low-consumption state */
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADXL355_SPI_MISO);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL355_SPI_MISO, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADXL355_SPI_MOSI);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL355_SPI_MOSI, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADXL355_SPI_SCK);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL355_SPI_SCK, IOC_IOPULL_DOWN);

  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_ADXL355_ADXL_CS);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_ADXL355_ADXL_CS, IOC_IOPULL_DOWN);
}
/*---------------------------------------------------------------------------*/
/** @} */

