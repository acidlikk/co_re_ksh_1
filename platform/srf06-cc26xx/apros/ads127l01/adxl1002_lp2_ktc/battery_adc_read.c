#include "contiki-conf.h"
#include "battery_adc_read.h"
#include "ti-lib.h"

uint32_t
read_bat_voltage(void)
{
  uint32_t bat_vol;
  // Enable AUX
  // Read
  ti_lib_aon_wuc_aux_wakeup_event(AONWUC_AUX_WAKEUP);
  while(!(ti_lib_aon_wuc_power_status_get() & AONWUC_AUX_POWER_ON))
  { }

  // Enable clocks
  ti_lib_aux_wuc_clock_enable(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK);
  while(ti_lib_aux_wuc_clock_status(AUX_WUC_ADI_CLOCK | AUX_WUC_ANAIF_CLOCK | AUX_WUC_SMPH_CLOCK) != AUX_WUC_CLOCK_READY)
  { }

  // Select input
  AUXADCSelectInput(ADC_COMPB_IN_AUXIO7); // AD Channel //DIO 23
  
  // Configure and enable  
  AUXADCEnableSync(AUXADC_REF_FIXED, AUXADC_SAMPLE_TIME_170_US, AUXADC_TRIGGER_MANUAL);
  //AUXADCEnableSync(AUXADC_REF_VDDS_REL, AUXADC_SAMPLE_TIME_85P3_US, AUXADC_TRIGGER_MANUAL);
    
  AUXADCGenManualTrigger();
  bat_vol = AUXADCReadFifo();
  
  // Disable ADC
  AUXADCDisable();
  return bat_vol;
}

