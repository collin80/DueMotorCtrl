#include <Arduino.h>
#include "pwm.h"
#include "config.h"

void setup_pwm()
{
  pinMode(42, OUTPUT);  

  pmc_enable_periph_clk (PWM_INTERFACE_ID) ;  // turn on clocking to PWM unit

  PWMC_DisableChannel(PWM_INTERFACE, 0);
  PWMC_DisableChannel(PWM_INTERFACE, 1);
  PWMC_DisableChannel(PWM_INTERFACE, 2);  
  
    // Configure (PC2) through (PC7) to be PWM now. This causes all of them to be PWM channels 0, 1, 2
  PIOC->PIO_PDR = 0xFC;  // disable PIO control for all of them
  PIOC->PIO_IDR = 0xFC;   // disable PIO interrupts for all of them
  PIOC->PIO_ABSR |= 0xFC;  // switch to B peripheral

  //assuming 10khz and 1050 that would be a clock of 21MHz (Center aligned is twice speed)
  //Center aligned mode counts from 0 up to max then back down to 0 every cycle
  //so you need twice the frequency to get the same result as left aligned mode.
  PWMC_ConfigureClocks((unsigned int)PWM_FREQ * MAX_PWM_DUTY * 2, 0, VARIANT_MCK );
  
  //                   PWM, Chan, Clock, Left/Center, Polarity, CountEvent, DeadEnable, DeadHighInv, DeadLowInv
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM_INTERFACE, 0, MAX_PWM_DUTY) ;  // period = 525 ticks (10Khz), every tick is 1/5.25 micro seconds
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above so min is 0 max is 1050)
  PWMC_SetDeadTime(PWM_INTERFACE, 0, PWM_DEADTIME, PWM_DEADTIME) ; //set a bit of dead time around all transitions

  PWMC_ConfigureChannelExt (PWM, 1, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM, 1, MAX_PWM_DUTY) ; 
  PWMC_SetDutyCycle (PWM, 1, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above
  PWMC_SetDeadTime(PWM, 1, PWM_DEADTIME, PWM_DEADTIME) ; //set some dead time

  PWMC_ConfigureChannelExt (PWM, 2, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM, 2, MAX_PWM_DUTY) ;
  PWMC_SetDutyCycle (PWM, 2, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above
  PWMC_SetDeadTime(PWM, 2, PWM_DEADTIME, PWM_DEADTIME) ; //set some dead time

  PWMC_ConfigureSyncChannel(PWM_INTERFACE, 7, 0, 0,0); //make channels 0, 1, 2 be synchronous
  PWMC_SetSyncChannelUpdatePeriod(PWM_INTERFACE, 1);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 1, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 2, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);  
  
  //Configure PWM hardware comparison unit to trigger while counting up and at the max duty value
  int compMode = PWM_CMPM_CEN; //+ PWM_CMPM_CTR(0) + PWM_CMPM_CPR(0) + PWM_CMPM_CUPR(0);
  PWMC_ConfigureComparisonUnit(PWM_INTERFACE, 0, MAX_PWM_DUTY, compMode);
  //Use comparison unit 0 for Event line 0 which the ADC hardware is looking for triggers on
  PWMC_ConfigureEventLineMode(PWM_INTERFACE, 0, PWM_ELMR_CSEL0);
  
  PWMC_EnableChannel (PWM_INTERFACE, 0) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 1) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 2) ;   // enable
}

void updatePWM(unsigned int a, unsigned int b, unsigned int c)
{
  static int ledCounter = 0;
  int pwmBuff = PWM_BUFFER;
  uint32_t upperDuty = MAX_PWM_DUTY - pwmBuff;
  uint32_t dutyA = a, dutyB = b, dutyC = c;
  
  //For debugging purposes the LED flashes in time to the PWM frequency. If the LED is flashing you're still up and running.
  ledCounter++;
  if (ledCounter == 1000) PIO_SetOutput(PIOC, PIO_PC20, HIGH, 0, PIO_PULLUP);
  if (ledCounter == 2000)
  {
      PIO_SetOutput(PIOC, PIO_PC20, LOW, 0, PIO_PULLUP);
      ledCounter = 0;
  }
  
  //Prevent both types of short pulses.
  if (dutyA < PWM_BUFFER) dutyA = 0;
  if (dutyB < PWM_BUFFER) dutyB = 0;
  if (dutyC < PWM_BUFFER) dutyC = 0;

  if (dutyA > 955) dutyA = MAX_PWM_DUTY;
  if (dutyB > upperDuty) dutyB = MAX_PWM_DUTY;
  if (dutyC > upperDuty) dutyC = MAX_PWM_DUTY;
    
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, dutyA);
  PWMC_SetDutyCycle (PWM_INTERFACE, 1, dutyB);
  PWMC_SetDutyCycle (PWM_INTERFACE, 2, dutyC);
  PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE); //enable setting of all those duties all at once
}

