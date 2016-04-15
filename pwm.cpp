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

  //assuming 10khz and 1050 that would be a clock of 10.5MHz
  PWMC_ConfigureClocks((unsigned int)PWM_FREQ * MAX_PWM_DUTY, 0, VARIANT_MCK );
  
  //find the number of ticks necessary to ensure that dead time is at least as long as requested.
  int deadTicks = ((VARIANT_MCK * 1000ul) / ((unsigned int)PWM_FREQ * MAX_PWM_DUTY) / (unsigned int)PWM_TARGET_DEADTIME) + 1;
  
  PWMC_ConfigureChannel(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_SetPeriod (PWM_INTERFACE, 0, 1050) ;  // period = 525 ticks (10Khz), every tick is 1/5.25 micro seconds
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above so min is 0 max is 525)
  PWMC_SetDeadTime(PWM_INTERFACE, 0, deadTicks, deadTicks) ; //set a bit of dead time around all transitions

  PWMC_ConfigureChannel (PWM, 1, PWM_CMR_CPRE_CLKA, 0, 0) ; // channel 1, 84Mhz / 16 (5.25Mhz), center aligned, start low
  PWMC_SetPeriod (PWM, 1, 1050) ;  // period = 525 ticks (10Khz), every tick is 1/5.25 micro seconds
  PWMC_SetDutyCycle (PWM, 1, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above so min is 0 max is 525)
  PWMC_SetDeadTime(PWM, 1, deadTicks, deadTicks) ; //set some dead time

  PWMC_ConfigureChannel (PWM, 2, PWM_CMR_CPRE_CLKA, 0, 0) ; // channel 2, 84Mhz / 16 (5.25Mhz), center aligned, start low
  PWMC_SetPeriod (PWM, 2, 1050) ;  // period = 525 ticks (10Khz), every tick is 1/5.25 micro seconds
  PWMC_SetDutyCycle (PWM, 2, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above so min is 0 max is 525)
  PWMC_SetDeadTime(PWM, 2, deadTicks, deadTicks) ; //set some dead time

  PWMC_ConfigureSyncChannel(PWM_INTERFACE, 7, 0, 0,0); //make channels 0, 1, 2 be synchronous
  PWMC_SetSyncChannelUpdatePeriod(PWM_INTERFACE, 1);
  PWMC_ConfigureChannel(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, 0, 0);
  PWMC_EnableChannel (PWM_INTERFACE, 0) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 1) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 2) ;   // enable
}

void updatePWM(unsigned int a, unsigned int b, unsigned int c)
{
  if (a < PWM_BUFFER) a = 0;
  if (b < PWM_BUFFER) b = 0;
  if (c < PWM_BUFFER) c = 0;

  if (a > MAX_PWM_DUTY- PWM_BUFFER) a = MAX_PWM_DUTY;
  if (b > MAX_PWM_DUTY- PWM_BUFFER) b = MAX_PWM_DUTY;
  if (c > MAX_PWM_DUTY- PWM_BUFFER) c = MAX_PWM_DUTY;
  
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, a);
  PWMC_SetDutyCycle (PWM_INTERFACE, 1, b);
  PWMC_SetDutyCycle (PWM_INTERFACE, 2, c);
  PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE); //enable setting of all those duties all at once
}