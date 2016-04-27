#include <Arduino.h>
#include "pwm.h"
#include "sinetable.h"
#include "config.h"

unsigned int posAccum;
unsigned int posInc;
int rotorPosition;

void setup_pwm()
{
  posAccum = 0;
  rotorPosition = 0;
  posInc = 0;
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
  PWMC_ConfigureClocks((unsigned int)PWM_FREQ * MAX_PWM_DUTY * 2, 0, VARIANT_MCK );
  
  //find the number of ticks necessary to ensure that dead time is at least as long as requested.
  int deadTicks = ((VARIANT_MCK * 1000ul) / ((unsigned int)PWM_FREQ * MAX_PWM_DUTY) / (unsigned int)PWM_TARGET_DEADTIME) + 1;

  deadTicks = 40;

  //                   PWM, Chan, Clock, Left/Center, Polarity, CountEvent, DeadEnable, DeadHighInv, DeadLowInv
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM_INTERFACE, 0, 1050) ;  // period = 525 ticks (10Khz), every tick is 1/5.25 micro seconds
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above so min is 0 max is 525)
  PWMC_SetDeadTime(PWM_INTERFACE, 0, deadTicks, deadTicks) ; //set a bit of dead time around all transitions

  PWMC_ConfigureChannelExt (PWM, 1, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM, 1, 1050) ; 
  PWMC_SetDutyCycle (PWM, 1, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above
  PWMC_SetDeadTime(PWM, 1, deadTicks, deadTicks) ; //set some dead time

  PWMC_ConfigureChannelExt (PWM, 2, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_SetPeriod (PWM, 2, 1050) ;
  PWMC_SetDutyCycle (PWM, 2, 0); //set duty cycle of channel 0 to 0 (duty is out of the period above
  PWMC_SetDeadTime(PWM, 2, deadTicks, deadTicks) ; //set some dead time

  PWMC_ConfigureSyncChannel(PWM_INTERFACE, 7, 0, 0,0); //make channels 0, 1, 2 be synchronous
  PWMC_SetSyncChannelUpdatePeriod(PWM_INTERFACE, 1);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 0, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 1, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);
  PWMC_ConfigureChannelExt(PWM_INTERFACE, 2, PWM_CMR_CPRE_CLKA, PWM_CMR_CALG, 0, 0, PWM_CMR_DTE, 0, 0);  
  
  //Configure PWM hardware comparison unit to trigger while counting up and just slightly
  //less than the maximum value. This triggers the ADC and it starts to convert right about at the time
  //we actually reach 1050 which is the top value.
  int compMode = PWM_CMPM_CEN; //+ PWM_CMPM_CTR(0) + PWM_CMPM_CPR(0) + PWM_CMPM_CUPR(0);
  PWMC_ConfigureComparisonUnit(PWM_INTERFACE, 0, 1030, compMode);
  //Use comparison unit 0 for Event line 0 which the ADC hardware is looking for triggers on
  PWMC_ConfigureEventLineMode(PWM_INTERFACE, 0, PWM_ELMR_CSEL0);
  
  PWMC_EnableChannel (PWM_INTERFACE, 0) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 1) ;   // enable
  PWMC_EnableChannel (PWM_INTERFACE, 2) ;   // enable
}

void updatePWM(unsigned int a, unsigned int b, unsigned int c)
{
  
  if (a < PWM_BUFFER) a = 0;
  if (b < PWM_BUFFER) b = 0;
  if (c < PWM_BUFFER) c = 0;

  if (a > (MAX_PWM_DUTY - PWM_BUFFER)) a = MAX_PWM_DUTY;
  if (b > (MAX_PWM_DUTY - PWM_BUFFER)) b = MAX_PWM_DUTY;
  if (c > (MAX_PWM_DUTY - PWM_BUFFER)) c = MAX_PWM_DUTY;
    
  PWMC_SetDutyCycle (PWM_INTERFACE, 0, a);
  PWMC_SetDutyCycle (PWM_INTERFACE, 1, b);
  PWMC_SetDutyCycle (PWM_INTERFACE, 2, c);
  PWMC_SetSyncChannelUpdateUnlock(PWM_INTERFACE); //enable setting of all those duties all at once
}

//this target RPM is in mechanical / motor RPM not electrical RPM
//Smoothly scales post and pre multipliers to keep within proper range. Output variable posInc
//is scaled up 65536 in the end.
void setVHzSpeed(int targetRPM)
{ 
	int elecRPM = (targetRPM * settings.numPoles);
	int preMultiplier = 16;
	int postMultiplier = 0;
	int testVal = elecRPM;
	while (testVal > 1024)
	{
	      testVal = testVal >> 1;
	      preMultiplier--;
	      postMultiplier++;
	}
	
	posInc = ((((elecRPM * 1024) / 60) << preMultiplier) / 10000) << postMultiplier;  
	controllerStatus.rpm = targetRPM;
}

void updatePosVHz()
{
	int a, b, c;
		
	posAccum += posInc;
	
	rotorPosition += (posAccum >> 16);
	rotorPosition &= 0x3FF;
	
	posAccum &= 0xFFFF;

	a = sineTable[rotorPosition];
	b = sineTable[(rotorPosition + 341) & 0x3FF];
	c = sineTable[(rotorPosition + 682) & 0x3FF];
  
	updatePWM(a,b,c);
}