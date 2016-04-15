#include <Arduino.h>
#include "adc.h"
#include "config.h"

void setup_adc()
{
  //nothing at the moment.
  //but, should switch to either DMA ADC or
  //ADC readings triggered by comparison on PWM
}

float getBusVoltage()
{
	float valu = analogRead(3) * settings.busVoltageScale;
	return valu;
}

float getCurrent1()
{
	float valu = analogRead(1) * settings.current1Scale;
	return valu;  
}

float getCurrent2()
{
  	float valu = analogRead(2) * settings.current2Scale;
	return valu;
}

float getInvTemp1()
{
  	float valu = analogRead(0) * settings.inverterTemp1Scale;
	return valu;
}

float getInvTemp2()
{
  	float valu = analogRead(4) * settings.inverterTemp2Scale;
	return valu;
}
