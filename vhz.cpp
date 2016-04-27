#include <Arduino.h>
#include "vhz.h"
#include "config.h"
#include "pwm.h"
#include "sinetable.h"

unsigned int posAccum;
unsigned int posInc;
int rotorPosition;

void setupVHz()
{
  posInc = 0;
  posAccum = 0;
  rotorPosition = 0;
}

//this target RPM is in mechanical / motor RPM not electrical RPM
//Smoothly scales post and pre multipliers to keep within proper range. Output variable posInc
//is scaled up 65536 in the end.u
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
