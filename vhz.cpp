#include <Arduino.h>
#include "vhz.h"
#include "config.h"
#include "pwm.h"
#include "sinetable.h"
#include "adc.h"
#include "encoder.h"
#include "dig_in.h"
#include <due_can.h>

int32_t posAccum;
int32_t posInc;
int rotorPosition;
uint32_t vhzCounter;
int a, b, c;
int currentSector, lastSector;
bool needSectorCorrection;

OFFSET_TEST offsetVhz;

void setupVHz()
{
  posInc = 0;
  posAccum = 0;
  rotorPosition = 0;
  vhzCounter = 0;
  needSectorCorrection = true;

  if (settings.hallAB != 255 && settings.hallBC != 255 & settings.hallCA != 255)
  {
	  int sector = getMotorSector();
	  if (sector == 0)
	  {
		  SerialUSB.println("Not using hall effect sensors for initial position");
		  return;
	  }
	  SerialUSB.print("Hall effect sensors indicate we're starting in sector ");
	  SerialUSB.println(sector);
	  rotorPosition = ((sector - 1) * 85) + 42; //start in the middle of whichever sector we're in as a guess
	  currentSector = lastSector = sector;
  }
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

	if (targetRPM == 0) 
	{
		  digitalWrite(42, LOW); //disable drive
		  return;
	}

	digitalWrite(42, HIGH); //enable drive

	while (testVal > 1024)
	{
	      testVal = testVal >> 1;
	      preMultiplier--;
	      postMultiplier++;
	}
	
	posInc = ((((elecRPM * 512) / 60) << preMultiplier) / 10000) << postMultiplier;

	controllerStatus.rpm = targetRPM;
}

void updatePosVHz()
{
  int localRotorPos;

  vhzCounter++;

  controllerStatus.phaseCurrentB = getCurrent2();
  controllerStatus.phaseCurrentC = getCurrent1();
  controllerStatus.phaseCurrentA = -controllerStatus.phaseCurrentB - controllerStatus.phaseCurrentC;

  if (needSectorCorrection)
  {
	  currentSector = getMotorSector();
	  if (currentSector != lastSector)
	  {
		  needSectorCorrection = false;
		  int offset = currentSector - lastSector;
		  if (offset == -1)
		  {
			  rotorPosition = ((currentSector) * 85) & 0x1FF;
		  }
		  else if (offset == 1)
		  {
			  rotorPosition = ((currentSector - 1) * 85);
		  }

		  lastSector = currentSector;
	  }
  }
		
	posAccum += posInc;

  if (controllerStatus.runningOffsetTest)
  {
    if (vhzCounter - offsetVhz.testStart > 1000) // 0.1 seconds
    {
      offsetVhz.testStart = vhzCounter;
      if (offsetVhz.posAccum > offsetVhz.bestAccum)
      {
        offsetVhz.bestAccum = offsetVhz.posAccum;
        offsetVhz.bestOffset = offsetVhz.currentOffset;
      }
      if (offsetVhz.currentOffset < 511)
      {
        offsetVhz.currentOffset++;
        settings.thetaOffset = offsetVhz.currentOffset;
        offsetVhz.posAccum = 0;
      }
      else
      {
        SerialUSB.println("Offset test is done.");
        SerialUSB.print("Best offset was: ");
        SerialUSB.println(offsetVhz.bestOffset);
        controllerStatus.runningOffsetTest = false;
        settings.thetaOffset = offsetVhz.bestOffset;       
        controllerStatus.IdRef = 0;
        controllerStatus.IqRef = 0;
      }
    }
  }
	
	rotorPosition += (posAccum >> 16);

	posAccum &= 0xFFFF;
  
	rotorPosition &= 0x1FF;

    localRotorPos = (rotorPosition + settings.thetaOffset) & 511;
		
	a = ( (_sin_times32768[localRotorPos] + 32768) * 200) / 65536;
	b = ( (_sin_times32768[(localRotorPos + 170) & 511]+32768) * 200) / 65536;
	c = ( (_sin_times32768[(localRotorPos + 341) & 511]+32768) * 200) / 65536;

  //SVM style PWM output
  if (a <= b)
  {
    if (a <= c) //A is smallest of all
    {
      updatePWM(0, b - a, c - a);
    }
    else //C is smallest then
    {
      updatePWM(a - c, b - c, 0);
    }
  }
  else
  {
    if (b <= c) //B is smallest
    {
      updatePWM(a - b, 0, c - b);
    }
    else //C is the smallest
    { 
      updatePWM(a - c, b - c, 0);
    }
  }

//updatePWM(a,b,c);
 
  if (vhzCounter & 8) sendVHzCANMsgs();
	
}

void startVHZOffsetTest()
{
  controllerStatus.runningOffsetTest = true;
  offsetVhz.bestOffset = 0;
  offsetVhz.currentOffset = 0;
  offsetVhz.posAccum = 0;
  offsetVhz.bestAccum = 0;
  offsetVhz.testStart = vhzCounter;
  settings.thetaOffset = 0;
}

void sendVHzCANMsgs()
{
  CAN_FRAME outFrame;
  int16_t temp;

  //debugging message. Sends rotor angle and phase currents
  outFrame.id = settings.canBaseTx;
  outFrame.length = 8;
  outFrame.extended = false;
  outFrame.data.byte[0] = highByte(rotorPosition);
  outFrame.data.byte[1] = lowByte(rotorPosition);
  temp = controllerStatus.phaseCurrentA >> 16;
  outFrame.data.byte[2] = highByte(temp);
  outFrame.data.byte[3] = lowByte(temp);
  temp = controllerStatus.phaseCurrentB >> 16;
  outFrame.data.byte[4] = highByte(temp);
  outFrame.data.byte[5] = lowByte(temp);
  temp = controllerStatus.phaseCurrentC >> 16;
  outFrame.data.byte[6] = highByte(temp);
  outFrame.data.byte[7] = lowByte(temp);
  Can0.sendFrame(outFrame);

  outFrame.id = settings.canBaseTx + 1;
  outFrame.data.byte[0] = 0;
  outFrame.data.byte[1] = 0;
  outFrame.data.byte[2] = highByte(a);
  outFrame.data.byte[3] = lowByte(a);  
  outFrame.data.byte[4] = highByte(b);
  outFrame.data.byte[5] = lowByte(b);
  outFrame.data.byte[6] = highByte(c);
  outFrame.data.byte[7] = lowByte(c);
  Can0.sendFrame(outFrame);

 Can0.sendFrame(outFrame); 
}

