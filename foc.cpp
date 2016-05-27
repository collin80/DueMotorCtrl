#include <due_can.h>
#include "config.h"
#include "pid.h"
#include "sinetable.h"
#include "pwm.h"
#include "adc.h"
#include "encoder.h"
#include "foc.h"

int32_t mechVelo, elecVelo; //velocity both mechanical and electrical
uint32_t focCounter; //how many times the FOC loop has run

PID qPID(0, 0, 0);
PID dPID(0, 0, 0);

OFFSET_TEST offsetTest;

void setupFOC()
{
	qPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
	qPID.setMinValue(0);
	qPID.setMaxValue(210); //max PWM duty cycle

	dPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
	dPID.setMinValue(0);
	dPID.setMaxValue(10); //max PWM duty cycle
	
	focCounter = 0;

	controllerStatus.IdRef = 0;
	controllerStatus.IqRef = 0;
}

//called every ADC interrupt to update settings for FOC (10,000 times per second)
void updateFOC()
{
	int32_t temp;
	int32_t Ia, Ib;
	int32_t Va, Vb, VaScaled;
	int32_t Vd, Vq;
	int32_t PWMA, PWMB, PWMC;
	int32_t vbSqrt;
	int localTheta;
	int anglePlus90;
	int sineVal, cosineVal;
	
	focCounter++;
	
	localTheta = (controllerStatus.theta + settings.thetaOffset) & 511;
	
	/*
	basic steps:
	1. Get current (already calculated by ADC interrupt) and rotor velocity (from encoder)
	2. Determine third current from first two
	3. Use clarke transform to go from 3-phase to Ia, Ib
	4. Use park transform to go from Ia, Ib to Id, Iq (using theta from before)
	5. Calculate reference values for Id, Iq for where we'd like them to be (IdRef is always zero
	   for PMAC motors as they don't need you to provide magnetizing current (in fact that tends to be
	   bad. You could go negative I suppose to do field weakening though. IqRef is the amount of torque
	   we want.)
	6. Create error signal of difference between Id, Iq and IdRef, IqRef
	7. Run PI loop to get Vd, Vq
	8. Calculate new rotor angle - Theta (and store it)
	9. Rotate Vd, Vq with inverse Park to get Va, Vb
	10. Use inverse clarke transform to go from Va, Vb to the three phase PWM values
	11. Update PWM values in hardware

	Also, much of this was written while looking at the AC controller code written by
	Paul Holmes. Give that man a beer for me. You might note that a lot is different but it
	was a good reference.
	*/
	
	//step 1 - Get current and rotor velocity
	controllerStatus.phaseCurrentB = getCurrent2();
	controllerStatus.phaseCurrentC = getCurrent1();
	//step 2 - Calculate the missing phase current
	controllerStatus.phaseCurrentA = -controllerStatus.phaseCurrentB - controllerStatus.phaseCurrentC;

	//continuation of step 1
	temp = getEncoderCount();
	mechVelo = temp - controllerStatus.lastEncoderPos;
	if (settings.encoderDirection != 0) mechVelo *= -1;
	if (controllerStatus.runningOffsetTest) offsetTest.posAccum += mechVelo;
	//division is reasonably cheap on a SAM3X (12 clocks) so no worries
	//divide by encoderCount * 4 because we have quadrature input so it is 4x encoder resolution
	//one full rotation is 65536 so everything here should be a small fraction of that.
	//Do electrical velocity first as it tends to be higher precision than mechanical velocity
	elecVelo = (mechVelo * 65536 * settings.numPoles) / (settings.encoderCount * 4);
	mechVelo = elecVelo / settings.numPoles;
	controllerStatus.lastEncoderPos = temp;

	//step 3 - Clarke transform
	Ia = controllerStatus.phaseCurrentA;
	Ib = ((controllerStatus.phaseCurrentB * 2 + controllerStatus.phaseCurrentA) * 37837) / 65536;

	//Step 4 - Park transform
	anglePlus90 = (localTheta  + 128) & 511;
	sineVal = _sin_times32768[localTheta];
	cosineVal = _sin_times32768[anglePlus90];

	controllerStatus.Id = ((Ia * cosineVal) + (Ib * sineVal)) / 32768;
	controllerStatus.Iq = ((-Ia * sineVal) + (Ib * cosineVal)) / 32768;

	//Step 5 - Refine reference values
	if (controllerStatus.runningOffsetTest)
	{
		Vd = 0;
		Vq = 200;
		if (focCounter - offsetTest.testStart > 1000) // 0.1 seconds
		{
			offsetTest.testStart = focCounter;
			if (offsetTest.posAccum > offsetTest.bestAccum)
			{
				offsetTest.bestAccum = offsetTest.posAccum;
				offsetTest.bestOffset = offsetTest.currentOffset;
			}
			if (offsetTest.currentOffset < 511)
			{
				offsetTest.currentOffset++;
				settings.thetaOffset = offsetTest.currentOffset;
				offsetTest.posAccum = 0;
			}
			else
			{
				SerialUSB.println("Offset test is done.");
				SerialUSB.print("Best offset was: ");
				SerialUSB.println(offsetTest.bestOffset);
				controllerStatus.runningOffsetTest = false;
				settings.thetaOffset = offsetTest.bestOffset;				
				controllerStatus.IdRef = 0;
				controllerStatus.IqRef = 0;
			}
		}
	}
	else
	{
		//controllerStatus.IdRef = 0; //for PMAC reference is zero here
		//controllerStatus.IqRef = 150; //hard coded small amount for testing

		//Step 6&7 - Use PI loops to get Vd, Vq values
		qPID.setRef(controllerStatus.IqRef);
		dPID.setRef(controllerStatus.IdRef);
		Vd = dPID.calculatePID(controllerStatus.Id);
		Vq = qPID.calculatePID(controllerStatus.Iq);
		
		Vd = 0; //for a PMAC motor we do not want any direct voltage
		//Vq = 0;
	}

	//Step 8 - calculate new rotor angle. Easier for PMAC as it is synchronous
	//for now just take old Theta and add electrical velocity. The electrical velocity
	//will naturally increase if we're causing more torque than it takes to be steady state
	//and naturally decrease if we ask for less torque than it takes to maintain speed.
	//theta is 0 to 511 for a full rotation but elecVelo is 0 to 65535 for full rotation
	//this is a ratio of 128 so shift down 7 to get into same ballpark.
	controllerStatus.theta = (controllerStatus.theta + (elecVelo >> 7)) & 511;

	//Step 9 - Inverse park transform from Vd, Vq to Va, Vb - Uses new rotation angle
	localTheta = (controllerStatus.theta + settings.thetaOffset) & 511;
	anglePlus90 = (localTheta  + 128) & 511;
	sineVal = _sin_times32768[localTheta];
	cosineVal = _sin_times32768[anglePlus90];
	VaScaled = Vd * cosineVal - Vq * sineVal;
	Va = VaScaled / 32768;
	Vb = (Vq * sineVal + Vq * cosineVal) / 32768;

	//Step 10 - Inverse Clarke to go from Va, Vb to phase PWMs
	PWMA = Va;
	vbSqrt = (Vb * (int)56756);
	PWMB = (-VaScaled + vbSqrt) / 65536;
	PWMC = (-VaScaled - vbSqrt) / 65536;

  if (!controllerStatus.runningOffsetTest)
  {
    if (focCounter & 8) sendCANMsgs(); //every 8 times through this routine send debugging msg on CAN7
    return;
  }
  
	//updatePWM(PWMA, PWMB, PWMC);

	//SVM style PWM output
	if (PWMA <= PWMB)
	{
		if (PWMA <= PWMC) //A is smallest of all
		{
			updatePWM(0, PWMB - PWMA, PWMC - PWMA);
		}
		else //C is smallest then
		{
			updatePWM(PWMA - PWMC, PWMB - PWMC, 0);
		}
	}
	else
	{
		if (PWMB <= PWMC) //B is smallest
		{
			updatePWM(PWMA - PWMB, 0, PWMC - PWMB);
		}
		else //C is the smallest
		{ 
			updatePWM(PWMA - PWMC, PWMB - PWMC, 0);
		}
	}
	
	if (focCounter & 8) sendCANMsgs(); //every 8 times through this routine send debugging msg on CAN

}

void startFOCOffsetTest()
{
	controllerStatus.runningOffsetTest = true;
	offsetTest.bestOffset = 0;
	offsetTest.currentOffset = 0;
	offsetTest.posAccum = 0;
	offsetTest.bestAccum = 0;
	offsetTest.testStart = focCounter;
	settings.thetaOffset = 0;
}

void sendCANMsgs()
{
	CAN_FRAME outFrame;
	int16_t temp;

	//debugging message. Sends rotor angle and phase currents
	outFrame.id = settings.canBaseTx;
	outFrame.length = 8;
	outFrame.extended = false;
	outFrame.data.byte[0] = highByte(controllerStatus.theta);
	outFrame.data.byte[1] = lowByte(controllerStatus.theta);
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
 outFrame.data.byte[0] = (elecVelo >> 24);
 outFrame.data.byte[1] = (elecVelo >> 16) & 0xFF;
 outFrame.data.byte[2] = (elecVelo >> 8) & 0xFF;
 outFrame.data.byte[3] = (elecVelo) & 0xFF;
 outFrame.data.byte[4] = (controllerStatus.lastEncoderPos >> 24);
 outFrame.data.byte[5] = (controllerStatus.lastEncoderPos >> 16) & 0xFF;
 outFrame.data.byte[6] = (controllerStatus.lastEncoderPos >> 8) & 0xFF;
 outFrame.data.byte[7] = (controllerStatus.lastEncoderPos) & 0xFF;
Can0.sendFrame(outFrame);
}
