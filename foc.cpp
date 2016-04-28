#include "config.h"
#include "pid.h"
#include "sinetable.h"
#include "pwm.h"
#include "adc.h"
#include "encoder.h"

int32_t mechVelo, elecVelo; //velocity both mechanical and electrical

PID qPID(0, 0, 0);
PID dPID(0, 0, 0);

void setupFOC()
{
	qPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
	qPID.setMinValue(0);
	qPID.setMaxValue(1050); //max PWM duty cycle

	dPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
	dPID.setMinValue(0);
	dPID.setMaxValue(1050); //max PWM duty cycle
}

//called every ADC interrupt to update settings for FOC (10,000 times per second)
void updateFOC()
{
	int32_t temp;
	int32_t Ia, Ib;
	int32_t Va, Vb;
	int32_t Vd, Vq;
	int32_t PWMA, PWMB, PWMC;
	int32_t vbSqrt;
	int64_t veloTemp;
	int anglePlus90;
	int sineVal, cosineVal;
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
	Paul Holmes. Give that man a beer for me.
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
	//division is reasonably cheap on a SAM3X but this is 64 bit division which might suck
	veloTemp = (mechVelo * (0x100000000ul) / settings.encoderCount) >> 16; //output is full rotation being 65536
	mechVelo = (int32_t) veloTemp;
	elecVelo = mechVelo * settings.numPoles; //always equal to or faster than mechanical speed.
	controllerStatus.lastEncoderPos = temp;

	//step 3 - Clarke transform
	Ia = controllerStatus.phaseCurrentA;
	Ib = ((controllerStatus.phaseCurrentB * 2 + controllerStatus.phaseCurrentA) * 37837u) >> 16;

	//Step 4 - Park transform
	anglePlus90 = (controllerStatus.theta  + 128) & 511;
	sineVal = _sin_times32768[controllerStatus.theta];
	cosineVal = _sin_times32768[anglePlus90];

	controllerStatus.Id = ((Ia * cosineVal) + (Ib * sineVal)) >> 15;
	controllerStatus.Iq = ((-Ia * sineVal) + (Ib * cosineVal)) >> 15;

	//Step 5 - Refine reference values
	controllerStatus.IdRef = 0; //for PMAC reference is zero here
	controllerStatus.IqRef = 200; //hard coded small amount for testing

	//Step 6&7 - Use PI loops to get Vd, Vq values
	qPID.setRef(controllerStatus.IqRef);
	dPID.setRef(controllerStatus.IdRef);
	Vd = dPID.calculatePID(controllerStatus.Id);
	Vq = qPID.calculatePID(controllerStatus.Iq);

	//Step 8 - calculate new rotor angle. Easier for PMAC as it is synchronous
	//for now just take old Theta and add electrical velocity. The electrical velocity
	//will naturally increase if we're causing more torque than it takes to be steady state
	//and naturally decrease if we ask for less torque than it takes to maintain speed.
	//theta is 0 to 511 for a full rotation but elecVelo is 0 to 65535 for full rotation
	//this is a ratio of 128 so shift down 7 to get into same ballpark.
	controllerStatus.theta += (elecVelo >> 7);

	//Step 9 - Inverse park transform from Vd, Vq to Va, Vb - Uses new rotation angle
	anglePlus90 = (controllerStatus.theta  + 128) & 511;
	sineVal = _sin_times32768[controllerStatus.theta];
	cosineVal = _sin_times32768[anglePlus90];
	Va = Vd * cosineVal - Vq * sineVal;
	Vb = Vq * sineVal + Vq * cosineVal;

	//Step 10 - Inverse Clarke to go from Va, Vb to phase PWMs
	PWMA = Va;
	vbSqrt = (Vb * 56756u) >> 16;
	PWMB = (-Va + vbSqrt) >> 1;
	PWMC = (-Va - vbSqrt) >> 1;

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
}