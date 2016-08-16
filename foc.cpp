#include <due_can.h>
#include "config.h"
#include "pid.h"
#include "sinetable.h"
#include "pwm.h"
#include "adc.h"
#include "encoder.h"
#include "foc.h"

//If this is defined then a whole crapload of very intimate details about
//FOC will be sent over CAN very rapidly. You don't really want this unless
//you're debugging the FOC code so don't define it otherwise.
#define DEBUG_FOC

int32_t mechVelo, elecVelo; //velocity both mechanical and electrical
int32_t PWMA, PWMB, PWMC;
uint32_t focCounter; //how many times the FOC loop has run
int32_t Vd, Vq;
int32_t Iq, Id;
int32_t Ia, Ib;
int32_t Va, Vb, VaScaled;
int32_t vbSqrt;
int16_t refIQ;

PID qPID(0, 0, 0);
PID dPID(0, 0, 0);

OFFSET_TEST offsetTest;

void setupFOC()
{
    qPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
    qPID.setMinValue(-300);
    qPID.setMaxValue(300); //max PWM duty cycle

    dPID.setCoeffs(settings.pid_KP, settings.pid_KI, settings.pid_KD);
    dPID.setMinValue(0);
    dPID.setMaxValue(0); //max PWM duty cycle
    
    refIQ = 0;

    focCounter = 0;

    controllerStatus.IdRef = 0;
    controllerStatus.IqRef = 0;
}

void setDesiredTorque(int16_t torq)
{
    //If desired torque is extremely low then disable the drive circuits
    if (torq < 5 && torq > -5)
    {
        digitalWrite(42, LOW); //disable drive
        updatePWM(0,0,0);
        refIQ = 0;
        return;
    }

    //otherwise be sure to enable drive circuits so we can provide or get power from motor
    digitalWrite(42, HIGH); //enable drive
    
    //Then set our target torque and let the FOC algorithm figure out how to make that happen
    refIQ = torq;
}

//called every ADC interrupt to update settings for FOC (10,000 times per second)
void updateFOC()
{
    int localTheta;
    int anglePlus90;
    int32_t temp;
    int32_t sineVal, cosineVal;
    int32_t currA, currB;

    focCounter++;

    //localTheta = (controllerStatus.theta + settings.thetaOffset) & 511;
    
    //instead of above line get absolute theta position from encoder count. Encoder
    //count resets at every rotation on index pulse so we can use it to get absolute position
    localTheta = getEncoderCount(); //encoder is backward on test device
    if (settings.encoderDirection != 0) localTheta *= -1;
    localTheta = (((localTheta * 512l * (int32_t)settings.numPoles) / ((int)settings.encoderCount * 4l)));
    localTheta &= 511; //must do this because otherwise the numPoles multiplication above will cause the range to be too large

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
    controllerStatus.phaseCurrentRawB = getCurrent1();
    controllerStatus.phaseCurrentRawA = getCurrent2();
    //step 2 - Calculate the missing phase current
    controllerStatus.phaseCurrentRawC = -controllerStatus.phaseCurrentRawA - controllerStatus.phaseCurrentRawB;
    
    currA = controllerStatus.phaseCurrentRawA / 6554; //turn super large number into tenths of an amp instead
    currB = controllerStatus.phaseCurrentRawB / 6554;
    
    controllerStatus.phaseCurrentFilteredA = ((controllerStatus.phaseCurrentFilteredA * 3) + controllerStatus.phaseCurrentRawA) / 4;
    controllerStatus.phaseCurrentFilteredB = ((controllerStatus.phaseCurrentFilteredB * 3) + controllerStatus.phaseCurrentRawB) / 4;
    controllerStatus.phaseCurrentFilteredC = ((controllerStatus.phaseCurrentFilteredC * 3) + controllerStatus.phaseCurrentRawC) / 4;

    //continuation of step 1
    temp = getEncoderCount();
    if (settings.encoderDirection != 0) temp *= -1;
    mechVelo = temp - controllerStatus.lastEncoderPos;    
    if (controllerStatus.runningOffsetTest) offsetTest.posAccum += mechVelo;
    //division is reasonably cheap on a SAM3X (12 clocks) so no worries
    //divide by encoderCount * 4 because we have quadrature input so it is 4x encoder resolution
    //one full rotation is 16384 so everything here should be a small fraction of that.
    //Do electrical velocity first as it tends to be higher precision than mechanical velocity
    elecVelo = (mechVelo * 16384l * settings.numPoles) / (int32_t)((int32_t)settings.encoderCount * 4l);
    mechVelo = elecVelo / settings.numPoles;
    
    //Finite filter by biasing toward current value three times compared to new value. Smooths the changes.
    controllerStatus.rpm = ((controllerStatus.rpm * 3) + ((60 * PWM_FREQ * elecVelo) / (16384 * settings.numPoles))) / 4;

    controllerStatus.lastEncoderPos = temp;

    //step 3 - Clarke transform - Since currA and currB are in tenths of an amp so are Ia and Ib
    Ia = currA;
    Ib = (((currB * 2) + currA) * 37837l) / 65536l; //the mult and div is the same as 1/sqrt(3)

    //Step 4 - Park transform
    anglePlus90 = (localTheta  + 128) & 511;
    sineVal = _sin_times32768[localTheta];
    cosineVal = _sin_times32768[anglePlus90];

    //we're still in tenths of an amp after these lines but now rotated to stationary frame:
    controllerStatus.Id = ((Ia * cosineVal) + (Ib * sineVal)) / 32768;
    controllerStatus.Iq = ((Ib * cosineVal) - (Ia * sineVal)) / 32768;
    
    //Step 5 - Refine reference values
    
    /*
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
    { */
        controllerStatus.IdRef = 0; //for PMAC reference is zero here
        if (controllerStatus.inReverse) controllerStatus.IqRef = -refIQ;
        else controllerStatus.IqRef = refIQ;

        //Step 6&7 - Use PI loops to get Vd, Vq values
        qPID.setRef(controllerStatus.IqRef);
        Vq = qPID.calculatePID(controllerStatus.Iq);
        dPID.setRef(controllerStatus.IdRef);
        Vd = dPID.calculatePID(controllerStatus.Id);
        if (settings.motorType > 0) Vd = 0; //for a PMAC/BLDC motor we do not want any direct voltage
    //}
    

    //Vd and Vq control PWM intensity but they could be negative now or after inverse park.
    //This is OK though. It works out.
    controllerStatus.Vd = Vd;
    controllerStatus.Vq = Vq;

    //Step 8 - calculate new rotor angle. Easier for PMAC as it is synchronous
    //for now just take old Theta and add electrical velocity. The electrical velocity
    //will naturally increase if we're causing more torque than it takes to be steady state
    //and naturally decrease if we ask for less torque than it takes to maintain speed.
    //theta is 0 to 511 for a full rotation but elecVelo is 0 to 16384 for full rotation
    //this is a ratio of 32 so shift down 5 to get into same ballpark.
    //Keep in mind that things are reversed if we're in reverse so subtract instead.
    if (settings.inReverse) controllerStatus.theta = (localTheta - (elecVelo >> 5)) & 511;
    else controllerStatus.theta = (localTheta + (elecVelo >> 5)) & 511;
    
    //Step 9 - Inverse park transform from Vd, Vq to Va, Vb - Uses new rotation angle
    localTheta = (controllerStatus.theta);// + settings.thetaOffset) & 511;
    anglePlus90 = (localTheta  + 128) & 511;
    sineVal = _sin_times32768[localTheta];
    cosineVal = _sin_times32768[anglePlus90];

    //Vq and Vd above were scaled as raw PWM values [0, 1000]
    VaScaled = Vd * cosineVal - Vq * sineVal;
    Va = VaScaled / 32768;
    Vb = (Vd * sineVal + Vq * cosineVal) / 32768;
    
    //Now, we've got Va and Vb which are duty cycles 0 to 1000 but rotated. Now they're still going to be within a circle
    //with radius 1000 but they could be anywhere in that circle including negative. The problem here is that this circle is
    //diameter * 2 the max PWM. But, due to the way the input was the radius of PWM_MAX should be followed. Even though
    //numbers could be negative it doesn't matter as SVM takes care of that and everything ends up positive in the end.
    
    //Step 10 - Inverse Clarke to go from Va, Vb to phase PWMs
    PWMA = Va;    
    vbSqrt = (Vb * 56757L); //multiply by sqrt(3) * 32768
    
    //vbSqrt and VaScaled are both scaled up 32768 times at this point
    
    PWMB = (-VaScaled + vbSqrt) / 65536; //now they should be /32768 to get back but there is 
    PWMC = (-VaScaled - vbSqrt) / 65536; //also a /2 in the equation so combined thats /65536
    
    //At this point PWMA - PWMC could very well still have negatives. But, the joy of the SVM below
    //is that those negatives will go away by biasing everything away. For instance,
    //if the three values are -400, -200, 500 then they'll end up
    //0, 200, 900
    
    //SVM style PWM output - This is optional. Can be commented out for traditional PWM
    if (PWMA <= PWMB)
    {
        if (PWMA <= PWMC) //A is smallest of all
        {
            PWMB -= PWMA;
            PWMC -= PWMA;
            PWMA = 0;
        }
        else //C is smallest then
        {
            PWMA -= PWMC;
            PWMB -= PWMC;
            PWMC = 0;
        }
    }
    else
    {
        if (PWMB <= PWMC) //B is smallest
        {
            PWMA -= PWMB;
            PWMC -= PWMB;
            PWMB = 0;
        }
        else //C is the smallest
        {
            PWMA -= PWMC;
            PWMB -= PWMC;
            PWMC = 0;
        }
    }
    
    updatePWM(PWMA,PWMB,PWMC);

#ifdef DEBUG_FOC
    if (focCounter > 18) //like 2800 frames per second
#else
    if (focCounter > 300) //MUCH slower - 35 updates per second at 10.5khz switching
#endif
    {
        focCounter = 0;
        sendCANMsgs();
    }

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
    int32_t temp2;
    
#ifdef DEBUG_FOC
    
    //debugging message. Sends rotor angle and phase currents
    outFrame.id = settings.canBaseTx;
    outFrame.length = 8;
    outFrame.extended = false;
    outFrame.data.byte[0] = highByte(controllerStatus.theta);
    outFrame.data.byte[1] = lowByte(controllerStatus.theta);
    temp = controllerStatus.phaseCurrentRawA >> 16;
    outFrame.data.byte[2] = highByte(temp);
    outFrame.data.byte[3] = lowByte(temp);
    temp = controllerStatus.phaseCurrentRawB >> 16;
    outFrame.data.byte[4] = highByte(temp);
    outFrame.data.byte[5] = lowByte(temp);
    temp = controllerStatus.phaseCurrentRawC >> 16;
    outFrame.data.byte[6] = highByte(temp);
    outFrame.data.byte[7] = lowByte(temp);
    Can0.sendFrame(outFrame);
    
    //Bus voltage and PWM duty cycles
    outFrame.id = settings.canBaseTx + 1;
    temp = (getBusVoltage() >> 17);
    outFrame.data.byte[0] = lowByte(temp);
    //outFrame.data.byte[1] = lowByte(getMotorSector());
    outFrame.data.byte[2] = highByte(PWMA);
    outFrame.data.byte[3] = lowByte(PWMA);
    outFrame.data.byte[4] = highByte(PWMB);
    outFrame.data.byte[5] = lowByte(PWMB);
    outFrame.data.byte[6] = highByte(PWMC);
    outFrame.data.byte[7] = lowByte(PWMC);
    Can0.sendFrame(outFrame);
    
    //Encoder position, Vd, Vq (which are outputs of PID loop)
    outFrame.id = settings.canBaseTx + 2;
    temp2 = controllerStatus.lastEncoderPos;
    outFrame.length = 8;
    outFrame.data.byte[0] = temp2 & 0xFF;
    outFrame.data.byte[1] = (temp2 >> 8) & 0xFF;
    outFrame.data.byte[2] = (temp2 >> 16) & 0xFF;
    outFrame.data.byte[3] = (temp2 >> 24) & 0xFF;
    outFrame.data.byte[4] = controllerStatus.Vd & 0xFF;
    outFrame.data.byte[5] = (controllerStatus.Vd >> 8) & 0xFF;
    outFrame.data.byte[6] = controllerStatus.Vq & 0xFF;
    outFrame.data.byte[7] = (controllerStatus.Vq >> 8) & 0xFF;
    Can0.sendFrame(outFrame);    

    outFrame.id = settings.canBaseTx + 3;
    outFrame.data.byte[0] = (elecVelo) & 0xFF;
    outFrame.data.byte[1] = (elecVelo >> 8) & 0xFF;
    outFrame.data.byte[2] = (elecVelo >> 16) & 0xFF;
    outFrame.data.byte[3] = (elecVelo >> 24);
    outFrame.data.byte[4] = Va & 0xFF;
    outFrame.data.byte[5] = (Va >> 8) & 0xFF;
    outFrame.data.byte[6] = Vb & 0xFF;
    outFrame.data.byte[7] = (Vb >> 8) & 0xFF;
    Can0.sendFrame(outFrame);
    
    outFrame.id = settings.canBaseTx + 4;
    outFrame.data.byte[0] = (controllerStatus.Id & 0xFF);
    outFrame.data.byte[1] = (controllerStatus.Id >> 8);
    outFrame.data.byte[2] = (controllerStatus.Iq & 0xFF);
    outFrame.data.byte[3] = (controllerStatus.Iq >> 8);
    outFrame.data.byte[4] = (Ia & 0xFF);
    outFrame.data.byte[5] = (Ia >> 8);
    outFrame.data.byte[6] = (Ib & 0xFF);
    outFrame.data.byte[7] = (Ib >> 8);
    Can0.sendFrame(outFrame);
#else
    //debugging message. Send RPM and filtered currents
    outFrame.id = settings.canBaseTx;
    outFrame.length = 8;
    outFrame.extended = false;
    outFrame.data.byte[0] = highByte(controllerStatus.rpm);
    outFrame.data.byte[1] = lowByte(controllerStatus.rpm);
    temp = controllerStatus.phaseCurrentFilteredA >> 16;
    outFrame.data.byte[2] = highByte(temp);
    outFrame.data.byte[3] = lowByte(temp);
    temp = controllerStatus.phaseCurrentFilteredB >> 16;
    outFrame.data.byte[4] = highByte(temp);
    outFrame.data.byte[5] = lowByte(temp);
    temp = controllerStatus.phaseCurrentFilteredC >> 16;
    outFrame.data.byte[6] = highByte(temp);
    outFrame.data.byte[7] = lowByte(temp);
    Can0.sendFrame(outFrame);
#endif
        
}
