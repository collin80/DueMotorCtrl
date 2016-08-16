#include <Arduino.h>
#include "vhz.h"
#include "config.h"
#include "pwm.h"
#include "sinetable.h"
#include "adc.h"
#include "encoder.h"
#include "dig_in.h"
#include <due_can.h>

volatile int32_t posAccum;
volatile int32_t posInc;
volatile int currentRotorPosition, targetRotorPosition;
volatile int lastEncoderPos;
volatile uint32_t vhzCounter;
volatile int a, b, c;
volatile int currentSector, lastSector;
volatile bool needSectorCorrection;
volatile int pwmPower = 0;
//temp consts for testing
const int pwmMinPower = 30; //Lowest ratio of power to allow. in tenths of a percent
const int pwmMaxPower = 1000; //highest ratio of power to allow. also 1/10 of a %
const int fullRPM = 1000; //RPM at which we go up to full power.

OFFSET_TEST offsetVhz;

void setupVHz()
{
    posInc = 0;
    posAccum = 0;
    currentRotorPosition = 0;
    targetRotorPosition = 0;
    vhzCounter = 0;
    a=0;
    b = 0;
    c = 0;
    needSectorCorrection = true;

    updatePWM(0,0,0);

    if (settings.hallAB != 255 && settings.hallBC != 255 & settings.hallCA != 255)
    {
        int sector = getMotorSector();
        if (sector == 0)
        {
            //SerialUSB.println("Not using hall effect sensors for initial position");
            return;
        }
        //SerialUSB.print("Hall effect sensors indicate we're starting in sector ");
        //SerialUSB.println(sector);
        controllerStatus.theta = ((sector - 1) * 85) + 42;
        targetRotorPosition = controllerStatus.theta;
        currentSector = lastSector = sector;
    }
}

//this target RPM is in mechanical / motor RPM not electrical RPM
//Smoothly scales post and pre multipliers to keep within proper range. Output variable posInc
//is scaled up 65536 in the end.u
void setVHzSpeed(int targetRPM)
{
    /*int elecRPM = (targetRPM * settings.numPoles);
    int preMultiplier = 16;
    int postMultiplier = 0;
    int testVal = elecRPM;
    */

    controllerStatus.rpm = targetRPM;

    if (targetRPM < 4)
    {
        digitalWrite(42, LOW); //disable drive
        updatePWM(0,0,0);
        posInc = 0;
        return;
    }

    digitalWrite(42, HIGH); //enable drive

    /*while (testVal > 1024)
    {
        testVal = testVal >> 1;
        preMultiplier--;
        postMultiplier++;
    }*/

    posInc = (((16777216ul * settings.numPoles) / PWM_FREQ) * targetRPM) / 60;
    
    //posInc = ((((elecRPM * 512) / 60) << preMultiplier) / 10000) << postMultiplier;
    
    if (targetRPM > fullRPM) pwmPower = pwmMaxPower;
    else 
    {
        pwmPower = pwmMaxPower - ((pwmMaxPower * (fullRPM - targetRPM)) / fullRPM);
        if (pwmPower < pwmMinPower) pwmPower = pwmMinPower;
    }
}

void updatePosVHz()
{
    volatile int localRotorPos;

    //if (posInc == 0) return;

    vhzCounter++;

    controllerStatus.phaseCurrentRawB = getCurrent2();
    controllerStatus.phaseCurrentRawC = getCurrent1();
    controllerStatus.phaseCurrentRawA = -controllerStatus.phaseCurrentRawB - controllerStatus.phaseCurrentRawC;
    
    controllerStatus.phaseCurrentFilteredA = ((controllerStatus.phaseCurrentRawA * 30) + (controllerStatus.phaseCurrentFilteredA * 70)) / 100;
    controllerStatus.phaseCurrentFilteredB = ((controllerStatus.phaseCurrentRawB * 30) + (controllerStatus.phaseCurrentFilteredB * 70)) / 100;
    controllerStatus.phaseCurrentFilteredC = ((controllerStatus.phaseCurrentRawC * 30) + (controllerStatus.phaseCurrentFilteredC * 70)) / 100; 
/*
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
*/   
    
    //the index pulse happens at the proper zero point.
    localRotorPos = -getEncoderCount(); //encoder is backward on test device
    localRotorPos = (((localRotorPos * 512l * (int32_t)settings.numPoles) / ((int)settings.encoderCount * 4L)));
    //localRotorPos = (((localRotorPos * 512l * 12l) / (170000)));
    
    posAccum += posInc;

    targetRotorPosition = localRotorPos + (posAccum >> 15);

    posAccum &= 0x7FFF;

    targetRotorPosition &= 0x1FF; //result of all above calcs must be constrained to 0-511

    //currently not using thetaOffset
    //localRotorPos = (targetRotorPosition + settings.thetaOffset) & 511;
    
    controllerStatus.theta = targetRotorPosition;
    
    a = ( (_sin_times32768[targetRotorPosition] + 32768) * pwmPower) / 65536;
    c = ( (_sin_times32768[(targetRotorPosition + 170) & 511]+32768) * pwmPower) / 65536;
    b = ( (_sin_times32768[(targetRotorPosition + 341) & 511]+32768) * pwmPower) / 65536;
    
    //SVM style PWM output - This is optional. Can be commented out for traditional PWM
    if (a <= b)
    {
        if (a <= c) //A is smallest of all
        {
            b -= a;
            c -= a;
            a = 0;
        }
        else //C is smallest then
        {
            a -= c;
            b -= c;
            c = 0;
        }
    }
    else
    {
        if (b <= c) //B is smallest
        {
            a -= b;
            c -= b;
            b = 0;
        }
        else //C is the smallest
        {
            a -= c;
            b -= c;
            c = 0;
        }
    }
    
    updatePWM(a,b,c);

    if (vhzCounter > 14) 
    {
        vhzCounter = 0;
        sendVHzCANMsgs();
    }
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
    int32_t temp2;

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

    outFrame.id = settings.canBaseTx + 1;
    temp = (getBusVoltage() >> 17);
    outFrame.data.byte[0] = lowByte(temp);
    outFrame.data.byte[1] = lowByte(getMotorSector());
    outFrame.data.byte[2] = highByte(a);
    outFrame.data.byte[3] = lowByte(a);
    outFrame.data.byte[4] = highByte(b);
    outFrame.data.byte[5] = lowByte(b);
    outFrame.data.byte[6] = highByte(c);
    outFrame.data.byte[7] = lowByte(c);
    Can0.sendFrame(outFrame);

    outFrame.id = settings.canBaseTx + 2;
    temp2 = getEncoderCount();
    outFrame.length = 8;
    outFrame.data.byte[0] = temp2 & 0xFF;
    outFrame.data.byte[1] = (temp2 >> 8) & 0xFF;
    outFrame.data.byte[2] = (temp2 >> 16) & 0xFF;
    outFrame.data.byte[3] = (temp2 >> 24) & 0xFF;
    outFrame.data.byte[4] = getDigitalInput(0);
    outFrame.data.byte[5] = getDigitalInput(1);
    outFrame.data.byte[6] = getDigitalInput(2);
    outFrame.data.byte[7] = getDigitalInput(3);
    Can0.sendFrame(outFrame);
    
}

