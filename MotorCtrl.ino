#include <due_can.h>
#include <FirmwareReceiver.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "encoder.h"
#include "dig_in.h"
#include "serialconsole.h"
#include "canbus.h"
#include "vhz.h"
#include "foc.h"

EEPROMSettings settings;
volatile STATUS controllerStatus;

//used for temporary debugging
extern volatile int interruptCount;
extern volatile uint16_t busVoltRaw;
extern volatile uint16_t current1Raw;
extern volatile uint16_t current2Raw;
extern volatile uint16_t invTemp1Raw;
extern volatile uint16_t invTemp2Raw;
extern volatile uint16_t motorTemp1Raw;
extern volatile uint16_t motorTemp2Raw;

//#define DEBUG_MSGS

void watchdogSetup(void)
{
  watchdogEnable(250); //number of milliseconds before wdt trips if you don't kick it (poor dog)
}

void setup() {

  analogReadResolution(12);
  
  PIO_Configure(PIOC, PIO_OUTPUT_0, PIO_PC20, PIO_DEFAULT); 

  Serial.begin(115200);
  
  Wire.begin();
  EEPROM.read(EEPROM_PAGE, settings);
  if (settings.version != EEPROM_VER)
  {
	settings.busVoltageScale = 7782;
	settings.busVoltageBias = 0;
	settings.canSpeed = 500000;
	settings.canBaseRx = 0x230;
	settings.canBaseTx = 0x410;
	settings.current1Scale = 32768;
	settings.current1Bias = 2008;
	settings.current2Scale = 32768;
	settings.current2Bias = 2008;
	settings.inverterTemp1Scale = 65536;
	settings.inverterTemp1Bias = 0;
	settings.inverterTemp2Scale = 65536;
	settings.inverterTemp2Bias = 0;
	settings.motorTemp1Bias = 0;
	settings.motorTemp1Scale = 65536;
	settings.motorTemp2Bias = 0;
	settings.motorTemp2Scale = 65536;
	settings.encoderCount = 42500;
	settings.encoderDirection = 1; //0 is incrementing forward, anything else is deincrementing forward
	settings.maxRPM = 1000;
	settings.maxTorque = 1000; //in tenths
	settings.motorType = 1; //PMAC
	settings.controlType = 1; //vhz = 0, 1 = FOC
	settings.numPoles = 12; //number of pairs really (N/S)
	settings.pid_KD = 0;
	settings.pid_KI = 80;
	settings.pid_KP = 4000;
	settings.rotorTimeConst = 655; //inductance of rotor over resistance of rotor (Lr/Rr)
	settings.maxAmpsDrive = 400; //in tenths so this isn't a lot of current
	settings.maxAmpsRegen = 200; //in tenths
	settings.logLevel = 1;
	settings.thetaOffset = 0;
	settings.vhzProperRPM = 1800;
	settings.hallAB = 2;
	settings.hallBC = 1;
	settings.hallCA = 0;
	settings.version = EEPROM_VER;
	EEPROM.write(EEPROM_PAGE, settings);
  }
  
	controllerStatus.rmsCurrent = 0;
	controllerStatus.phaseCurrentRawA = 0;
	controllerStatus.phaseCurrentRawB = 0;
	controllerStatus.phaseCurrentRawC = 0;
    controllerStatus.phaseCurrentFilteredA = 0;
    controllerStatus.phaseCurrentFilteredB = 0;
    controllerStatus.phaseCurrentFilteredC = 0;
	controllerStatus.Iq = 0; 
	controllerStatus.Id = 0;
	controllerStatus.IqRef = 0; 
	controllerStatus.IdRef = 0;
	controllerStatus.theta = 0; 
	controllerStatus.lastEncoderPos = 0;
	controllerStatus.rpm = 0;
	controllerStatus.runningOffsetTest = false;
	controllerStatus.rampingTest = false;
	controllerStatus.rampingUp = true;
	controllerStatus.rampRPM = 0;
    controllerStatus.inReverse = false; //start out defaulting to forward

  setup_encoder();
  setup_digital_inputs();

  if (settings.controlType == 0) setupVHz();
  if (settings.controlType == 1) setupFOC();
  setup_adc();
  setup_pwm();
  setup_CAN();
  
  //this block of code averages current readings over 1 second and uses that for the bias
  //ignoring the set bias from above. So sorry...
  /*
  int accum1, accum2;
  accum1 = 0;
  accum2 = 0;
  for (int i = 0; i < 200; i++)
  {
        watchdogReset();
        accum1 += current1Raw;
        accum2 += current2Raw;
        delay(5);
  }
  accum1 /= 200;
  accum2 /= 200;
  if (accum1 > 1900 && accum1 < 2100)
  {
    settings.current1Bias = accum1;    
  }
  if (accum2 > 1900 && accum2 < 2100)
  {
    settings.current2Bias = accum2;
  }*/
  
  //digitalWrite(42, HIGH); //enable drive
  //if (settings.controlType == 0) setVHzSpeed(10);
  //if (settings.controlType == 0) startVHZOffsetTest();
}

void loop() {
	static int count;
	int serialCnt;
	int in_byte;
	static int rampingCount = 0;

	watchdogReset();
  
	canRX(); //see if we've got any messages to input and parse
  
	count++;
	if (count > 200)
	{
		count = 0;
#ifdef DEBUG_MSGS
		SerialUSB.print("En:");
		SerialUSB.println(getEncoderCount());
		SerialUSB.print("V:");
		SerialUSB.println(getBusVoltage() >> 16);
		SerialUSB.print("C1:");
		SerialUSB.println(getCurrent1() >> 16);
		SerialUSB.print("C2:");
		SerialUSB.println(getCurrent2() >> 16);
		SerialUSB.print("IT1:");
		SerialUSB.println(invTemp1Raw);
		SerialUSB.print("IT2:");
		SerialUSB.println(invTemp2Raw);
		SerialUSB.print("MT1:");
		SerialUSB.println(motorTemp1Raw);
		SerialUSB.print("MT1:");
		SerialUSB.println(motorTemp2Raw);
		SerialUSB.print("D1:");
		SerialUSB.println(getDigitalInput(0));
		SerialUSB.print("D2:");
		SerialUSB.println(getDigitalInput(1));
		SerialUSB.print("D3:");
		SerialUSB.println(getDigitalInput(2));
		SerialUSB.print("D4:");
		SerialUSB.println(getDigitalInput(3));
		//if (PWM->PWM_FSR & (1 << 12)) SerialUSB.println("Faulted!");
		SerialUSB.println();
#endif
	}
	delayMicroseconds(2000);
  
	serialCnt = 0;
	while ((SerialUSB.available() > 0) && serialCnt < 128) {
		serialCnt++;
		in_byte = SerialUSB.read();
		serialRXChar((uint8_t)in_byte);
	}

	if (controllerStatus.rampingTest)
	{
		rampingCount++;
		if (rampingCount > 250)
		{
			rampingCount = 0;
			if (controllerStatus.rampingUp) 
			{
				controllerStatus.rampRPM++;
				if (controllerStatus.rampRPM > 400) controllerStatus.rampingUp = false;
			}
			else 
			{
				controllerStatus.rampRPM--;
				if (controllerStatus.rampRPM < 4) controllerStatus.rampingUp = true;
			}
			setVHzSpeed(controllerStatus.rampRPM);
		}
	}
	else
	{
		rampingCount++;
		if (rampingCount > 50)
		{
			rampingCount = 0;
			if (controllerStatus.rampRPM > 0) 
			{
				controllerStatus.rampRPM--;
				setVHzSpeed(controllerStatus.rampRPM);
			}
		}
	}
}




