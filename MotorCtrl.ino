#include <due_can.h>
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

void watchdogSetup(void)
{
  watchdogEnable(250); //number of milliseconds before wdt trips if you don't kick it (poor dog)
}

void setup() {

  analogReadResolution(12);
  
  PIO_Configure(PIOC, PIO_OUTPUT_0, PIO_PC20, PIO_DEFAULT); 
  
  Wire.begin();
  EEPROM.read(EEPROM_PAGE, settings);
  if (settings.version != EEPROM_VER)
  {
	settings.busVoltageScale = 7782;
	settings.busVoltageBias = 0;
	settings.canSpeed = 500000;
	settings.canBaseRx = 0x230;
	settings.canBaseTx = 0x410;
	settings.current1Scale = 65;
	settings.current1Bias = 2013;
	settings.current2Scale = 65;
	settings.current2Bias = 2013;
	settings.inverterTemp1Scale = 65536;
	settings.inverterTemp1Bias = 0;
	settings.inverterTemp2Scale = 65536;
	settings.inverterTemp2Bias = 0;
	settings.motorTemp1Bias = 0;
	settings.motorTemp1Scale = 65536;
	settings.motorTemp2Bias = 0;
	settings.motorTemp2Scale = 65536;
	settings.encoderCount = 425;
	settings.encoderDirection = 255; //1 is incrementing forward, anything else is deincrementing forward
	settings.maxRPM = 1000;
	settings.maxTorque = 1000; //in tenths
	settings.motorType = 0; //induction motor
	settings.controlType = 1; //FOC
	settings.numPoles = 4; //number of pairs really (N/S)
	settings.pid_KD = 0;
	settings.pid_KI = 80;
	settings.pid_KP = 4000;
	settings.rotorTimeConst = 655; //inductance of rotor over resistance of rotor (Lr/Rr)
	settings.maxAmpsDrive = 400; //in tenths so this isn't a lot of current
	settings.maxAmpsRegen = 200; //in tenths
	settings.logLevel = 1;
	settings.thetaOffset = 0;
	settings.vhzProperRPM = 1800;
	settings.hallAB = 255;
	settings.hallBC = 255;
	settings.hallCA = 255;
	settings.version = EEPROM_VER;
	EEPROM.write(EEPROM_PAGE, settings);
  }
  
	controllerStatus.rmsCurrent = 0;
	controllerStatus.phaseCurrentA = 0;
	controllerStatus.phaseCurrentB = 0;
	controllerStatus.phaseCurrentC = 0;
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

  setup_encoder();
  setup_digital_inputs();

  if (settings.controlType == 0) setupVHz();
  if (settings.controlType == 1) setupFOC();
  setup_adc();
  setup_pwm();
  setup_CAN();

  //digitalWrite(42, HIGH); //enable drive
  //if (settings.controlType == 0) setVHzSpeed(5);
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
	}
	delay(2);
  
	serialCnt = 0;
	while ((SerialUSB.available() > 0) && serialCnt < 128) {
		serialCnt++;
		in_byte = SerialUSB.read();
		serialRXChar((uint8_t)in_byte);
	}

	if (controllerStatus.rampingTest || getDigitalInput(3))
	{
		rampingCount++;
		if (rampingCount > 250)
		{
			rampingCount = 0;
			if (controllerStatus.rampingUp) 
			{
				controllerStatus.rampRPM++;
				if (controllerStatus.rampRPM > 40) controllerStatus.rampingUp = false;
			}
			else 
			{
				controllerStatus.rampRPM--;
				if (controllerStatus.rampRPM < 2) controllerStatus.rampingUp = true;
			}
			setVHzSpeed(controllerStatus.rampRPM);
		}
	}
	else if (!getDigitalInput(3))
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




