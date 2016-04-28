#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include "config.h"
#include "adc.h"
#include "pwm.h"
#include "encoder.h"
#include "serialconsole.h"
#include "canbus.h"
#include "vhz.h"

EEPROMSettings settings;
volatile STATUS controllerStatus;

//used for temporary debugging
extern volatile int interruptCount;
extern volatile uint16_t busVoltRaw;
extern volatile uint16_t current1Raw;
extern volatile uint16_t current2Raw;
extern volatile uint16_t invTemp1Raw;
extern volatile uint16_t invTemp2Raw;

void setup() {

  analogReadResolution(12);
  
  Wire.begin();
  EEPROM.read(EEPROM_PAGE, settings);
  if (settings.version != 0x13)
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
	settings.encoderCount = 425;
	settings.encoderDirection = 255; //1 is incrementing forward, anything else is deincrementing forward
	settings.maxRPM = 1000;
	settings.maxTorque = 1000; //in tenths
	settings.motorType = 0; //induction motor
	settings.numPoles = 4; //number of pairs really (N/S)
	settings.pid_KD = 0;
	settings.pid_KI = 80;
	settings.pid_KP = 4000;
	settings.rotorTimeConst = 655; //inductance of rotor over resistance of rotor (Lr/Rr)
	settings.maxAmpsDrive = 400; //in tenths so this isn't a lot of current
	settings.maxAmpsRegen = 200; //in tenths
	settings.logLevel = 1;
	settings.version = 0x13;
	EEPROM.write(EEPROM_PAGE, settings);
  }
  
  setup_encoder();
  setupVHz();
  setup_adc();
  setup_pwm();
  setup_CAN();
  
  setVHzSpeed(5); //ask for 5 RPM from V/Hz control system.

  //temporary junk just for testing
  digitalWrite(42, HIGH); //enable drive
}

void loop() {
  static int count;
  int serialCnt;
  int in_byte;
  
  canRX(); //see if we've got any messages to input and parse
  
  count++;
  if (count > 200)
  {
    count = 0;
    SerialUSB.println(getEncoderCount());
	SerialUSB.println(getBusVoltage() >> 16);
	SerialUSB.println(getCurrent1() >> 16);
	SerialUSB.println(getCurrent2() >> 16);
    SerialUSB.println();
	SerialUSB.println(busVoltRaw);
	SerialUSB.println(current1Raw);
	SerialUSB.println(current1Raw);
	SerialUSB.println();
  }
  delay(2);
  
    serialCnt = 0;
	while ((SerialUSB.available() > 0) && serialCnt < 128) {
		serialCnt++;
		in_byte = SerialUSB.read();
		serialRXChar((uint8_t)in_byte);
	}
}




