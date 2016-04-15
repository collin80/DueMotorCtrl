#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include "config.h"
#include "pwm.h"
#include "encoder.h"
#include "sinetable.h"
#include "serialconsole.h"

int a, b, c, pos;

EEPROMSettings settings;

void setup() {

  analogReadResolution(12);
  
  Wire.begin();
  EEPROM.read(EEPROM_PAGE, settings);
  if (settings.version != 0x11)
  {
      settings.busVoltageScale = 7782;
      settings.canSpeed = 500000;
	  settings.canBaseAddr = 0x400;
      settings.current1Scale = 65;
      settings.current2Scale = 65;
      settings.inverterTemp1Scale = 65535;
      settings.inverterTemp2Scale = 65535;
	  settings.encoderCount = 425;
	  settings.encoderDirection = 255; //1 is incrementing forward, anything else is deincrementing forward
	  settings.maxRPM = 1000;
	  settings.maxTorque = 1000;
      settings.logLevel = 1;
      settings.version = 0x11;

      EEPROM.write(EEPROM_PAGE, settings);
  }
  
  setup_encoder();
  setup_pwm();

  //temporary junk just for testing
  digitalWrite(42, HIGH); //enable drive
  a = 0;
  b = 0;
  c = 0;
  pos = 0;
}

void loop() {
  static int count;
  int serialCnt;
  int in_byte;

  count++;
  if (count > 700)
  {
    count = 0;
    SerialUSB.println(getEncoderCount());
  }
  
  pos = (pos + 1) & 0x3FF;

  a = sineTable[pos];
  b = sineTable[(pos + 341) & 0x3FF];
  c = sineTable[(pos + 682) & 0x3FF];
  
  //updatePWM(a,b,c);

  delayMicroseconds(3000);

    serialCnt = 0;
	while ((SerialUSB.available() > 0) && serialCnt < 128) {
		serialCnt++;
		in_byte = SerialUSB.read();
		serialRXChar((uint8_t)in_byte);
	}
}




