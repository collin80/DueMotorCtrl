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
  if (settings.version != 0x10)
  {
      settings.busVoltageScale = 0.11875f;
      settings.canSpeed = 500000;
      settings.current1Scale = 0.001f;
      settings.current2Scale = 0.001f;
      settings.inverterTemp1Scale = 1.0f;
      settings.inverterTemp2Scale = 1.0f;
      settings.logLevel = 1;
      settings.version = 0x10;
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




