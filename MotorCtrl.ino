#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include "config.h"
#include "pwm.h"
#include "encoder.h"
#include "sinetable.h"

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
  pos = (pos + 1) & 0x3FF;

  a = sineTable[pos];
  b = sineTable[(pos + 341) & 0x3FF];
  c = sineTable[(pos + 682) & 0x3FF];
  
  updatePWM(a,b,c);

  delayMicroseconds(1000);

  /*
  SerialUSB.print("a: ");
  SerialUSB.println(a);  
  
  SerialUSB.print("b: ");
  SerialUSB.println(b);  

  SerialUSB.print("c: ");
  SerialUSB.println(c);  
*/
}




