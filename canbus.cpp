#include <Arduino.h>
#include <due_can.h>
#include "canbus.h"
#include "config.h"

void setup_CAN()
{
    Can0.begin(settings.canSpeed);
    Can0.setRXFilter(0, 0x7F0, settings.canBaseRx, false);
}

void canRX()
{
  
}
