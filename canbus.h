#ifndef CAN_H_
#define CAN_H_

#include <Arduino.h>
#include <due_can.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include <FirmwareReceiver.h>
#include "vhz.h"
#include "foc.h"
#include "config.h"

void setup_CAN();
void canRX();
uint8_t calcCRC8(CAN_FRAME &frame);

#endif

