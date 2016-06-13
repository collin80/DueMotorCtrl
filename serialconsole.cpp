#include <Arduino.h>
#include <due_wire.h>
#include <Wire_EEPROM.h>
#include <due_can.h>
#include "serialconsole.h"
#include "config.h"
#include "Logger.h"
#include "pwm.h"
#include "vhz.h"
#include "foc.h"

char cmdBuffer[80];
int ptrBuffer;

void serialInit() {
	//State variables for serial console
	ptrBuffer = 0;      
}

void serialPrintMenu() {
	char buff[80];
	//Show build # here as well in case people are using the native port and don't get to see the start up messages
	SerialUSB.print("Build number: ");
	SerialUSB.println(CFG_BUILD_NUM);
	SerialUSB.println("System Menu:");
	SerialUSB.println();
	SerialUSB.println("Enable line endings of some sort (LF, CR, CRLF)");
	SerialUSB.println();
	SerialUSB.println("Short Commands:");
	SerialUSB.println("h = help (displays this message)");
	SerialUSB.println("R = reset to factory defaults");
	SerialUSB.println("X = Find angular offset (PM motors)");
	SerialUSB.println("Y = Toggle testing torque On/Off");
	SerialUSB.println("Z = Toggle ramping test");
	SerialUSB.println();
	SerialUSB.println("Config Commands (enter command=newvalue). Current values shown in parenthesis:");
	SerialUSB.println();

	Logger::console("LOGLEVEL=%i - set log level (0=debug, 1=info, 2=warn, 3=error, 4=off)", settings.logLevel);
	SerialUSB.println();

	Logger::console("CANSPEED=%i - Set speed of CAN in baud (125000, 250000, etc)", settings.canSpeed);
	Logger::console("CANRXBASE=%X - Set base address for command messages to inverter", settings.canBaseRx);
	Logger::console("CANTXBASE=%X - Set base address for status messages from inverter", settings.canBaseTx);
	SerialUSB.println();
	
	Logger::console("BUSVOLTSCALE=%f - Set bus voltage scaling factor", settings.busVoltageScale / 65536.0f);
	Logger::console("CURR1SCALE=%f - Set current sensor 1 scaling factor", settings.current1Scale / 65536.0f);
	Logger::console("CURR2SCALE=%f - Set current sensor 2 scaling factor", settings.current2Scale / 65536.0f);
	Logger::console("INVTEMP1SCALE=%f - Set inverter temp 1 scaling factor", settings.inverterTemp1Scale / 65536.0f);
	Logger::console("INVTEMP2SCALE=%f - Set inverter temp 2 scaling factor", settings.inverterTemp2Scale / 65536.0f);
	SerialUSB.println();

	Logger::console("BUSVOLTBIAS=%i - Set bus voltage bias", settings.busVoltageBias);
	Logger::console("CURR1BIAS=%i - Set current sensor 1 bias", settings.current1Bias);
	Logger::console("CURR2BIAS=%i - Set current sensor 2 bias", settings.current2Bias);
	Logger::console("INVTEMP1BIAS=%i - Set inverter temp 1 bias", settings.inverterTemp1Bias);
	Logger::console("INVTEMP2BIAS=%i - Set inverter temp 2 bias", settings.inverterTemp2Bias);
	SerialUSB.println();
	
	Logger::console("MOTORTYPE=%i - Set motor type (0 = induction, 1 = PMAC, 2 = BLDC (trapezoid)", settings.motorType);
	Logger::console("CTRLTYPE=%i - Set control type (0 = V/Hz, 1 = FOC)", settings.controlType);
	Logger::console("MOTORNUMPOLES=%i - Set number of pole pairs", settings.numPoles);
	Logger::console("MOTORTIMECONST=%f - Set motor time constant (for induction motors)", settings.rotorTimeConst / 65536.0f);
	Logger::console("ENCODERCOUNT=%i - Set number of encoder pulses per rev", settings.encoderCount);
	Logger::console("ENCODERDIR=%i - Set encoder direction (0 = count up forward, 1 = count up backward)", settings.encoderDirection);
	Logger::console("PIDKP = %f - Set P parameter of PID loop", settings.pid_KP / 65536.0f);
	Logger::console("PIDKI = %f - Set I parameter of PID loop", settings.pid_KI / 65536.0f);
	Logger::console("PIDKD = %f - Set D parameter of PID loop", settings.pid_KD / 65536.0f);
	SerialUSB.println();
	
	Logger::console("MAXRPM=%i - Set maximum RPM", settings.maxRPM);
	Logger::console("MAXTRQ=%i - Set maximum torque (0.1Nm scale)", settings.maxTorque);
	Logger::console("MAXAMPDRIVE=%i - Set maximum amperage while in drive/reverse (0.1A scale)", settings.maxAmpsDrive);
	Logger::console("MAXAMPREGEN=%i - Set maximum amperage for regen (0.1A scale)", settings.maxAmpsRegen);
	SerialUSB.println();

	Logger::console("VHZRPM=%i - Set V/Hz mode target RPM (for debugging only)", controllerStatus.rpm);
	Logger::console("ANGLEOFFSET=%i - Set anglular offset (0 - 511)", settings.thetaOffset);
}

/*	There is a help menu (press H or h or ?)
 This is no longer going to be a simple single character console.
 Now the system can handle up to 80 input characters. Commands are submitted
 by sending line ending (LF, CR, or both)
 */
void serialRXChar(uint8_t chr) {
	if (chr == 10 || chr == 13) { //command done. Parse it.
		handleConsoleCmd();
		ptrBuffer = 0; //reset line counter once the line has been processed
	} else {
		cmdBuffer[ptrBuffer++] = (unsigned char) chr;
		if (ptrBuffer > 79)
			ptrBuffer = 79;
	}
}

void handleConsoleCmd() {	
	if (ptrBuffer == 1) 
	{ //command is a single ascii character
		handleShortCmd();
	} 
	else //at least two bytes 
	{
        boolean equalSign = false;
		for (int i = 0; i < ptrBuffer; i++) if (cmdBuffer[i] == '=') equalSign = true;
		if (equalSign) handleConfigCmd();
	}
	ptrBuffer = 0; //reset line counter once the line has been processed
}


/*For simplicity the configuration setting code uses four characters for each configuration choice. This makes things easier for
 comparison purposes.
 */
void handleConfigCmd() {
	int i;
	int newValue;
	float newFloatVal;
	char *newString;
	bool writeEEPROM = false;

	//Logger::debug("Cmd size: %i", ptrBuffer);
	if (ptrBuffer < 6)
		return; //4 digit command, =, value is at least 6 characters
	cmdBuffer[ptrBuffer] = 0; //make sure to null terminate
	String cmdString = String();
	unsigned char whichEntry = '0';
	i = 0;

	while (cmdBuffer[i] != '=' && i < ptrBuffer) {
	 cmdString.concat(String(cmdBuffer[i++]));
	}
	i++; //skip the =
	if (i >= ptrBuffer)
	{
		Logger::console("Command needs a value..ie TORQ=3000");
		Logger::console("");
		return; //or, we could use this to display the parameter instead of setting
	}

	// strtol() is able to parse also hex values (e.g. a string "0xCAFE"), useful for enable/disable by device id
	newValue = strtol((char *) (cmdBuffer + i), NULL, 0); //try to turn the string into a number
	newFloatVal = strtof((char *) (cmdBuffer + i), 0);
	newString = (char *)(cmdBuffer + i); //leave it as a string

	cmdString.toUpperCase();

	if (cmdString == String("CANSPEED")) {
		if (newValue > 0 && newValue <= 1000000) 
		{
			Logger::console("Setting CAN Baud Rate to %i", newValue);
			settings.canSpeed = newValue;
			Can0.begin(settings.canSpeed, 255);
			writeEEPROM = true;
		}
		else Logger::console("Invalid baud rate! Enter a value 1 - 1000000"); 
	} else if (cmdString == String("CANRXBASE")) {
		if (newValue > 0 && newValue <= 0x7FF) 
		{
			Logger::console("Setting CAN RX Base to %X", newValue);
			settings.canBaseRx = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid base address. Must be more than 0 and less than 0x7FF"); 
	} else if (cmdString == String("CANTXBASE")) {
		if (newValue > 0 && newValue <= 0x7FF) 
		{
			Logger::console("Setting CAN TX Base to %X", newValue);
			settings.canBaseTx = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid base address. Must be more than 0 and less than 0x7FF");	  
	} else if (cmdString == String("BUSVOLTBIAS")) {
		if (newValue > 0 && newValue <= 0xFFFF) 
		{
			Logger::console("Setting bus voltage ADC bias to %i", newValue);
			settings.busVoltageBias = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid ADC bias value. Must be between 0 and 65535");	  
	} else if (cmdString == String("CURR1BIAS")) {
		if (newValue > 0 && newValue <= 0xFFFF) 
		{
			Logger::console("Setting current 1 ADC bias to %i", newValue);
			settings.current1Bias = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid ADC bias value. Must be between 0 and 65535");	  
	} else if (cmdString == String("CURR2BIAS")) {
		if (newValue > 0 && newValue <= 0xFFFF) 
		{
			Logger::console("Setting current 2 ADC bias to %i", newValue);
			settings.current2Bias = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid ADC bias value. Must be between 0 and 65535");	  
	} else if (cmdString == String("INVTEMP1BIAS")) {
		if (newValue > 0 && newValue <= 0xFFFF) 
		{
			Logger::console("Setting temp 1 bias ADC bias to %i", newValue);
			settings.inverterTemp1Bias = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid ADC bias value. Must be between 0 and 65535");	  
	} else if (cmdString == String("INVTEMP2BIAS")) {
		if (newValue > 0 && newValue <= 0xFFFF) 
		{
			Logger::console("Setting temp 2 bias ADC bias to %i", newValue);
			settings.inverterTemp2Bias = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid ADC bias value. Must be between 0 and 65535");	  
	} else if (cmdString == String("BUSVOLTSCALE")) {
		if (newFloatVal > 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting Bus voltage scaling factor to %f", newFloatVal);
			settings.busVoltageScale = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a scaling factor between 0 and 10.0");	  
	} else if (cmdString == String("CURR1SCALE")) {
		if (newFloatVal > 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting current sensor 1 scaling factor to %f", newFloatVal);
			settings.current1Scale = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a scaling factor between 0 and 10.0");	  
	} else if (cmdString == String("CURR2SCALE")) {
		if (newFloatVal > 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting current sensor 2 scaling factor to %f", newFloatVal);
			settings.current2Scale = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a scaling factor between 0 and 10.0");	  
	} else if (cmdString == String("INVTEMP1SCALE")) {
		if (newFloatVal > 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting temperature sensor 1 scaling factor to %f", newFloatVal);
			settings.inverterTemp1Scale = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a scaling factor between 0 and 10.0");	  
	} else if (cmdString == String("INVTEMP2SCALE")) {
		if (newFloatVal > 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting temperature sensor 2 scaling factor to %f", newFloatVal);
			settings.inverterTemp2Scale = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a scaling factor between 0 and 10.0");	  
	} else if (cmdString == String("MOTORTIMECONST")) {
		if (newFloatVal > 0.0f && newFloatVal <= 1.0f) 
		{
			Logger::console("Setting motor time constant to %f", newFloatVal);
			settings.rotorTimeConst = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a time constant more than 0 and less than 1.0");	  		
	} else if (cmdString == String("MOTORTYPE")) {
		if (newValue >= 0 && newValue <= 2) 
		{
			Logger::console("Setting motor type to %i", newValue);
			settings.motorType = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid motor type. Please enter 0, 1, or 2"); 
	} else if (cmdString == String("CTRLTYPE")) {
		if (newValue >= 0 && newValue <= 1) 
		{
			Logger::console("Setting control type to %i", newValue);
			settings.controlType = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid control type. Please enter 0 or 1"); 
	} else if (cmdString == String("MOTORNUMPOLES")) {
		if (newValue > 0 && newValue <= 40) 
		{
			Logger::console("Setting motor pole pairs to %i", newValue);
			settings.numPoles = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid number of pole pairs. Must be at least one and less than 40"); 
	} else if (cmdString == String("ENCODERCOUNT")) {
		if (newValue > 0 && newValue <= 100000) 
		{
			Logger::console("Setting encoder pulses per rev to %i", newValue);
			settings.encoderCount = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid encoder count. Value must be more than 0 and less than 100000"); 
	} else if (cmdString == String("ENCODERDIR")) {
		if (newValue >= 0 && newValue <= 1) 
		{
			Logger::console("Setting encoder direction to %i", newValue);
			settings.encoderDirection = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid encoder direction. Value must be either 0 or 1"); 
	} else if (cmdString == String("PIDKP")) {
		if (newFloatVal >= 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting PID P coef to %f", newFloatVal);
			settings.pid_KP = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a coefficient between 0.0 and 10.0");	  
	} else if (cmdString == String("PIDKI")) {
		if (newFloatVal >= 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting PID I coef to %f", newFloatVal);
			settings.pid_KI = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a coefficient between 0.0 and 10.0");	  
	} else if (cmdString == String("PIDKD")) {
		if (newFloatVal >= 0.0f && newFloatVal <= 10.0f) 
		{
			Logger::console("Setting PID D coef to %f", newFloatVal);
			settings.pid_KD = (uint32_t)(newFloatVal * 65536.0f);
			writeEEPROM = true;
		}
		else Logger::console("Please enter a coefficient between 0.0 and 10.0");	  
	} else if (cmdString == String("VHZRPM")) {
		if (newValue >= 0 && newValue <= settings.maxRPM) 
		{
			Logger::console("Setting V/Hz target RPM to %i", newValue);
			controllerStatus.rpm = newValue;
			setVHzSpeed(newValue);
		}
		else Logger::console("Invalid max RPM. Must be positive and less than your max RPM setting"); 
	} else if (cmdString == String("ANGLEOFFSET")) {
		if (newValue >= 0 && newValue <= 511) 
		{
			Logger::console("Setting angular offset to %i", newValue);
			settings.thetaOffset = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid angular offset. Must be between 0 and 511"); 		
	} else if (cmdString == String("MAXRPM")) {
		if (newValue > 0 && newValue <= 30000) 
		{
			Logger::console("Setting max RPM to %i", newValue);
			settings.maxRPM = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid max RPM. Must be more than 0 and less than 30000"); 
	} else if (cmdString == String("MAXTRQ")) {
		if (newValue > 0 && newValue <= 30000) 
		{
			Logger::console("Setting max torque to %i", newValue);
			settings.maxTorque = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid max torque. Must be more than 0 and less than 30000"); 
	} else if (cmdString == String("MAXAMPDRIVE")) {
		if (newValue > 0 && newValue <= 20000) 
		{
			Logger::console("Setting max drive amperage to %i", newValue);
			settings.maxAmpsDrive = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid max drive amperage. Must be more than 0 and less than 20000"); 
	} else if (cmdString == String("MAXAMPREGEN")) {
		if (newValue > 0 && newValue <= 20000) 
		{
			Logger::console("Setting max regen amperage to %i", newValue);
			settings.maxAmpsDrive = newValue;
			writeEEPROM = true;
		}
		else Logger::console("Invalid max regen amperage. Must be more than 0 and less than 20000"); 
	} else if (cmdString == String("LOGLEVEL")) {
		switch (newValue) {
		case 0:
			Logger::setLoglevel(Logger::Debug);
			Logger::console("setting loglevel to 'debug'");
			writeEEPROM = true;
			break;
		case 1:
			Logger::setLoglevel(Logger::Info);
			Logger::console("setting loglevel to 'info'");
			writeEEPROM = true;
			break;
		case 2:
			Logger::console("setting loglevel to 'warning'");
			Logger::setLoglevel(Logger::Warn);
			writeEEPROM = true;
			break;
		case 3:
			Logger::console("setting loglevel to 'error'");
			Logger::setLoglevel(Logger::Error);
			writeEEPROM = true;
			break;
		case 4:
			Logger::console("setting loglevel to 'off'");
			Logger::setLoglevel(Logger::Off);
			writeEEPROM = true;
			break;
		}
	} 
	else {
		Logger::console("Unknown command");
	}
	if (writeEEPROM) 
	{
		EEPROM.write(EEPROM_PAGE, settings);
	}
}

void handleShortCmd() {
	uint8_t val;

	switch (cmdBuffer[0]) {
	case 'h':
	case '?':
	case 'H':
		serialPrintMenu();
		break;
	case 'R': //reset to factory defaults.
		settings.version = 0xFF;
		EEPROM.write(EEPROM_PAGE, settings);
		Logger::console("Power cycle to reset to factory defaults");
		break;
	case 'X':
		Logger::console("Starting the offset test");
		if (settings.controlType == 1) startFOCOffsetTest();
    if (settings.controlType == 0) startVHZOffsetTest();
		break;
	case 'Y':
		if (controllerStatus.IqRef != 0)
		{
			Logger::console("Stopping torque output");
			controllerStatus.IqRef = 0;
			controllerStatus.IdRef = 0;
		}
		else
		{
			Logger::console("Starting torque output");
			controllerStatus.IqRef = 150;
			controllerStatus.IdRef = 0;
		}
		break;
	case 'Z':
		if (controllerStatus.rampingTest)
		{
			Logger::console("Stopping RPM ramp test");
			controllerStatus.rampingTest = false;
		}
		else
		{
			Logger::console("Starting RPM ramp test");
			controllerStatus.rampingUp = true;
			controllerStatus.rampRPM = 0;
			controllerStatus.rampingTest = true;
			setVHzSpeed(0);
		}
		break;
	}
}

unsigned int parseHexCharacter(char chr)
{
	unsigned int result = 0;
	if (chr >= '0' && chr <= '9') result = chr - '0';
	else if (chr >= 'A' && chr <= 'F') result = 10 + chr - 'A';
	else if (chr >= 'a' && chr <= 'f') result = 10 + chr - 'a';
	
	return result;
}

unsigned int parseHexString(char *str, int length)
{
    unsigned int result = 0;
    for (int i = 0; i < length; i++) result += parseHexCharacter(str[i]) << (4 * (length - i - 1));
    return result;
}

