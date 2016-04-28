/*
 * Various configuration options for the motor control program. As much as possible all settings that are done
 * at compile time are stored here. 
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include <Arduino.h>

#define PWM_FREQ			10000
#define MAX_PWM_DUTY		1050 //max value duty cycle can take
//if the value for PWM is within this number from either 0 or MAX then set it to either 0 or MAX. Prevents very short PWM pulses.
//For instance, using a value of 50 when max duty is 1050 and freq is 10000 = Each pwm tick is 10k * 1050 = 10.5M resolution.
//Thus, 50 / 10500000 = 4.76us minimum pulse width. But, then dead time is subtracted from that. So, plan accordingly
#define PWM_BUFFER			100
#define PWM_TARGET_DEADTIME	1300 //target dead time in nano seconds. So, 1000 is a micro second

#define DRIVE_ENABLE		42

#define EEPROM_PAGE			10

#define CFG_BUILD_NUM		1000

struct EEPROMSettings { //87 bytes so far. Keep under 256
	uint8_t version; //1
	uint8_t logLevel; //1
	
	uint32_t canSpeed; //4
	uint32_t canBaseRx; //4
	uint32_t canBaseTx; //4
	
	uint32_t busVoltageScale;   //4 - scale value is 65536 at 1 to 1 - this is shifted 16 bits mode
	uint32_t busVoltageBias;    //4 amount to remove from value to get to true zero
	uint32_t current1Scale;     //4
	uint32_t current1Bias;      //4
	uint32_t current2Scale;     //4
	uint32_t current2Bias;      //4
	uint32_t inverterTemp1Scale;//4
	uint32_t inverterTemp1Bias; //4
	uint32_t inverterTemp2Scale;//4	
	uint32_t inverterTemp2Bias; //4
	
	uint8_t motorType; //1 - 0 = induction, 1 = permanent magnet (sine), 2 = BLDC (trapezoid)
	uint32_t encoderCount; //4 - number of pulses per mechanical revolution
	uint8_t encoderDirection; //1
	uint8_t numPoles; //1
	uint32_t rotorTimeConst; // 4 - scaled up so that 65536 is 1
	int32_t pid_KP; //4 //pid parameters. - scaled up 4096 (12 bits)
	int32_t pid_KI; //4	
	int32_t pid_KD; //4 //usually D isn't used for motor controller tuning but just in case here it is.
	
	uint16_t maxRPM; //2 - whole RPMS 
	uint16_t maxTorque; //2 - in tenths of a Nm
	uint16_t maxAmpsDrive; // 2 - in tenths of an amp
	uint16_t maxAmpsRegen; // 2 - in tenths
	uint16_t vhzProperRPM; // 2 - RPM at which we should be up to full voltage.
};

//much of this is only relevant for debugging and internal operations
//I highly doubt any end user cares about IQ or theta.
struct STATUS
{
	int rmsCurrent;
	int phaseCurrentA;
	int phaseCurrentB;
	int phaseCurrentC;
	int Iq; //quadrature current - torque producing - Latest measured value
	int Id; //direct current - magnetizing
	int IqRef; //Where we wanted the value to actually be.
	int IdRef;
	uint32_t theta; //rotation angle. Uses sine table with 512 entries so range is 0 - 511
	int32_t lastEncoderPos;
	uint16_t rpm; //whole numbers - 1 rpm
};

extern EEPROMSettings settings;
extern volatile STATUS controllerStatus;

#endif