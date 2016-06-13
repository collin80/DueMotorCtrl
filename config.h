/*
 * Various configuration options for the motor control program. As much as possible all settings that are done
 * at compile time are stored here. 
 */

#ifndef CONFIG_H_
#define CONFIG_H_
#include <Arduino.h>

//The two values PWM_FREQ and MAX_PWM_DUTY should form an even divisor to the 84Mhz main clock
//The clock used can be from 1x divisor to 255 but will be integer increments
//The equation is 84Mhz / (Freq * Duty * 2)
//So, with 7KHz PWM we get 84Mhz / 7Khz = 12k. It divides evenly. 
//The max duty being 1000 means 12k turns into 12. Then, center aligned mode brings it down to 6.
//The register can take a value of 6 so this works out. 
#define PWM_FREQ			7000
#define MAX_PWM_DUTY		1000 //max value duty cycle can take
#define PWM_CLOCK			(PWM_FREQ * MAX_PWM_DUTY * 2ul) //number of PWM ticks per second - autocalculated - don't modify this line
#define PWM_PICO_PER_TICK	(1000000000ul / (PWM_CLOCK / 1000ul)) //Number of picoSeconds per PWM clock tick - trust me, it's relevant

//target dead time in nano seconds. So, 1000 is a micro second. You will need to tune this to the IGBT hardware.
#define PWM_TARGET_DEADTIME	1300 
//autocalculated from constant parameters. Don't change this, change the input parameters above
#define PWM_DEADTIME		(((PWM_TARGET_DEADTIME * 1000ul) / PWM_PICO_PER_TICK) + 1)

//If the PWM pulse would be shorter than this many nano seconds then suppress it. 
#define PWM_BUFFER_NANOS	1000 
//this is an autocalculated value based on a bunch of the above constants. Do not change this equation.
#define PWM_BUFFER			(((PWM_BUFFER_NANOS * 500ul) / PWM_PICO_PER_TICK) + (PWM_DEADTIME * 2))

//The ADC ports are 12 bits and so range from 0 to 4096 in value
//The current sensors rest around in the middle at 2013
//These below values form a window. If the ADC values of current sensor 1 goes outside this window the PWM fault line will trigger and PWM will shut down
//this forms a reasonably fast fault mechanism but it is not as reliable or fast as a hardware solution. But, if you're using a DMOC then the
//DMOC power hardware has hardware based faulting anyway. This is just a backup mechanism.
//The current sensors on a dmoc have a scaling factor of about 0.5 so each value here is 1/2 an amp. Plan accordingly.
#define ADC_CURR_LOWTHRESH	1100 //right around -453A
#define ADC_CURR_HIGHTHRESH	2900 //right around +447A

#define DRIVE_ENABLE		42
#define ADC_TEMP1			0
#define ADC_TEMP2			4
#define ADC_CURRENT1		1
#define ADC_CURRENT2		2
#define ADC_BUSV			3
#define ADC_MOTORTEMP1		5
#define ADC_MOTORTEMP2		7
#define DIN0				48
#define DIN1				49
#define DIN2				50
#define DIN3				51

#define EEPROM_PAGE			10

#define CFG_BUILD_NUM		1000

#define EEPROM_VER			0x14

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
	uint32_t motorTemp1Scale;//4
	uint32_t motorTemp1Bias; //4
	uint32_t motorTemp2Scale;//4	
	uint32_t motorTemp2Bias; //4

	
	uint8_t motorType; //1 - 0 = induction, 1 = permanent magnet (sine), 2 = BLDC (trapezoid)
	uint8_t controlType; //0 = Vhz, 1 = FOC
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
	uint16_t thetaOffset; //2 - offset for theta used for PMAC motors
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
	bool runningOffsetTest;
	bool rampingTest;
	bool rampingUp;
	uint16_t rampRPM;
};

struct OFFSET_TEST
{
	uint32_t testStart; //value when current test iteration started
	uint32_t currentOffset; //the current offset we're on
	uint32_t bestOffset; //best offset we've seen so far.
	uint32_t bestAccum; //largest positional offset found
	uint32_t posAccum; //stores the positional offset so far for this test.
};

extern EEPROMSettings settings;
extern volatile STATUS controllerStatus;

#endif

