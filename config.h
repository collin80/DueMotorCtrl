/*
 * Various configuration options for the motor control program. As much as possible all settings that are done
 * at compile time are stored here. 
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#define PWM_FREQ		10000
#define MAX_PWM_DUTY		1050 //max value duty cycle can take
#define PWM_BUFFER		50 //if the value for PWM is within this number from either 0 or MAX then set it to either 0 or MAX. Prevents very short PWM pulses.
#define PWM_TARGET_DEADTIME	600 //target dead time in nano seconds. So, 1000 is a micro second

#define DRIVE_ENABLE		42

#define EEPROM_PAGE		10

#define CFG_BUILD_NUM		1000

struct EEPROMSettings { //37 bytes so far. Keep under 256
	uint8_t version; //1
	uint8_t logLevel; //1
	uint32_t canSpeed; //4
	uint32_t canBaseAddr; //4
	uint32_t busVoltageScale;//4 - scale value is 65535 at 1 to 1.
	uint32_t current1Scale;//4
	uint32_t current2Scale;//4
	uint32_t inverterTemp1Scale;//4
	uint32_t inverterTemp2Scale;//4	
	uint16_t encoderCount; //2
	uint8_t encoderDirection; //1
	uint16_t maxRPM; //2 - whole RPMS 
	uint16_t maxTorque; //2 - in tenths of a Nm
};

extern EEPROMSettings settings;

#endif