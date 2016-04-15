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

struct EEPROMSettings { //25 bytes so far. Keep under 256
	uint8_t version; //1
	uint32_t canSpeed; //4
	float busVoltageScale;//4
	float current1Scale;//4
	float current2Scale;//4
	float inverterTemp1Scale;//4
	float inverterTemp2Scale;//4
};

extern EEPROMSettings settings;

#endif