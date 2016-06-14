#include "config.h"
#include "dig_in.h"

int digPins[] = { DIN0, DIN1, DIN2, DIN3 };

//the dimensions are AB first, then BC, then CA [0] is when it is off, [1] is when it is on.
//returned value is sector number where sector 1  is based at 0 and each one up is 60 degrees more
//sector 0 is special and is an error state
int hallToSector[2][2][2] = 
{
	{ //AB off
		{5, 6}, //BC off
		{0, 1}  //BC On
	},
	{ //AB on
		{4, 0}, //BC off
		{3, 2} //BC on
	} 
};

void setup_digital_inputs()
{
	for (int x = 0; x < 4; x++) pinMode(digPins[x], INPUT);
}

bool getDigitalInput(int input)
{
	if (input < 0) return false;
	if (input > 3) return false;
	return (!digitalRead(digPins[input]));
}

int getMotorSector()
{
	int A = 0, B = 0, C = 0;
	if (getDigitalInput(settings.hallAB)) A = 1;
	if (getDigitalInput(settings.hallBC)) B = 1;
	if (getDigitalInput(settings.hallCA)) C = 1;
	return hallToSector[A][B][C];
}
