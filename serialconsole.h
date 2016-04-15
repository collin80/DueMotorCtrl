#ifndef SERIAL_H_
#define SERIAL_H_

void serialInit();
void serialPrintMenu();
void serialRXChar(uint8_t chr);
void handleConsoleCmd();
void handleConfigCmd();
void handleShortCmd();
unsigned int parseHexCharacter(char chr);
unsigned int parseHexString(char *str, int length);

#endif