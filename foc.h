#ifndef FOC_H_
#define FOC_H_
void setupFOC();
void updateFOC();
void setDesiredTorque(int16_t torq);
void startFOCOffsetTest();
void sendCANMsgs();
#endif

