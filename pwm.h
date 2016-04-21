#ifndef PWM_H_
#define PWM_H_
void setup_pwm();
void updatePWM(unsigned int a, unsigned int b, unsigned int c);
void updatePosVHz();
void setVHzSpeed(int targetRPM);
#endif