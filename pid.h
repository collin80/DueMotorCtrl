#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

#define FLOAT_TO_INT_SCALE	(1 << 12)  //scale up by 4096

class PID
{
public:
    PID(int p, int i, int d);
    PID(float p, float i, float d);

    void setCoeffs(int32_t p, int32_t i, int32_t d);
    void setCoeffs(float p, float i, float d);
    void setRef(int32_t pnt);
    void setMinValue(int32_t mn);
    void setMaxValue(int32_t mx);
    int32_t calculatePID(int32_t input);


protected:

private:
    int32_t kP;
    int32_t kI;
    int32_t kD;
    int32_t refVal;
    int32_t inputValue;
    int32_t lastValue;
    int32_t iTerm;
    int32_t minValue;
    int32_t maxValue;
    int32_t scaledMin;
    int32_t scaledMax;
};

#endif

