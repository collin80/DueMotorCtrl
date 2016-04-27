#include "pid.h"

PID::PID(int32_t p, int32_t i, int32_t d)
{
      kP = p;
      kI = i;
      kD = d;
      refVal = 0;
      inputValue = 0;
      iTerm = 0;
      lastValue = 0;
}

PID::PID(float p, float i, float d)
{
      kP = (int32_t)(p * FLOAT_TO_INT_SCALE);
      kI = (int32_t)(i * FLOAT_TO_INT_SCALE);
      kD = (int32_t)(d * FLOAT_TO_INT_SCALE);
}
  
void PID::setRef(int32_t pnt)
{
      refVal = pnt;
}
void PID::setMinValue(int32_t mn)
{
      minValue = mn;
      scaledMin = mn * FLOAT_TO_INT_SCALE;
}

void PID::setMaxValue(int32_t mx)
{
      maxValue = mx;
      scaledMax = mx * FLOAT_TO_INT_SCALE;
}

int32_t PID::calculatePID(int32_t input)
{
      int32_t output, error, diffInput;
      
      inputValue = input;
      error = refVal - inputValue;
      iTerm += (kI * error); //remember, all k* variables are scaled up
      
      if (iTerm > scaledMax) iTerm = scaledMax;
      if (iTerm < scaledMin) iTerm = scaledMin;
      
      diffInput = (inputValue - lastValue);
      
      output = ((kP * error) + (iTerm) - (kD * diffInput)) / FLOAT_TO_INT_SCALE;

      if (output > maxValue) output = maxValue;
      if (output < minValue) output = minValue;
      
      lastValue = inputValue;
      
      return output;
}
  
    
