#include <Arduino.h>
#include "encoder.h"

  //REG_TC0_CV0 Stores count from encoder
  //REG_TC0_CV1 Stores count from index if interrupts are off

volatile int z_Total=0;
  
void setup_encoder()
{
  // Setup Quadrature Encoder with Marker
  REG_PMC_PCER0 = (1<<27); // activate clock for TC0
  REG_PMC_PCER0 = (1<<28); // activate clock for TC1

  // select XC0 as clock source and set capture mode
  REG_TC0_CMR0 = (1<<0)|(1<<2)|(1<<8)|(1<<10);

  // activate quadrature encoder and position measure mode, no filters
  REG_TC0_BMR = (1<<8)|(1<<9)|(1<<12);

  // enable the clock (CLKEN=1) and reset the counter (SWTRG=1) 
  REG_TC0_CCR0 = 5;  
  REG_TC0_CCR1 = 5;

  //Remark out the next 4 lines to remove index interrupt and
  // accumulate index count
  REG_TC0_CMR1 = (1<<8); // Set rising edge of Z
  REG_TC0_IER1=0b10000000; // enable interrupt on Z
  REG_TC0_IDR1=0b01111111; // disable other interrupts
  NVIC_EnableIRQ(TC1_IRQn);
}

int getEncoderCount()
{
   int value = REG_TC0_CV0;
   return value;
}

void TC1_Handler() {

 z_Total++;
 long dummy=REG_TC0_SR1; // vital - reading this clears some flag
 // otherwise you get infinite interrupts
} 