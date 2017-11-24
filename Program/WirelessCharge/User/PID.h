#ifndef __PID_H
#define __PID_H


#include "Zhuhc.h"

struct PID_Struct
{
  float P_Value;
  float I_Value;
  float D_Value;
  float IntegralSum;
  float IntegralMax;
  signed char ControlError;
  float ErrorValue;
  float LastErrorValue;
  float PIDResult;
  float PIDResultMax;
  float Result;
  float LastResult;
  float TimeInner;
  unsigned long TimeControl;
  float Frequency;
  float CurrentValue;
  unsigned char PIDCount;
};

extern struct PID_Struct PID;

void PID_Init(void);
void PID_Control(void);

#endif


