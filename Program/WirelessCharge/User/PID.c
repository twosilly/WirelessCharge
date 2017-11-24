#include "PID.h"

struct PID_Struct PID;

void PID_Init(void)
{
  PID.TimeControl  = 0;
  PID.TimeInner    = 0.005;      //5ms
  PID.P_Value      = 9.0;
  PID.I_Value      = 0.04;
  PID.D_Value      = 0.1;
  PID.IntegralSum  = 0;
  PID.IntegralMax  = 3000;   //积分限幅
  PID.PIDResultMax = 20000;  //PID限幅
  PID.Frequency    = 175000; //175KHz
  PID.ControlError = 0;
  PID.ErrorValue   = 0;
  PID.PIDResult    = 0;
  PID.Result       = 0;
  PID.LastResult   = 0;
  PID.CurrentValue = 0;
  PID.PIDCount     = 0;
  PID.LastErrorValue = 0;
}

//Control Error有符号数 -128 --- 127
void PID_Control(void)
{
  //Get_AllAdc();
  PID.LastErrorValue = PID.ErrorValue;
  PID.ErrorValue = AD_I_OUT_Average * 0.8 * ((float)(PID.ControlError) / 128);
  //printf("AD:%d\n", AD_I_OUT_Average);
  PID.IntegralSum += PID.I_Value * PID.ErrorValue * PID.TimeInner;
  if(PID.IntegralSum > PID.IntegralMax)
    PID.IntegralSum = PID.IntegralMax;
  if(PID.IntegralMax < -PID.IntegralMax)
    PID.IntegralSum = -PID.IntegralMax;
  
  PID.PIDResult = PID.P_Value * PID.ErrorValue 
                + PID.IntegralSum 
                + PID.D_Value * (PID.ErrorValue - PID.LastErrorValue) / PID.TimeInner;
  
  if(PID.PIDResult > PID.PIDResultMax)
    PID.PIDResult = PID.PIDResultMax;
  if(PID.PIDResult < -PID.PIDResultMax)
    PID.PIDResult = -PID.PIDResultMax;
  
  PID.LastResult = PID.Result;
  if(PID.Frequency <= 140000)
    PID.Result = PID.LastResult - PID.PIDResult * 1;//1.5
  else if(PID.Frequency <= 160000)
    PID.Result = PID.LastResult - PID.PIDResult * 2;
  else if(PID.Frequency <= 180000)
    PID.Result = PID.LastResult - PID.PIDResult * 3;
  else if(PID.Frequency <= 205000)
    PID.Result = PID.LastResult - PID.PIDResult * 5;
  
  PID.PIDCount = 0;
#if 0
  //虚拟示波器
  float test = PID.Frequency / 1000;
  CX_SendWare(&test, 4);
#endif
}


