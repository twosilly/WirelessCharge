#include "Control.h"

#define CheckObject_Value       (0x020)
#define PoweTransferr_Value     (0x180)
/*
 * Selection阶段:
 *     在这个阶段，功率发送器通常监测接口表面物体的放置和移除。如果电源发送器发现一个或多个对象，
 *     如果它支持自由定位，它应该尝试定位这些对象。此外，功率发送器可以尝试区分功率接收器和异物，
 *     如钥匙、硬币等。此外，功率发送器应尝试选择用于功率传输的功率接收器。
 */
volatile uint32_t SelectTime = 0, PingTime = 0, IAndCTime = 0, RealTimeCheckTime = 0, LEDControlTime = 0;
volatile uint32_t PowerTransferTime_Error = 0, PowerTransferTime_Power = 0, ADConvertTime = 0;
volatile uint32_t VoltageCheckTime = 0;
  
unsigned char ErrorCode = 0;
bool isEndPowerTransfer = false;
unsigned char Identification_Count = 0;

bool isLowVoltage = false;

void SelectControl(void)
{
  if(micros() - SelectTime > 500000)  //500ms
  {
    SelectTime = micros();
    
    I_Sense_Count = 0;
    GPIOA->IE |= GPIO_PIN_1; //开启中断
    MagneticField_Enable();
    Delay_ms(10);
    if(I_Sense_Count >= 2)
    {
      QiStatus = Ping;
      PingTime = micros();
      MagneticField_Disable();
      GPIOA->IE |= GPIO_PIN_5;
      GPIOA->IE &= ~GPIO_PIN_1;
    }
    I_Sense_Count = 0;
    MagneticField_Disable();
//    Get_AllAdc();
//    if(AD_I_OUT > CheckObject_Value)
//    {
//      Delay_ms(5);
//      Get_AllAdc();
//      if(AD_I_OUT > CheckObject_Value)
//      {
//        Delay_ms(5);
//        Get_AllAdc();
//        if(AD_I_OUT > CheckObject_Value)
//        {
//          QiStatus = Ping;
//          PingTime = micros();
//          MagneticField_Disable();
//          GPIOA->IE |= GPIO_PIN_5;
//          //LED3(LED_ON);
//        }
//      }
//    }
  }
  else
  {
    MagneticField_Disable();
  }
}

void PingControl(void)
{
  if(micros() - PingTime < Time_Ping_Target)
  {
    //LED1(LED_ON);
    MagneticField_Enable();
  }
  else if(micros() - PingTime < (Time_Ping_Target + Time_Terminate_Max))
  {
    if(rightDone == 1)
    {
      if(rightData[0] == Header_SignalStrength)
      {
        //接收到信号强度包
        printf("SignalStrength:%d\r\n", rightData[1]);
        QiStatus = IdentificationAndConfiguration;
        IAndCTime = micros();
      }
      else
      {
        if(rightData[0] == Header_EndPowerTransfer)
        {
          printf("EndPowerTransfer:%d\r\n", rightData[1]);
          ErrorCode = rightData[1];
          isEndPowerTransfer = true;
        }
        MagneticField_Disable();
        //LED1(LED_OFF);
        MagneticField_Disable();
        QiStatus = Select;
        SelectTime = micros();
      }
      
      rightDone = 0;
    }
  }
  else
  { 
    //LED1(LED_ON);
    //LED2(LED_OFF);
    //LED3(LED_OFF);
    MagneticField_Disable();
    QiStatus = Select;
    SelectTime = micros();
  } 
}

void ConfigControl(void)
{
  if(micros() - IAndCTime < Time_NextPacket_Max)
  {
    
  }
  else if(micros() - IAndCTime < (Time_PacketLength_Max + Time_NextPacket_Max))
  {
    if(micros() - IAndCTime > (Time_NextPacket_Max + Time_Terminate_Max))
    {
      if(startData == 0)
      {
        Identification_Count = 0;
        //LED1(LED_OFF);
        MagneticField_Disable();
        QiStatus = Select;
        SelectTime = micros();
      }
    }
  
    if(rightDone == 1)
    {
      
      if(rightData[0] != Header_Configuration)
      {
        printf("X:%X %X\r\n", rightData[0], rightData[1]);
        if(rightData[0] == Header_Identification)
        {
          printf("\r\nVersion:V%d.%d\r\n\r\n", ((rightData[1]&0xF0)>>4), (rightData[1]&0x0F));
        }
        else
        {
          Identification_Count++;
        }
        IAndCTime = micros();
      }
      else
      {
        //配置
        //printf("Config:%d %d\r\n", rightData[0], rightData[1]);
        //得到配置参数
        //printf("%X %X %X %X %X\r\n", rightData[1], rightData[2], rightData[3], rightData[4], rightData[5]);
        
        QiConfig.PowerClass        = rightData[1] >> 6;
        QiConfig.MaximumPowerValue = rightData[1] & 0x3F;
        QiConfig.Reserved1         = rightData[2];
        QiConfig.Prop              = rightData[3] >> 7;
        QiConfig.Reserved2         = (rightData[3] >> 3) & 0xF;
        QiConfig.Count             = rightData[3] & 0x7;
        QiConfig.WindowSize        = rightData[4] >> 3;
        QiConfig.WindowOffset      = rightData[4] & 0x7;
        QiConfig.Neg               = rightData[5] >> 7;
        QiConfig.Polarity          = (rightData[5] >> 6) & 0x1;
        QiConfig.Depth             = (rightData[5] >> 4) & 0x3;
        QiConfig.Reserved3         = rightData[5] & 0xF;
        
        printf("PCls: %d\r\n", QiConfig.PowerClass);
        printf("MaxPV:%d\r\n", QiConfig.MaximumPowerValue);
        printf("Prop: %d\r\n", QiConfig.Prop);
        printf("Count:%d\r\n", QiConfig.Count);
        printf("Size: %d\r\n", QiConfig.WindowSize);
        printf("Off:  %d\r\n", QiConfig.WindowOffset);
        printf("Neg:  %d\r\n", QiConfig.Neg);
        printf("Polar:%d\r\n", QiConfig.Polarity);
        printf("Depth:%d\r\n", QiConfig.Depth);
        printf("Res1: %d\r\n", QiConfig.Reserved1);
        printf("Res2: %d\r\n", QiConfig.Reserved2);
        printf("Res3: %d\r\n\r\n", QiConfig.Reserved3);
          
        if((QiConfig.PowerClass == 0x0) && (QiConfig.Reserved1 == 0) &&
           (QiConfig.Reserved2 == 0)    && (QiConfig.Reserved3 == 0) &&
           (QiConfig.Count == Identification_Count) )
        {
          Identification_Count = 0;
          QiStatus = PowerTransfer;
          PowerTransferTime_Error = micros();  
          PowerTransferTime_Power = PowerTransferTime_Error;
          PID_Init();
          PID.Frequency = 175000;
          PWM->CMOD = 143; 
          PWM->VAL0 = 71;
          //LED1(LED_OFF);
          //LED2(LED_ON);
          //LED3(LED_ON);
        }
        else
        {
          printf("Config Error!\r\n");
          Identification_Count = 0;
          //LED1(LED_ON);
          //LED2(LED_OFF);
          //LED3(LED_OFF);
          MagneticField_Disable();
          QiStatus = Select;
          SelectTime = micros();
        }
      }
      
      if(Identification_Count > 7)
      {
        Identification_Count = 0;
        //LED1(LED_ON);
        //LED2(LED_OFF);
        //LED3(LED_OFF);
        MagneticField_Disable();
        QiStatus = Select;
        SelectTime = micros();
      }
      startData = 0;
      rightDone = 0;
    }
  }
  else
  {
    Identification_Count = 0;
    //LED1(LED_ON);
    //LED2(LED_OFF);
    //LED3(LED_OFF);
    MagneticField_Disable();
    QiStatus = Select;
    SelectTime = micros();
  }
}

void PowerTransferControl(void)
{
  if(rightDone == 1)
  {
    if(rightData[0] == Header_EndPowerTransfer)
    {
      printf("EndPower:%d\r\n", rightData[1]);
      MagneticField_Disable();
      rightDone = 0;
      //while(1);
    } 
  }

  if(micros() - PowerTransferTime_Error < Time_ControlActive_Target)
  {
    //PID控制
    //if((micros() - PowerTransferTime_Error > PID.TimeControl) && !isLowVoltage)
    if(micros() - PowerTransferTime_Error > PID.TimeControl)
    {
      PID.TimeControl += PID.TimeInner * 1000 * 1000;
      PID.PIDCount++;
      
      PID.Frequency += PID.Result / ((float)Time_ControlActive_Target / 1000 / 1000 / PID.TimeInner);
      //printf("%f\r\n", ((float)Time_ControlActive_Target / 1000 / 1000 / PID.TimeInner));
      if(PID.Frequency < 110000)
      {
        PWM->CMOD = 227; 
        PWM->VAL0 = 227 / 2;
        PID.Frequency = 110000;
      }
      else if(PID.Frequency > 205000)
      {
        PWM->CMOD = 122; 
        PWM->VAL0 = 122 / 2;
        PID.Frequency = 205000;
      }
      else
      {
        PWM->CMOD = (unsigned short)(25000000 / PID.Frequency + 0.5);
        PWM->VAL0 = PWM->CMOD / 2;
      }
    }
  }
  else if(micros() - PowerTransferTime_Error < (Time_ControlActive_Target + Time_ControlSettling_Target))
  {
    //等待稳定
    PID.TimeControl = 0;
  }
  else if(micros() - PowerTransferTime_Error < 
    (Time_ControlActive_Target + Time_ControlSettling_Target + Time_ControlError_Target))
  {
    //接收数据
    if(rightDone == 1)
    {
      if(rightData[0] == Header_ControlError)
      {
        printf("Error:%d\r\n", (signed char)rightData[1]);
        PID.ControlError = (signed char)rightData[1];
        //printf("PID:%d\r\n", PID.PIDCount);
        PID_Control();
        PowerTransferTime_Error = micros();
        rightDone = 0;
      } 
    }
  }
  else
  {
    printf("Error Overtime!\r\n");
    //LED1(LED_ON);
    //LED2(LED_OFF);
    //LED3(LED_OFF);
    MagneticField_Disable();
    QiStatus = Select;
    SelectTime = micros();
    PID.Frequency = 175000;
    PWM->CMOD = 143; 
    PWM->VAL0 = 71;
  }
  
  if(micros() - PowerTransferTime_Power < Time_ReceivedPower_Target)
  {
    //接收功率信息
    if(rightDone == 1)
    {
      if(rightData[0] == Header_8BitReceivedPower)
      {
        printf("-----Power:%d\r\n", rightData[1]);
        PowerTransferTime_Power = micros();
        rightDone = 0;
      } 
    }
  }
  else
  {
    printf("Power Overtime!\r\n");
    //LED1(LED_ON);
    //LED2(LED_OFF);
    //LED3(LED_OFF);
    MagneticField_Disable();
    QiStatus = Select;
    SelectTime = micros();
    PID.Frequency = 175000;
    PWM->CMOD = 143; 
    PWM->VAL0 = 71;
  }
}

void AD_Convert(void)
{
  static unsigned long AD_I_OUT_SUM = 0, AD_S_OVP_SUM, AD_V_IN_SUM;
  static unsigned char AD_Count = 0;
  if(micros() - ADConvertTime > 1000) 
  {
    ADConvertTime = micros();
    Get_AllAdc();
    AD_I_OUT_SUM += AD_I_OUT;
    AD_S_OVP_SUM += AD_S_OVP;
    AD_V_IN_SUM  += AD_V_IN;
    AD_Count++;
    if(AD_Count >= 5)
    {
      AD_I_OUT_Average = AD_I_OUT_SUM / 5;
      AD_S_OVP_Average = AD_S_OVP_SUM / 5;
      AD_V_IN_Average  = AD_V_IN_SUM  / 5;
      AD_Count = 0;
      AD_I_OUT_SUM = 0;
      AD_S_OVP_SUM = 0;
      AD_V_IN_SUM  = 0;
    }
  }
}

void RealTimeCheck(void)
{
  static unsigned char RealTimeCheck_Count = 0;
  static unsigned long ADAverage = 0;
  if(micros() - RealTimeCheckTime > 10000)  //10ms
  {
    RealTimeCheckTime = micros();
    if(QiStatus != Select)
    {
      Get_AllAdc();
      
      ADAverage += AD_I_OUT;
      RealTimeCheck_Count++;

      if(RealTimeCheck_Count >= 10)
      {
        ADAverage = (ADAverage + 5) / 10;
        printf("%ld\r\n", ADAverage);
        if(ADAverage < PoweTransferr_Value)
        {
//          RealTimeCheck_Count = 0;
//          LED1(LED_OFF);
//          LED2(LED_OFF);
//          MagneticField_Disable();
//          QiStatus = Select;
//          SelectTime = micros();
//          printf("Device Remove!\r\n");
        }
        ADAverage = 0;
        RealTimeCheck_Count = 0;
      }
    }
    else 
    {
      ADAverage = 0;
      RealTimeCheck_Count = 0;
    }
  }
}

void Voltage_Check(void)
{
  if(micros() - VoltageCheckTime > 30000)  //30ms
  {
    VoltageCheckTime = micros();
    //printf("%d\r\n", AD_S_OVP_Average);
    if(AD_V_IN_Average < VIN_4V5_AD_VALUE)
      isLowVoltage = true;
    else
      isLowVoltage = false;
  }
}

void LED_Control(void)
{
  static int LEDCount = 0; 
  if(micros() - LEDControlTime > 10000)  //10ms
  {
    LEDControlTime = micros();
    LEDCount++;
    
    if(QiStatus == Select)
    {
      LED1(LED_ON);   //绿
      LED2(LED_OFF);  //红
      LED3(LED_OFF);
    }
    else if(QiStatus == Ping)
    {
      LED1(LED_ON);
      LED2(LED_OFF);
      LED3(LED_ON);
    }
    else if(QiStatus == IdentificationAndConfiguration)
    {
      LED1(LED_ON);
      LED2(LED_OFF);
      LED3(LED_ON);
    }
    else if(QiStatus == PowerTransfer)
    {
      if(AD_V_IN_Average < VIN_9V_AD_VALUE)
        LED1(LED_OFF);
      else
        LED1(LED_ON);
      LED2(LED_ON);
      if(isLowVoltage)
      {
        if(LEDCount % 100 < 50)
          LED3(LED_ON);
        else
          LED3(LED_OFF);;
      }
      else
      {
        LED3(LED_ON);
      }
      
     // printf("%d\r\n", AD_V_IN_Average);
    }
  }
}

void QiControl(void)
{
  SelectTime = micros();
  RealTimeCheckTime = micros();
  LEDControlTime = micros();
  VoltageCheckTime = micros();
  while(1)
  {
    if(QiStatus == Select)
    {
      SelectControl();
    }
    else if(QiStatus == Ping)
    { 
      PingControl();
    }
    else if(QiStatus == IdentificationAndConfiguration)
    {
      ConfigControl();
    }
    else if(QiStatus == PowerTransfer)
    {
      PowerTransferControl();
    }
    
    AD_Convert();
    Decode();
    LED_Control();
    Voltage_Check();
    //RealTimeCheck();
  }
}
