#include "BSP.h"

void Variable_Init(void)
{
  QiStatus = Select;   //选择阶段
  memset(&QiConfig, 0, sizeof(struct QiConfig_Struct)); 
  memset(&PID, 0, sizeof(struct PID_Struct));
}

void Power_Up(void)
{
  LED1(LED_OFF);
  LED2(LED_ON);
  LED3(LED_OFF);
  Delay_ms(500);
  LED1(LED_ON);
  LED2(LED_OFF);
  LED3(LED_OFF);
  Delay_ms(500);
  LED1(LED_ON);
  LED2(LED_ON);
  LED3(LED_OFF);
  Delay_ms(500);
  LED1(LED_ON);
  LED2(LED_OFF);
  LED3(LED_OFF);
}

void BSP_Init(void)
{
  Variable_Init();
  
  UART_Init(115200);
  LED_Init();
  SysTick_Init();
  MagneticField_Init();
  ADC_Init();
  Decode_Init();
  PID_Init();
  Power_Up();
}


