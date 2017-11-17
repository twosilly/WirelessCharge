#include "BSP.h"

void Variable_Init(void)
{
  QiStatus = Select;   //选择阶段
}

void BSP_Init(void)
{
  Variable_Init();
  
  UART_Init(57600);
  LED_Init();
  SysTick_Init();
  MagneticField_Init();
}


