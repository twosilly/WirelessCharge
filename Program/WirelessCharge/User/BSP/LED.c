#include "LED.h"


// LED: PB3 PB4 PA15

void LED_Init(void)
{
  // 配置LED的管脚
  IOCFG->PB3 = IOCFG_DEFAULT;
  IOCFG->PB4 = IOCFG_DEFAULT;
  GPIOB->DIR  = GPIOA->DIR | GPIO_PIN_3 | GPIO_PIN_4;
  
  IOCFG->PA15 = IOCFG_DEFAULT;
  GPIOA->DIR  = GPIOA->DIR | GPIO_PIN_15;
  
  LED1(LED_OFF);
  LED2(LED_OFF);
  LED3(LED_OFF);
  
  printf("LED Init Success!\r\n");
}



