#include "LED.h"


// LED: PA9 PA10

void LED_Init(void)
{
  // 配置LED的管脚
  IOCFG->PA9  = IOCFG_DEFAULT;
  IOCFG->PA10 = IOCFG_DEFAULT;
  GPIOA->DIR  = GPIOA->DIR | GPIO_PIN_9 | GPIO_PIN_10;
  
  LED1(LED_OFF);
  LED2(LED_OFF);
  
  printf("LED Init Success!\r\n");
}



