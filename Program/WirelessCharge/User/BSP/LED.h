#ifndef __LED_H
#define __LED_H

#include "Zhuhc.h"

#define LED_ON  0
#define LED_OFF 1

// LED: PB3 PB4 PA15
#define LED1(a) do{if(a) GPIOB->SET = GPIO_PIN_3;  else GPIOB->CLR = GPIO_PIN_3;}  while(0)
#define LED2(a) do{if(a) GPIOB->SET = GPIO_PIN_4;  else GPIOB->CLR = GPIO_PIN_4;}  while(0)
#define LED3(a) do{if(a) GPIOA->CLR = GPIO_PIN_15; else GPIOA->SET = GPIO_PIN_15;} while(0)

void LED_Init(void);

#endif


