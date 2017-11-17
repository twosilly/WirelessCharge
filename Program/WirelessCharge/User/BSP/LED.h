#ifndef __LED_H
#define __LED_H

#include "Zhuhc.h"

#define LED_ON  0
#define LED_OFF 1

// LED: PA9 PA10
#define LED1(a) do{if(a) GPIOA->SET = GPIO_PIN_9;  else GPIOA->CLR = GPIO_PIN_9; } while(0)
#define LED2(a) do{if(a) GPIOA->SET = GPIO_PIN_10; else GPIOA->CLR = GPIO_PIN_10;} while(0)


void LED_Init(void);

#endif


