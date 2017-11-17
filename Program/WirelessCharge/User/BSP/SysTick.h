#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "Zhuhc.h"

void SysTick_Init(void);
uint32_t micros(void);
uint32_t millis(void);
void Delay_ms(uint16_t nms);
void Delay_us(uint32_t nus);
void Get_Time(unsigned long *time);

extern volatile uint32_t sysTickUptime;

#endif


