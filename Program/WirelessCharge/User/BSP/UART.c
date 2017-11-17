#include "UART.h"

#pragma import(__use_no_semihosting_swi)

struct __FILE 
{
  int handle;        
};

FILE __stdout;

FILE __stdin;

int fputc(int ch, FILE *f) 
{
  while(UART0->SR & UART_SR_TXF);
  UART0->DR = ch;
  while(UART0->SR & UART_SR_TXNE);
  return ch;
}

int fgetc(FILE *f) 
{
  return 0;
}

void _ttywrch(int ch) 
{
  while(UART0->SR & UART_SR_TXF);
  UART0->DR = ch;
  while(UART0->SR & UART_SR_TXNE);
}

int ferror(FILE *f) 
{   
  return EOF;
}

void _sys_exit(int return_code) 
{
label:  goto label; 
}

void UART_Init(unsigned long bound)
{
  // 打开GPIOA和UART0的时钟
  SYSCON->SYSAHBCLKCTRL |= (SYSCON_SYSAHBCLKCTRL_GPIOAEN | SYSCON_SYSAHBCLKCTRL_UART0EN);
  
  // 配置UART0的管脚
  IOCFG->PA2 = IOCFG_DEFAULT |IOCFG_PA2_FUNC_TXD0;
  IOCFG->PA3 = IOCFG_DEFAULT |IOCFG_PA3_FUNC_RXD0;
  GPIOA->DIR = (GPIOA->DIR & (~GPIO_PIN_3)) | GPIO_PIN_2;
  
  SYSCON->PRESETCTRL &= ~SYSCON_PRESETCTRL_UART0_RST_N;
  SYSCON->PRESETCTRL |= SYSCON_PRESETCTRL_UART0_RST_N;   //复位UART0
  
  // 配置串口
  SYSCON->UART0CLKDIV = 1;
  UART0->BAUDDIV = MainClock / bound;
  UART0->CR = UART_CR_RXEN | UART_CR_TXEN;
  
  UART0->CR |= UART_ISR_RXNEINT;  //使能接收中断
  
	NVIC_SetPriority(UART0_IRQn, 1);//设置串口0中断通道的优先级
	NVIC_EnableIRQ(UART0_IRQn);			//使能串口0中断通道
  
  printf("\r\nUART0 baud frequncy:%ldbps \r\n", bound);
}

